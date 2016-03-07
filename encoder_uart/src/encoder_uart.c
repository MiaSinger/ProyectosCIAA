/* Copyright 2014, Mariano Cerdeiro
 * Copyright 2014, Pablo Ridolfi
 * Copyright 2014, Juan Cecconi
 * Copyright 2014, Gustavo Muro
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Blinking_echo example source file
 **
 ** This is a mini example of the CIAA Firmware to test the periodical
 ** task excecution and serial port funcionality.
 ** To run this sample in x86 plataform you must enable the funcionality of
 ** uart device setting a value of une or more of folowing macros defined
 ** in header file modules/plataforms/x86/inc/ciaaDriverUart_Internal.h
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Blinking Blinking_echo example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * MaCe         Mariano Cerdeiro
 * PR           Pablo Ridolfi
 * JuCe         Juan Cecconi
 * GMuro        Gustavo Muro
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20141019 v0.0.2   JuCe add printf in each task,
 *                        remove trailing spaces
 * 20140731 v0.0.1   PR   first functional version
 */

/*==================[inclusions]=============================================*/
#include "chip.h"             /* <= chip header */

#include "os.h"               /* <= operating system header */
#include "ciaaPOSIX_stdio.h"  /* <= device handler header */
#include "ciaaPOSIX_string.h" /* <= string header */
#include "ciaak.h"            /* <= ciaa kernel header */
#include "encoder_uart.h"         /* <= own header */

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/* Error de lectura del encoder */
#define ENC_ERROR 0

/* Configurar un pin de GPIO como entrada */
void configurar_entrada(uint8_t pinNamePort,uint8_t pinNamePin,uint8_t func,uint8_t gpioPort,uint8_t gpioPin);

/* Obtener el estado de un pic de GPIO declarado como entrada */
uint8_t get_pin_value(uint8_t gpioPort,uint8_t gpioPin);

/* Leer las dos entradas que vienen del encoder */
void leer_entradas_encoder(uint8_t *entrada_1, uint8_t *entrada_2);

/* Obtiene el estado de las lecturas del encoder */
int encoder(uint8_t encoder_in);

/* Obtener valor de la matriz segun entradas */
uint8_t obtener_valor_matriz(uint8_t entrada_1, uint8_t entrada_2);

/* Enviar dato por UART */
void send_uart(uint8_t data_1, uint8_t data_2);


/** \brief File descriptor for digital output ports
 *
 * Device path /dev/dio/out/0
 */
static int32_t fd_out;

/** \brief File descriptor of the USB uart
 *
 * Device path /dev/serial/uart/1
 */
static int32_t fd_uart1;

uint8_t pulsos = 0;
uint8_t vueltas_signo = 0;
uint8_t signo = 0; // 0 vueltas positivas, 1 vueltas negativas (al lado contrario)
uint8_t vueltas = 0;

int previo = 0, actual = 0, inc;


/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */
int main(void)
{
   /* Starts the operating system in the Application Mode 1 */
   /* This example has only one Application Mode */
   StartOS(AppMode1);

   /* StartOs shall never returns, but to avoid compiler warnings or errors
    * 0 is returned */
   return 0;
}

/** \brief Error Hook function
 *
 * This fucntion is called from the os if an os interface (API) returns an
 * error. Is for debugging proposes. If called this function triggers a
 * ShutdownOs which ends in a while(1).
 *
 * The values:
 *    OSErrorGetServiceId
 *    OSErrorGetParam1
 *    OSErrorGetParam2
 *    OSErrorGetParam3
 *    OSErrorGetRet
 *
 * will provide you the interface, the input parameters and the returned value.
 * For more details see the OSEK specification:
 * http://portal.osek-vdx.org/files/pdf/specs/os223.pdf
 *
 */
void ErrorHook(void)
{
   ciaaPOSIX_printf("ErrorHook was called\n");
   ciaaPOSIX_printf("Service: %d, P1: %d, P2: %d, P3: %d, RET: %d\n", OSErrorGetServiceId(), OSErrorGetParam1(), OSErrorGetParam2(), OSErrorGetParam3(), OSErrorGetRet());
   ShutdownOS(0);
}

/** \brief Initial task
 *
 * This task is started automatically in the application mode 1.
 */
TASK(InitTask)
{
   /* init CIAA kernel and devices */
   ciaak_start();

   ciaaPOSIX_printf("Init Task...\n");

   //GPIO8
   configurar_entrada(6,12,FUNC0,2,8);
   //GPIO7
   configurar_entrada(6,11,FUNC0,3,7);

   /* open CIAA digital outputs */
   fd_out = ciaaPOSIX_open("/dev/dio/out/0", ciaaPOSIX_O_RDWR);

   /* open UART connected to USB bridge (FT2232) */
   fd_uart1 = ciaaPOSIX_open("/dev/serial/uart/1", ciaaPOSIX_O_RDWR);

   /* change baud rate for uart usb */
   ciaaPOSIX_ioctl(fd_uart1, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)ciaaBAUDRATE_115200);

   /* change FIFO TRIGGER LEVEL for uart usb */
   ciaaPOSIX_ioctl(fd_uart1, ciaaPOSIX_IOCTL_SET_FIFO_TRIGGER_LEVEL, (void *)ciaaFIFO_TRIGGER_LEVEL3);

   /* activate example tasks */
   SetRelAlarm(ActivatePeriodicTask, 200, 200);

   /* Activates the SerialEchoTask task */
   ActivateTask(SerialEchoTask);

   /* end InitTask */
   TerminateTask();
}



/** \brief Periodic Task
 *
 * This task is activated by the Alarm ActivatePeriodicTask.
 * This task copies the status of the inputs bits 0..3 to the output bits 0..3.
 * This task also blinks the output 4
 */
TASK(PeriodicTask)
{
  /*Lectura de encoder*/
  uint8_t entrada_1, entrada_2;
  leer_entradas_encoder(&entrada_1, &entrada_2);
  uint8_t encoder_in = obtener_valor_matriz(entrada_1, entrada_2);

  int incremento = encoder(encoder_in);

  if(incremento!=0){
    if(incremento>0){
       if(pulsos==32){
         pulsos = 0;

         if(vueltas_signo == 0) {
            vueltas++;
         } else {
            if(vueltas == 1) {
              vueltas_signo = 0;
              vueltas = 0;
              signo = 0;
            } else {
              vueltas--;
            }
         }
       }
       else pulsos++;
    } else {
       if(pulsos==0){
         pulsos=32;

        if(vueltas_signo == 0) {
          if(vueltas == 0) {
            vueltas_signo = 1;
            vueltas = 1;
            signo = 1;
          } else {
            vueltas--;
          }
        } else {
            vueltas++;
        }
       }
       else pulsos--;
    }
  }
  //pulsos += incremento;

  //vueltas = pulsos / 32;

  uint8_t data = (uint8_t) pulsos;
  //uint8_t data2 = (uint8_t) vueltas;
  //send_uart((uint8_t) incremento);
  //send_uart(data);
  //send_uart(data2);


   /* variables to store input/output status */
   uint8_t outputs = 0;

   /* read outputs */
   ciaaPOSIX_read(fd_out, &outputs, 1);

   /* blink */
   outputs ^= 0x10;

   /* write */
   ciaaPOSIX_write(fd_out, &outputs, 1);

  /* terminate task */
  TerminateTask();
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/


/** \brief Serial Echo Task
 *
 * This tasks waits for input data from fd_uart1 and writes the received data
 * to fd_uart1 and fd_uart2. This taks alos blinkgs the output 5.
 *
 */
TASK(SerialEchoTask)
{
   int8_t buf[20];   /* buffer for uart operation              */
   uint8_t outputs;  /* to store outputs status                */
   int32_t ret;      /* return value variable for posix calls  */

   ciaaPOSIX_printf("SerialEchoTask...\n");

   while(1)
   {
      /* wait for any character ... */
      ret = ciaaPOSIX_read(fd_uart1, buf, 1);

      if(ret > 0)
      {

        if(buf[0] == 112) { // If "p" sends pulsos vueltas y signo de vueltas
          buf[0] = pulsos;
          buf[1] = vueltas;
          buf[2] = signo;

          /* ... and write them to the same device */
          ciaaPOSIX_write(fd_uart1, buf, 3);
        }
      }
   }
}

/*
 * Configurar Entrada
 */
void configurar_entrada(uint8_t pinNamePort,uint8_t pinNamePin,uint8_t func,uint8_t gpioPort,uint8_t gpioPin) {
    // Configuración como INPUT
     Chip_SCU_PinMux(
        pinNamePort,
        pinNamePin,
        SCU_MODE_PULLDOWN | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS,
        func
     );

     uint8_t INPUT = 0;

     Chip_GPIO_SetDir(LPC_GPIO_PORT, gpioPort, (1 << gpioPin), INPUT);
}

/*
 * Enviar dato por UART
 */
 void send_uart(uint8_t data_1, uint8_t data_2) {
   /* buf = buffer para UART */
   int8_t buf[2];
   buf[0] = data_1;
   buf[1] = data_2;

   ciaaPOSIX_write(fd_uart1, buf, 2);
 }

/*
 * Obtener estado de entrada
 */
uint8_t get_pin_value(uint8_t gpioPort, uint8_t gpioPin) {
  uint8_t value = Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, gpioPort, gpioPin);
  return value;
}

/*
 * Función de lectura de los datos del encoder
 */
int encoder(uint8_t encoder_in) {
  /* Matriz para identificar el sentido de giro del encoder */
  static int M_inc[4][4] = {{0,      1,         -1,        ENC_ERROR},
                            {-1,      0,         ENC_ERROR, 1        },
                            {1,       ENC_ERROR, 0,        -1        },
                            {ENC_ERROR,-1,       1,         0         }
  };

  // Actualizacion
  previo = actual;
  actual = encoder_in;

  // Detectamos el incremento (si lo hay)
  inc = M_inc[previo][actual];

  // Regresamos el cambio (direccion o error)
  return inc;
}

/*
 * Leer las entradas del encoder
 */
void leer_entradas_encoder(uint8_t *entrada_1, uint8_t *entrada_2) {
  *entrada_1 = get_pin_value(2, 8);
  *entrada_2 = get_pin_value(3, 7);
}

/*
 * Obtiene un valor de la matriz segun las entradas
 */
uint8_t obtener_valor_matriz(uint8_t entrada_1, uint8_t entrada_2) {
  static uint8_t M_gray[2][2] = {{0,2},{1,3}};
  return M_gray[entrada_1][entrada_2];
}
