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
 ** This is a mini example of the CIAA Firmware.
 **
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
 * ErPe         Eric Pernia
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20150603 v0.0.3   ErPe change uint8 type by uint8_t
 *                        in line 172
 * 20141019 v0.0.2   JuCe add printf in each task,
 *                        remove trailing spaces
 * 20140731 v0.0.1   PR   first functional version
 */

/*==================[inclusions]=============================================*/

/* Si ponemos esto a lo ultimo se rompe WTF! */
#include "chip.h"

#include "os.h"               /* <= operating system header */
#include "ciaaPOSIX_stdio.h"  /* <= device handler header */
#include "ciaaPOSIX_string.h" /* <= string header */
#include "ciaak.h"            /* <= ciaa kernel header */
#include "encoder.h"          /* <= own header */

static int32_t fd_out;
static int32_t fd_in;

uint8_t value = 0;

int speed = 0;
int sentido = 0;
int velocidades[5] = {200, 400, 600, 800, 1000};

int stop = 0, tocando = 0, tocando1 = 0, tocando2 = 0, tocando3 = 0;

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

void configurar_salida(uint8_t pinNamePort,uint8_t pinNamePin,uint8_t func,uint8_t gpioPort,uint8_t gpioPin);

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

   /* print message (only on x86) */
   ciaaPOSIX_printf("Init Task...\n");

   fd_out = ciaaPOSIX_open("/dev/dio/out/0", ciaaPOSIX_O_RDWR);
   fd_in = ciaaPOSIX_open("/dev/dio/in/0", ciaaPOSIX_O_RDONLY);

   //GPIO8
   configurar_salida(6,12,FUNC0,2,8);
   //GPIO7
   configurar_salida(6,11,FUNC0,3,7);

   /* activate periodic task:
    *  - for the first time after 350 ticks (350 ms)
    *  - and then every 250 ticks (250 ms)
    */
   SetRelAlarm(ActivatePeriodicTask, 0, velocidades[speed]);
   SetRelAlarm(ActivatePeriodicTaskTeclas, 0, 150);

   /* terminate task */
   TerminateTask();
}

/** \brief Periodic Task
 *
 * This task is started automatically every time that the alarm
 * ActivatePeriodicTask expires.
 *
 */
TASK(PeriodicTask)
{
   //Leer el estado, y cambiarlo
   uint8_t gpioPort;
   uint8_t gpioPin;
   uint8_t pinValue;

   if(stop) {

   } else {
     if(sentido == 0) {
       if(value == 0) {
          gpioPort = 2;
          gpioPin = 8;
          Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpioPort, gpioPin, 1);
          value = 1;
        } else if (value == 1) {
          gpioPort = 3;
          gpioPin = 7;
          Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpioPort, gpioPin, 1);
          value = 2;
        } else if (value == 2) {
          gpioPort = 2;
          gpioPin = 8;
          Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpioPort, gpioPin, 0);
          value = 3;
        } else if (value == 3) {
          gpioPort = 3;
          gpioPin = 7;
          Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpioPort, gpioPin, 0);
          value = 0;
        }
      } else {
        if(value == 0) {
           gpioPort = 3;
           gpioPin = 7;
           Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpioPort, gpioPin, 1);
           value = 1;
         } else if (value == 1) {
           gpioPort = 2;
           gpioPin = 8;
           Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpioPort, gpioPin, 1);
           value = 2;
         } else if (value == 2) {
           gpioPort = 3;
           gpioPin = 7;
           Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpioPort, gpioPin, 0);
           value = 3;
         } else if (value == 3) {
           gpioPort = 2;
           gpioPin = 8;
           Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpioPort, gpioPin, 0);
           value = 0;
         }
      }
    }

/*
      count++;

      if(count == 16) {
        count = 0;

        if(sentido == 0) {
          sentido = 1;
        } else {
          sentido = 0;
        }
      }*/

/*
   if(sentido == 0) {
     Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpioPort, gpioPin, pinValue);
     sentido = 1;
   } else {
     Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpioPort, gpioPin, );
     sentido = 0;
   }

   if(++count == 10) {
     count = 0;

     speed = (speed + 1) % 5;

     CancelAlarm(ActivatePeriodicTask);
     SetRelAlarm(ActivatePeriodicTask, 0, velocidades[speed]);
   }
*/

   /* terminate task */
   TerminateTask();
}

TASK(PeriodicTaskTeclas)
{
  uint8_t inputs;

  ciaaPOSIX_read(fd_in, &inputs, 1);
  if(inputs == 0b1) { //Boton de Stop [TEC1]
    tocando = 1;    
  } else
  {
     if(tocando == 1)
     {
      if(stop == 1) stop = 0;
      else stop = 1;
      tocando = 0;
     } 
  } 
  if (inputs == 0b10) { //Boton del sentido [TEC2]
     tocando1 = 1;
  } else{
     if(tocando1 == 1){
       if(sentido == 1) sentido = 0;
       else sentido = 1;
       tocando1 = 0;
     }
  }     
  if (inputs == 0b100) { //Boton de disminucion de velocidad [TEC3]
     tocando2 = 1;
  } else{
      if(tocando2 == 1){
        if(speed == 4) {
        //Nada
        } else {
            speed++;
            CancelAlarm(ActivatePeriodicTask);
            SetRelAlarm(ActivatePeriodicTask, 0, velocidades[speed]);
          }
        tocando2 = 0;
      }
  }
  if (inputs == 0b1000) { //Boton de aumento de velocidad [TEC4]
    tocando3 = 1;
  } else{
      if(tocando3 == 1){
        if(speed == 0) {
        //Nada
        } else {
            speed--;
            CancelAlarm(ActivatePeriodicTask);
            SetRelAlarm(ActivatePeriodicTask, 0, velocidades[speed]);
          }
        tocando3 = 0;
       }
  }


  /* terminate task */
  TerminateTask();
}

void configurar_salida(uint8_t pinNamePort,uint8_t pinNamePin,uint8_t func,uint8_t gpioPort,uint8_t gpioPin) {
    //Configuración como OUTPUT:
     Chip_SCU_PinMux(
        pinNamePort,
        pinNamePin,
        SCU_MODE_INACT | SCU_MODE_ZIF_DIS,
        func
     );

     uint8_t OUTPUT = 1;

     Chip_GPIO_SetDir(LPC_GPIO_PORT, gpioPort, ( 1 << gpioPin ), OUTPUT);

     //Inicializo en 0
     uint8_t init = 0;

     Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpioPort, gpioPin, init);
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
