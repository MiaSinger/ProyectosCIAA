#include "chip.h"             /* <= chip header */
#include "os.h"               /* <= operating system header */
#include "ciaaPOSIX_stdio.h"  /* <= device handler header */
#include "ciaaPOSIX_string.h" /* <= string header */
#include "ciaak.h"            /* <= ciaa kernel header */
#include "lectura_encoder.h"  /* <= own header */

/* Error de lectura del encoder */
#define ENC_ERROR 2

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
void send_uart(uint8_t data);

/* File descriptor para las salidas digitales (onboard leds) */
static int32_t fd_out;

/* File descriptor para la UART USB */
static int32_t fd_uart;

int pulsos = 0;
double vueltas;

int previo = 0, actual = 0, inc;

int main(void)
{
   /* Starts the operating system in the Application Mode 1 */
   /* This example has only one Application Mode */
   StartOS(AppMode1);

   /* StartOs shall never returns, but to avoid compiler warnings or errors
    * 0 is returned */
   return 0;
}

/*
 * Error Hook function
 */
void ErrorHook(void)
{
   ciaaPOSIX_printf("ErrorHook was called\n");
   ciaaPOSIX_printf("Service: %d, P1: %d, P2: %d, P3: %d, RET: %d\n", OSErrorGetServiceId(), OSErrorGetParam1(), OSErrorGetParam2(), OSErrorGetParam3(), OSErrorGetRet());
   ShutdownOS(0);
}

/*
 * Initial Task
 */
TASK(InitTask)
{
   /* init CIAA kernel and devices */
   ciaak_start();

   /* print message (only on x86) */
   ciaaPOSIX_printf("Init Task...\n");

   //GPIO8
   configurar_entrada(6,12,FUNC0,2,8);
   //GPIO7
   configurar_entrada(6,11,FUNC0,3,7);

   //Configuro salidas led
   fd_out = ciaaPOSIX_open("/dev/dio/out/0", ciaaPOSIX_O_RDWR);

   /* open UART connected to USB bridge (FT2232) */
   fd_uart = ciaaPOSIX_open("/dev/serial/uart/1", ciaaPOSIX_O_RDWR);

   /* change baud rate for uart usb */
   ciaaPOSIX_ioctl(fd_uart, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)ciaaBAUDRATE_115200);

   /* change FIFO TRIGGER LEVEL for uart usb */
   ciaaPOSIX_ioctl(fd_uart, ciaaPOSIX_IOCTL_SET_FIFO_TRIGGER_LEVEL, (void *)ciaaFIFO_TRIGGER_LEVEL3);

   /* activate periodic task */
   SetRelAlarm(ActivatePeriodicTask, 0, 100);

   /* terminate task */
   TerminateTask();
}

/*
 * Periodic Task
 */
TASK(PeriodicTask) {
   /*Lectura de encoder*/
   uint8_t entrada_1, entrada_2;
   leer_entradas_encoder(&entrada_1, &entrada_2);
   uint8_t encoder_in = obtener_valor_matriz(entrada_1, entrada_2);

   int incremento = encoder(encoder_in);
   pulsos += incremento;

   vueltas = pulsos / 32;

   uint8_t data = (uint8_t) pulsos;
   send_uart((uint8_t) incremento);

   /* terminate task */
   TerminateTask();
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
 void send_uart(uint8_t data) {
   /* buf = buffer para UART */
   int8_t buf[0];
   buf[0] = data;

   ciaaPOSIX_write(fd_uart, buf, 1);
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
  static uint8_t M_gray[2][2] = {{0,1},{2,3}};
  return M_gray[entrada_1][entrada_2];
}
