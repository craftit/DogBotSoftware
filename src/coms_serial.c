#include "coms_serial.h"

#include "ch.h"
#include "hal.h"

static virtual_timer_t vt3, vt4, vt5;

static const uint8_t message[] = "0123456789ABCDEF";
static uint8_t buffer[16];

static void led3off(void *p) {

  (void)p;
  //palClearPad(GPIOD, GPIOD_LED3);
}

static void led4off(void *p) {

  (void)p;
  palClearPad(GPIOC, GPIOC_PIN5);
}

static void led5off(void *p) {

  (void)p;
  //palClearPad(GPIOD, GPIOD_LED5);
}

/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {

  (void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {

  (void)uartp;
  //palSetPad(GPIOD, GPIOD_LED5);
  chSysLockFromISR();
  chVTResetI(&vt5);
  chVTSetI(&vt5, MS2ST(200), led5off, NULL);
  chSysUnlockFromISR();
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {

  (void)uartp;
  (void)e;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {

  (void)uartp;
  (void)c;
  /* Flashing the LED each time a character is received.*/
  palSetPad(GPIOC, GPIOC_PIN5);       /* Orange.  */
  chSysLockFromISR();
  chVTResetI(&vt4);
  chVTSetI(&vt4, MS2ST(200), led4off, NULL);
  chSysUnlockFromISR();
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {

  (void)uartp;

  /* Flashing the LED each time a character is received.*/
  //palSetPad(GPIOD, GPIOD_LED3);
  chSysLockFromISR();
  chVTResetI(&vt3);
  chVTSetI(&vt3, MS2ST(200), led3off, NULL);
  chSysUnlockFromISR();
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg_1 = {
  txend1,
  txend2,
  rxend,
  rxchar,
  rxerr,
  38400,
  0,
  USART_CR2_LINEN,
  0
};

void InitSerial()
{
  /*
   * Activates the UART driver 1
   */
  uartStart(&UARTD1, &uart_cfg_1);


}

void SendSerialTest(void)
{
  static const uint8_t message[] = "0123456789ABCDEF";
  static uint8_t buffer[16];

  /*
   * Starts both a transmission and a receive operations, both will be
   * handled entirely in background.
   */
  uartStopReceive(&UARTD1);
  uartStopSend(&UARTD1);
  uartStartReceive(&UARTD1, 16, buffer);
  uartStartSend(&UARTD1, 16, message);
}

