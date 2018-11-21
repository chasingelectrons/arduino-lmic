/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * Copyright (c) 2018 MCCI Corporation
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the HAL to run LMIC on top of the Arduino environment.
 *******************************************************************************/

#include <Arduino.h>
#include <SPI.h>
// include all the lmic header files, including ../lmic/hal.h
#include "../lmic.h"
// include the C++ hal.h
#include "hal.h"
// we may need some things from stdio.
#include <stdio.h>

// -----------------------------------------------------------------------------
// I/O

static const Arduino_LMIC::HalPinmap_t *plmic_pins;
static Arduino_LMIC::HalConfiguration_t *pHalConfig;
static Arduino_LMIC::HalConfiguration_t nullHalConig;

static void hal_interrupt_init(); // Fwd declaration

static void hal_io_init () {
    // NSS and DIO0 are required, DIO1 is required for LoRa, DIO2 for FSK
    ASSERT(plmic_pins->nss != LMIC_UNUSED_PIN);
    ASSERT(plmic_pins->dio[0] != LMIC_UNUSED_PIN);
    ASSERT(plmic_pins->dio[1] != LMIC_UNUSED_PIN || plmic_pins->dio[2] != LMIC_UNUSED_PIN);

//    Serial.print("nss: "); Serial.println(plmic_pins->nss);
//    Serial.print("rst: "); Serial.println(plmic_pins->rst);
//    Serial.print("dio[0]: "); Serial.println(plmic_pins->dio[0]);
//    Serial.print("dio[1]: "); Serial.println(plmic_pins->dio[1]);
//    Serial.print("dio[2]: "); Serial.println(plmic_pins->dio[2]);

    // initialize SPI chip select to high (it's active low)
    digitalWrite(plmic_pins->nss, HIGH);
    pinMode(plmic_pins->nss, OUTPUT);

    if (plmic_pins->rxtx != LMIC_UNUSED_PIN) {
        // initialize to RX
        digitalWrite(plmic_pins->rxtx, LOW != plmic_pins->rxtx_rx_active);
        pinMode(plmic_pins->rxtx, OUTPUT);
    }
    if (plmic_pins->rst != LMIC_UNUSED_PIN) {
        // initialize RST to floating
        pinMode(plmic_pins->rst, INPUT);
    }

    hal_interrupt_init();
}

// val == 1  => tx
void hal_pin_rxtx (u1_t val) {
    if (plmic_pins->rxtx != LMIC_UNUSED_PIN)
        digitalWrite(plmic_pins->rxtx, val != plmic_pins->rxtx_rx_active);
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
    if (plmic_pins->rst == LMIC_UNUSED_PIN)
        return;

    if(val == 0 || val == 1) { // drive pin
        digitalWrite(plmic_pins->rst, val);
        pinMode(plmic_pins->rst, OUTPUT);
    } else { // keep pin floating
        pinMode(plmic_pins->rst, INPUT);
    }
}

s1_t hal_getRssiCal (void) {
    return plmic_pins->rssi_cal;
}

#if !defined(LMIC_USE_INTERRUPTS)
static void hal_interrupt_init() {
    pinMode(plmic_pins->dio[0], INPUT);
    if (plmic_pins->dio[1] != LMIC_UNUSED_PIN)
        pinMode(plmic_pins->dio[1], INPUT);
    if (plmic_pins->dio[2] != LMIC_UNUSED_PIN)
        pinMode(plmic_pins->dio[2], INPUT);
}

static bool dio_states[NUM_DIO] = {0};
static void hal_io_check() {
    uint8_t i;
    for (i = 0; i < NUM_DIO; ++i) {
        if (plmic_pins->dio[i] == LMIC_UNUSED_PIN)
            continue;

        if (dio_states[i] != digitalRead(plmic_pins->dio[i])) {
            dio_states[i] = !dio_states[i];
            if (dio_states[i])
                radio_irq_handler(i);
        }
    }
}

#else
// Interrupt handlers
static ostime_t interrupt_time[NUM_DIO] = {0};

static void hal_isrPin0() {
    ostime_t now = os_getTime();
    interrupt_time[0] = now ? now : 1;
}
static void hal_isrPin1() {
    ostime_t now = os_getTime();
    interrupt_time[1] = now ? now : 1;
}
static void hal_isrPin2() {
    ostime_t now = os_getTime();
    interrupt_time[2] = now ? now : 1;
}

typedef void (*isr_t)();
static isr_t interrupt_fns[NUM_DIO] = {hal_isrPin0, hal_isrPin1, hal_isrPin2};

static void hal_interrupt_init() {
  for (uint8_t i = 0; i < NUM_DIO; ++i) {
      if (plmic_pins->dio[i] == LMIC_UNUSED_PIN)
          continue;

      attachInterrupt(digitalPinToInterrupt(plmic_pins->dio[i]), interrupt_fns[i], RISING);
  }
}

static void hal_io_check() {
    uint8_t i;
    for (i = 0; i < NUM_DIO; ++i) {
        ostime_t iTime;
        if (plmic_pins->dio[i] == LMIC_UNUSED_PIN)
            continue;

        iTime = interrupt_time[i];
        if (iTime) {
            interrupt_time[i] = 0;
            radio_irq_handler_v2(i, iTime);
        }
    }
}
#endif // LMIC_USE_INTERRUPTS

// -----------------------------------------------------------------------------
// SPI

static void hal_spi_init () {
    SPI.begin();
}

void hal_pin_nss (u1_t val) {
    if (!val) {
        uint32_t spi_freq;

        if ((spi_freq = plmic_pins->spi_freq) == 0)
            spi_freq = LMIC_SPI_FREQ;

        SPISettings settings(spi_freq, MSBFIRST, SPI_MODE0);
        SPI.beginTransaction(settings);
    } else {
        SPI.endTransaction();
    }

    //Serial.println(val?">>":"<<");
    digitalWrite(plmic_pins->nss, val);
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
    u1_t res = SPI.transfer(out);
/*
    Serial.print(">");
    Serial.print(out, HEX);
    Serial.print("<");
    Serial.println(res, HEX);
    */
    return res;
}

// -----------------------------------------------------------------------------
// TIME
// This code is intended for ATmega328p only
#include <avr/interrupt.h>
#include <avr/sleep.h>

static void hal_time_init () {
    // safe procedure of switching the clock source
    // when Timer2 operates asynchronously, according to datasheet
    // 22.9. Asynchronous Operation of Timer/Counter2
    // see also https://www.avrfreaks.net/forum/overflow-interrupt-timer2-asynchronous-mode
    TIMSK2 = 0;           // Disable timer2 interrupts              (1)
    ASSR   = _BV(AS2);    // Switch to asynchronous mode            (2)
    TCNT2  = 0;           // Clear timer-counter                    (3)
    TCCR2A = 0;           // Reset control registers: normal mode,  (3)
    TCCR2B = _BV(CS20);   //   no prescaling (32.768/256 ovf/s)     (3)
    while (ASSR & (_BV(TCN2UB) | _BV(TCR2AUB) | _BV(TCR2BUB)));  // (4)
    TIFR2  = 0;           // Clear interrupt flags                  (5)
    TIMSK2 = _BV(TOIE2);  // Enable timer2 overflow interrupt       (6)
}

static volatile u4_t RTCoverflow=0;
static u4_t HAL_timer=0;

ISR(TIMER2_OVF_vect)
{
    // Zu Dummywrite und Whileschleife siehe Mikrocontroller.net
    // sie dienen dazu dass ein Overflow nicht mehrmahls gezählt wird.
    // https://www.mikrocontroller.net/articles/AVR-GCC-Tutorial/Die_Timer_und_Zähler_des_AVR#Timer2_im_Asynchron_Mode
    TCCR2B = TCCR2B;              //Dummy Write
    RTCoverflow++;
    RTCoverflow&=0x00FFFFFF;      // Make sure it overflows at 3-byte boundary
    while(ASSR & ((1<<TCN2UB) | (1<<OCR2AUB) | (1<<OCR2BUB) |
                  (1<<TCR2AUB) | (1<<TCR2BUB)));
}

u4_t hal_ticks() {
    return (RTCoverflow<<8)+TCNT2;
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
static s4_t delta_time(u4_t time) {
    return (s4_t)(time - hal_ticks());
}

void hal_waitUntil (u4_t time) {
    s4_t delta = delta_time(time);
    if (delta > ms2osticks(16)) {
        HAL_timer=time;
        hal_sleep();
    } else {
        if (delta > 0) {
            delayMicroseconds(osticks2us(delta));
        }
    }
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    HAL_timer=time;
    return delta_time(time) <= 0;
}

static uint8_t irqlevel = 0;

void hal_disableIRQs () {
    noInterrupts();
    irqlevel++;
}

void hal_enableIRQs () {
    if(--irqlevel == 0) {
        interrupts();

        // Instead of using proper interrupts (which are a bit tricky
        // and/or not available on all pins on AVR), just poll the pin
        // values. Since os_runloop disables and re-enables interrupts,
        // putting this here makes sure we check at least once every
        // loop.
        //
        // As an additional bonus, this prevents the can of worms that
        // we would otherwise get for running SPI transfers inside ISRs
        hal_io_check();
    }
}

void hal_sleep () {
  if(delta_time(HAL_timer) <= 0){
    return; // TODO: Find out who is calling hal_sleep without setting the timer!
  }
  Serial.flush();
  while((HAL_timer/256)>RTCoverflow){ // only sleep if TIMER2OVF will occur
    // actually sleep here
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    cli();
    sleep_enable();
    sleep_bod_disable(); // disabling BOD during sleep: 26uA less
    sei();
    sleep_cpu();
    // sleeping here until we wake up
    sleep_disable();
  }
  while(((HAL_timer/256)==RTCoverflow) && ((HAL_timer%256)>TCNT2)){
    delayMicroseconds(osticks2us(1));
  }
}

// -----------------------------------------------------------------------------

#if defined(LMIC_PRINTF_TO)
#if !defined(__AVR)
static ssize_t uart_putchar (void *, const char *buf, size_t len) {
    return LMIC_PRINTF_TO.write((const uint8_t *)buf, len);
}

static cookie_io_functions_t functions =
 {
     .read = NULL,
     .write = uart_putchar,
     .seek = NULL,
     .close = NULL
 };

void hal_printf_init() {
    stdout = fopencookie(NULL, "w", functions);
    if (stdout != nullptr) {
        setvbuf(stdout, NULL, _IONBF, 0);
    }
}
#else // defined(__AVR)
static int uart_putchar (char c, FILE *)
{
    LMIC_PRINTF_TO.write(c) ;
    return 0 ;
}

void hal_printf_init() {
    // create a FILE structure to reference our UART output function
    static FILE uartout;
    memset(&uartout, 0, sizeof(uartout));

    // fill in the UART file descriptor with pointer to writer.
    fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);

    // The uart is the standard output device STDOUT.
    stdout = &uartout ;
}

#endif // !defined(ESP8266) || defined(ESP31B) || defined(ESP32)
#endif // defined(LMIC_PRINTF_TO)

void hal_init (void) {
    // use the global constant
    Arduino_LMIC::hal_init_with_pinmap(&lmic_pins);
}

// hal_init_ex is a C API routine, written in C++, and it's called
// with a pointer to an lmic_pinmap.
void hal_init_ex (const void *pContext) {
    const lmic_pinmap * const pHalPinmap = (const lmic_pinmap *) pContext;
    if (! Arduino_LMIC::hal_init_with_pinmap(pHalPinmap)) {
        hal_failed(__FILE__, __LINE__);
    }
}

// C++ API: initialize the HAL properly with a configuration object
namespace Arduino_LMIC {
bool hal_init_with_pinmap(const HalPinmap_t *pPinmap)
    {
    if (pPinmap == nullptr)
        return false;

    // set the static pinmap pointer.
    plmic_pins = pPinmap;

    // set the static HalConfiguration pointer.
    HalConfiguration_t * const pThisHalConfig = pPinmap->pConfig;

    if (pThisHalConfig != nullptr)
        pHalConfig = pThisHalConfig;
    else
        pHalConfig = &nullHalConig;

    pHalConfig->begin();

    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();
#if defined(LMIC_PRINTF_TO)
    // printf support
    hal_printf_init();
#endif
    // declare success
    return true;
    }
}; // namespace Arduino_LMIC

void hal_failed (const char *file, u2_t line) {
#if defined(LMIC_FAILURE_TO)
    LMIC_FAILURE_TO.println("FAILURE ");
    LMIC_FAILURE_TO.print(file);
    LMIC_FAILURE_TO.print(':');
    LMIC_FAILURE_TO.println(line);
    LMIC_FAILURE_TO.flush();
#endif
    hal_disableIRQs();
    while(1);
}

ostime_t hal_setTcxoPower (u1_t val) {
    return pHalConfig->setTcxoPower(val);
}

bit_t hal_queryUsingTcxo(void) {
    return pHalConfig->queryUsingTcxo();
}