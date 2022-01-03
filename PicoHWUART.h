/*
    Serial-over-UART for the Raspberry Pi Pico RP2040

    Copyright (c) 2021 Earle F. Philhower, III <earlephilhower@yahoo.com>
    Modified 2022 Eric Brundick <spirilis at linux dot com>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once

#include <Arduino.h>
#include <api/HardwareSerial.h>
#include <stdarg.h>
#include <CoreMutex.h>

extern "C" typedef struct uart_inst uart_inst_t;

// Setting to 32 because the RP2040 UART FIFOs are 32 bytes deep
#define PICOHWUART_DEFAULT_RX_BUF_SIZE 32
#define PICOHWUART_DEFAULT_TX_BUF_SIZE 8

class PicoHWUART : public HardwareSerial {
public:
    PicoHWUART(uart_inst_t *uart, pin_size_t tx, pin_size_t rx);

    // Select the pinout.  Call before .begin()
    bool setRX(pin_size_t pin);
    bool setTX(pin_size_t pin);
    bool setPinout(pin_size_t tx, pin_size_t rx) {
        bool ret = setRX(rx);
        ret &= setTX(tx);
        return ret;
    }

    void begin(unsigned long baud = 115200) override {
        begin(baud, SERIAL_8N1);
    };
    void begin(unsigned long baud, uint16_t config) override;
    void end() override;

    virtual int peek() override;
    virtual int read() override;
    virtual int available() override;
    virtual int availableForWrite() override;
    virtual void flush() override;
    virtual size_t write(uint8_t c) override;
    virtual size_t write(const uint8_t *p, size_t len) override;
    using Print::write;
    operator bool() override;

    // Public function allows user to provide their own buffers
    void setBuffers(char *txbuf, unsigned int txbufsz, char *rxbuf, unsigned int rxbufsz);

    // Need to be public in order for the static IRQ handler functions to use
    uart_inst_t *_uart;
    void _rxBufWrite(char);
    int _txBufRead();
    void _triggerTX();

private:
    bool _running = false;
    pin_size_t _tx, _rx;
    int _baud;
    int _peek;
    mutex_t _mutex;
    void _uart_irq_config(bool enabled, int rx_timeout_setting, int tx_timeout_setting);
    void _bufReset();
    int _rxBufRead();
    int _rxBufPeek(int);
    bool _txAvailableForWrite();
    void _txBufWrite(char);

    volatile char *_rxBuf;
    char _defaultRxBuf[PICOHWUART_DEFAULT_RX_BUF_SIZE];
    volatile char *_rxHead, *_rxTail;
    volatile char *_txBuf;
    char _defaultTxBuf[PICOHWUART_DEFAULT_TX_BUF_SIZE];
    volatile char *_txHead, *_txTail;
    int _rxBufSz, _txBufSz;
};

extern PicoHWUART PicoSerial1; // HW UART 0
extern PicoHWUART PicoSerial2; // HW UART 1

namespace arduino {
extern void serialEvent1Run(void) __attribute__((weak));
extern void serialEvent2Run(void) __attribute__((weak));
};
