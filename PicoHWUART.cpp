/*
    Serial-over-UART for the Raspberry Pi Pico RP2040

    Copyright (c) 2021 Earle F. Philhower, III <earlephilhower@yahoo.com>

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

#include "PicoHWUART.h"
#include <CoreMutex.h>
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/regs/intctrl.h>
#include <hardware/address_mapped.h>

// SerialEvent functions are weak, so when the user doesn't define them,
// the linker just sets their address to 0 (which is checked below).
// The Serialx_available is just a wrapper around Serialx.available(),
// but we can refer to it weakly so we don't pull in the entire
// HardwareSerial instance if the user doesn't also refer to it.
extern void serialEvent1() __attribute__((weak));
extern void serialEvent2() __attribute__((weak));

PicoHWUART *_uart0_obj;
void _uart0_IRQ_handler(void) {
    uint32_t mis = uart_get_hw(_uart0_obj->_uart)->mis;
    int c;

    if (mis & (UART_UARTMIS_RXMIS_BITS | UART_UARTMIS_RTMIS_BITS)) {
        // Receive data & stuff into rxBuf
        while (uart_is_readable(_uart0_obj->_uart)) {
            c = (int)uart_getc(_uart0_obj->_uart);
            _uart0_obj->_rxBufWrite((char)c);
        }
    }
    if (mis & (UART_UARTMIS_TXMIS_BITS)) {
        // Transmit more data from txBuf
        while (uart_is_writable(_uart0_obj->_uart) && (c = _uart0_obj->_txBufRead()) >= 0) {
            uart_putc(_uart0_obj->_uart, (char)c);
        }
        if (c < 0) {
            // Disable TX IRQ for now if we have nothing left to transmit
            uart_get_hw(_uart0_obj->_uart)->imsc = (UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS);
        }
    }
}

PicoHWUART *_uart1_obj;
void _uart1_IRQ_handler(void) {
    uint32_t mis = uart_get_hw(_uart1_obj->_uart)->mis;
    int c;

    if (mis & (UART_UARTMIS_RXMIS_BITS | UART_UARTMIS_RTMIS_BITS)) {
        // Receive data & stuff into rxBuf
        while (uart_is_readable(_uart1_obj->_uart)) {
            c = (int)uart_getc(_uart1_obj->_uart);
            _uart1_obj->_rxBufWrite((char)c);
        }
    }
    if (mis & (UART_UARTMIS_TXMIS_BITS)) {
        // Transmit more data from txBuf
        while (uart_is_writable(_uart1_obj->_uart) && (c = _uart1_obj->_txBufRead()) >= 0) {
            uart_putc(_uart1_obj->_uart, (char)c);
        }
        if (c < 0) {
            // Disable TX IRQ for now if we have nothing left to transmit
            uart_get_hw(_uart1_obj->_uart)->imsc = (UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS);
        }
    }
}


bool PicoHWUART::setRX(pin_size_t pin) {
    constexpr uint32_t valid[2] = { __bitset({1, 13, 17, 29}) /* UART0 */,
                                    __bitset({5, 9, 21, 25})  /* UART1 */
                                  };
    if ((!_running) && ((1 << pin) & valid[uart_get_index(_uart)])) {
        _rx = pin;
        return true;
    }

    if (_running) {
        panic("FATAL: Attempting to set Serial%d.RX while running", uart_get_index(_uart) + 1);
    } else {
        panic("FATAL: Attempting to set Serial%d.RX to illegal pin %d", uart_get_index(_uart) + 1, pin);
    }
    return false;
}

bool PicoHWUART::setTX(pin_size_t pin) {
    constexpr uint32_t valid[2] = { __bitset({0, 12, 16, 28}) /* UART0 */,
                                    __bitset({4, 8, 20, 24})  /* UART1 */
                                  };
    if ((!_running) && ((1 << pin) & valid[uart_get_index(_uart)])) {
        _tx = pin;
        return true;
    }

    if (_running) {
        panic("FATAL: Attempting to set Serial%d.TX while running", uart_get_index(_uart) + 1);
    } else {
        panic("FATAL: Attempting to set Serial%d.TX to illegal pin %d", uart_get_index(_uart) + 1, pin);
    }
    return false;
}

PicoHWUART::PicoHWUART(uart_inst_t *uart, pin_size_t tx, pin_size_t rx) {
    _uart = uart;
    _tx = tx;
    _rx = rx;
    _txBuf = _defaultTxBuf;
    _rxBuf = _defaultRxBuf;
    _txBufSz = PICOHWUART_DEFAULT_TX_BUF_SIZE;
    _rxBufSz = PICOHWUART_DEFAULT_RX_BUF_SIZE;
    mutex_init(&_mutex);
}

void PicoHWUART::_uart_irq_config(bool enabled, int rx_timeout_setting, int tx_timeout_setting) {
    if (!enabled) {
        uart_get_hw(_uart)->imsc = ~(UART_UARTIMSC_TXIM_BITS | UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS);
    } else {
        uart_get_hw(_uart)->imsc = (UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS);
        hw_write_masked(&uart_get_hw(_uart)->ifls, rx_timeout_setting << UART_UARTIFLS_RXIFLSEL_LSB, UART_UARTIFLS_RXIFLSEL_BITS);
        hw_write_masked(&uart_get_hw(_uart)->ifls, tx_timeout_setting << UART_UARTIFLS_TXIFLSEL_LSB, UART_UARTIFLS_TXIFLSEL_BITS);
    }
}

void PicoHWUART::_bufReset() {
    _rxHead = _rxBuf;
    _rxTail = _rxBuf;
    _txHead = _txBuf;
    _txTail = _txBuf;
}

int PicoHWUART::_rxBufRead() {
    char c;

    if (_rxTail == _rxHead) {
        return -1;  // EOF
    }
    c = *_rxTail++;
    if ( (_rxTail - _rxBuf) >= (_rxBufSz) ) {
        _rxTail = _rxBuf;
    }
    return (int)c;
}

int PicoHWUART::_rxBufPeek(int idx) {
    if (_rxTail == _rxHead) {
        return -1;
    }
    int i;

    if (_rxTail < _rxHead) {
        if ( (_rxTail + idx) < _rxHead ) {
            return *(_rxTail+idx);
        }
        return -1;
    }
    i = _rxBufSz - (_rxTail - _rxBuf);
    if (idx < i) {
        return *(_rxTail + idx);
    }
    idx -= i;
    if ( (_rxBuf + idx) < _rxHead ) {
        return *(_rxBuf+idx);
    }
    return -1;
}

// Intended to be used by the IRQ handler
void PicoHWUART::_rxBufWrite(char c) {
    *_rxHead++ = c;
    if ( (_rxHead - _rxBuf) >= (_rxBufSz) ) {
        _rxHead = _rxBuf;
    }
    if (_rxHead == _rxTail) {  // Overflow; advance rxTail by 1 to avoid losing all the buffer's data
        _rxTail++;
        if ( (_rxTail - _rxBuf) >= (_rxBufSz) ) {
            _rxTail = _rxBuf;
        }
    }
}

void PicoHWUART::_txBufWrite(char c) {
    *_txHead++ = c;
    if ( (_txHead - _txBuf) >= (_txBufSz) ) {  // Wraparound
        _txHead = _txBuf;
    }
    if (_txHead == _txTail) {  // Overflow; advance txTail by 1 to avoid losing all the buffer's data
        _txTail++;
        if ( (_txTail - _txBuf) >= (_txBufSz) ) {
            _txTail = _txBuf;
        }
    }
}

void PicoHWUART::_triggerTX() {
    // Trigger TX

    // Must kick-start the TX process if it's idle
    int c;
    while (uart_is_writable(_uart) && (c = _txBufRead()) >= 0) {
        uart_putc(_uart, (char)c);
    }

    // Enable TX interrupt
    uart_get_hw(_uart)->imsc = (UART_UARTIMSC_TXIM_BITS | UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS);
}

// Intended to be used by the IRQ handler
int PicoHWUART::_txBufRead() {
    char c;

    if (_txTail == _txHead) {
        return -1;  // EOF
    }
    c = *_txTail++;
    if ( (_txTail - _txBuf) >= (_txBufSz) ) {
        _txTail = _txBuf;
    }
    return (int)c;
}

bool PicoHWUART::_txAvailableForWrite() {
    if (_txHead == _txTail) {
        return 1;
    }
    if (_txHead == _txBuf) {
        if (_txTail < (_txBuf + _txBufSz - 1)) {
            return 1;
        } else {
            return 0;
        }
    }
    if (_txTail < (_txHead - 1) || (_txTail > _txHead) ) {
        return 1;
    }
    return 0;
}

void PicoHWUART::setBuffers(char *txbuf, unsigned int txbufsz, char *rxbuf, unsigned int rxbufsz) {
    if (txbuf != NULL && rxbuf != NULL &&
        txbufsz > 0 && rxbufsz > 0) {
        _txBuf = txbuf;
        _txBufSz = txbufsz;
        _rxBuf = rxbuf;
        _rxBufSz = rxbufsz;
        _bufReset();
    }
}

void PicoHWUART::begin(unsigned long baud, uint16_t config) {
    _baud = baud;
    uart_init(_uart, baud);
    int bits, stop;
    uart_parity_t parity;
    switch (config & SERIAL_PARITY_MASK) {
    case SERIAL_PARITY_EVEN: parity = UART_PARITY_EVEN; break;
    case SERIAL_PARITY_ODD: parity = UART_PARITY_ODD; break;
    default: parity = UART_PARITY_NONE; break;
    }
    switch (config & SERIAL_STOP_BIT_MASK) {
    case SERIAL_STOP_BIT_1: stop = 1; break;
    default: stop = 2; break;
    }
    switch (config & SERIAL_DATA_MASK) {
    case SERIAL_DATA_5: bits = 5; break;
    case SERIAL_DATA_6: bits = 6; break;
    case SERIAL_DATA_7: bits = 7; break;
    default: bits = 8; break;
    }
    uart_set_format(_uart, bits, stop, parity);
    gpio_set_function(_tx, GPIO_FUNC_UART);
    gpio_set_function(_rx, GPIO_FUNC_UART);
    _bufReset();

    switch (uart_get_index(_uart)) {
    case 0:
        _uart0_obj = this;
        irq_clear(UART0_IRQ);
        irq_set_exclusive_handler(UART0_IRQ, _uart0_IRQ_handler);
        irq_set_enabled(UART0_IRQ, true);
        break;
    case 1:
        _uart1_obj = this;
        _bufReset();
        irq_clear(UART1_IRQ);
        irq_set_exclusive_handler(UART1_IRQ, _uart1_IRQ_handler);
        irq_set_enabled(UART1_IRQ, true);
        break;
    }
    uart_set_fifo_enabled(_uart, true);
    _uart_irq_config(true, 1, 1);
    _running = true;
    _peek = -1;
}

void PicoHWUART::end() {
    if (!_running) {
        return;
    }
    _uart_irq_config(false, 0, 0);
    switch (uart_get_index(_uart)) {
    case 0:
        irq_set_enabled(UART0_IRQ, false);
        irq_remove_handler(UART0_IRQ, _uart0_IRQ_handler);
        break;
    case 1:
        irq_set_enabled(UART1_IRQ, false);
        irq_remove_handler(UART1_IRQ, _uart1_IRQ_handler);
        break;
    }
    uart_deinit(_uart);
    _running = false;
}

int PicoHWUART::peek() {
    CoreMutex m(&_mutex);
    if (!_running || !m) {
        return -1;
    }
    return _rxBufPeek(1);
    /*if (_peek >= 0) {
        return _peek;
    }
    if (uart_is_readable_within_us(_uart, _timeout * 1000)) {
        _peek = uart_getc(_uart);
    } else {
        _peek = -1; // Timeout
    }*/

    return _peek;
}

int PicoHWUART::read() {
    CoreMutex m(&_mutex);
    if (!_running || !m) {
        return -1;
    }

    return _rxBufRead();
/*    if (_peek >= 0) {
        int ret = _peek;
        _peek = -1;
        return ret;
    }
    if (uart_is_readable_within_us(_uart, _timeout * 1000)) {
        return uart_getc(_uart);
    } else {
        return -1; // Timeout
    } */
}

int PicoHWUART::available() {
    CoreMutex m(&_mutex);
    if (!_running || !m) {
        return 0;
    }
    return (_rxTail != _rxHead) ? 1 : 0;
    //return (uart_is_readable(_uart)) ? 1 : 0;
}

int PicoHWUART::availableForWrite() {
    CoreMutex m(&_mutex);
    if (!_running || !m) {
        return 0;
    }

    if (_txAvailableForWrite()) {
        return 1;
    }
    return 0;
    //return (uart_is_writable(_uart)) ? 1 : 0;
}

void PicoHWUART::flush() {
    CoreMutex m(&_mutex);
    if (!_running || !m) {
        return;
    }
    /*
    // Double-check that we don't have stuck data in the TX buffer
    if ( (_txTail != _txHead) && !(uart_get_hw(_uart0_obj->_uart)->imsc & (UART_UARTIMSC_TXIM_BITS)) ) {
        // Something wrong here - we have TX data in the buffer but the TX IRQ isn't masked
    }
    */
    // Poll waiting for _txTail == _txHead
    while (_txTail != _txHead) {
        delayMicroseconds(100);
    }
    //uart_tx_wait_blocking(_uart);
}

size_t PicoHWUART::write(uint8_t c) {
    CoreMutex m(&_mutex);
    if (!_running || !m) {
        return 0;
    }
    while (!_txAvailableForWrite()) {
        delayMicroseconds(50);
    }
    _txBufWrite((char)c);
    _triggerTX();
    //uart_putc_raw(_uart, c);
    return 1;
}

size_t PicoHWUART::write(const uint8_t *p, size_t len) {
    CoreMutex m(&_mutex);
    if (!_running || !m) {
        return 0;
    }
    size_t cnt = len;
    while (cnt) {
        //uart_putc_raw(_uart, *p);
        while (cnt && _txAvailableForWrite()) {
            _txBufWrite((char)*p);
            cnt--;
            p++;
        }
        _triggerTX();
    }
    return len;
}

PicoHWUART::operator bool() {
    return _running;
}

PicoHWUART PicoSerial1(uart0, PIN_SERIAL1_TX, PIN_SERIAL1_RX);
PicoHWUART PicoSerial2(uart1, PIN_SERIAL2_TX, PIN_SERIAL2_RX);

void arduino::serialEvent1Run(void) {
    if (serialEvent1 && PicoSerial1.available()) {
        serialEvent1();
    }
}

void arduino::serialEvent2Run(void) {
    if (serialEvent2 && PicoSerial2.available()) {
        serialEvent2();
    }
}
