# SPDX-FileCopyrightText: Copyright (c) 2023 Scott Shawcroft for Adafruit Industries
#
# SPDX-License-Identifier: MIT

# PIO assembly is based on the examples in the RP2040 datasheet.
# SPDX-FileCopyrightText: Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
# SPDX-License-Identifier: BSD-3-Clause

"""
`adafruit_pio_uart`
================================================================================

PIO implementation of CircuitPython UART API


* Author(s): Scott Shawcroft
"""

import array
import time

import adafruit_pioasm
import busio
import rp2pio

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_PIO_UART.git"


class UART:
    """PIO implementation of CircuitPython UART API."""

    Parity = busio.UART.Parity

    def __init__(
        self, tx=None, rx=None, baudrate=9600, bits=8, parity=None, stop=1, timeout=1, cts=None, rts=None
    ):  # pylint: disable=invalid-name, too-many-arguments
        self.bitcount = bits + (1 if parity else 0)
        self.bits = bits
        self.parity = parity
        self.mask = (1 << bits) - 1
        self.shift = 8 - (self.bitcount % 8)
        self._timeout = timeout
        self.rx_pio = None
        if rx:
            if rts: 
                # Fleshed-out 8n1 UART receiver with hardware flow control handling
                # framing errors and break conditions more gracefully.
                # Wait for the start bit whilst updating rts with the FIFO level
                # then sample 8 bits with the correct timing.
                # IN pin 0 and JMP pin are both mapped to the GPIO used as UART RX.
                # OUT pin 0 is mapped to the GPIO used as UART RTS# (Request To Send).

                # Line by line explanation:
                # * Update rts pin with status of fifo level (high when full).
                # * Loop back to start whilst waiting for the start bit (low).
                # * Preload bit counter, then delay until eye of first data bit.
                # * Shift data bit into ISR
                # * Loop bitcount times, each loop iteration is 8 cycles.
                # * Jump to good_stop if there's a stop bit (high)
                # * otherwise wait for line to return to idle state.
                # * Don't push data if we didn't see good framing and start again.
                # * Push valid data and return to the start.
                rx_code = adafruit_pioasm.assemble(
                    ".program uart_rx_mini\n"
                    + "start:\n"
                    + "  mov pins !status\n"
                    + "  jmp pin start\n"
                    + f"  set x, {self.bitcount - 1} [10]\n"
                    + "bitloop:\n"
                    + "  in pins, 1\n"
                    + "  jmp x-- bitloop [6]\n"
                    + "  jmp pin good_stop\n"
                    + "  wait 1 pin 0\n"  # Skip IRQ
                    + "  jmp start\n"
                    + "good_stop:\n"
                    + "  push\n"
                )

                # mov_status_n is set to 7 allowing 2 further
                # entries (8 in the joined FIFO and one in ISR).
                self.rx_pio = rp2pio.StateMachine(
                    rx_code,
                    first_in_pin=rx,
                    jmp_pin=rx,
                    frequency=8 * baudrate,
                    auto_push=False,
                    push_threshold=self.bitcount,
                    first_out_pin=rts,
                    mov_status_type="rxfifo",
                    mov_status_n=7,
                )
            else:
                # Fleshed-out 8n1 UART receiver which handles
                # framing errors and break conditions more gracefully.
                # IN pin 0 is mapped to the GPIO used as UART RX.

                # Line by line explanation:
                # * Wait for start bit
                # * Preload bit counter, then delay until eye of first data bit.
                # * Shift data bit into ISR
                # * Loop bitcount times, each loop iteration is 8 cycles.
                # * Jump to good_stop if there's a stop bit (high)
                # * otherwise wait for line to return to idle state.
                # * Don't push data if we didn't see good framing and start again.
                # * Push valid data and return to the start.
                rx_code = adafruit_pioasm.assemble(
                    ".program uart_rx_mini\n"
                    + "start:\n"
                    + "  wait 0 pin 0\n"
                    + f"  set x, {self.bitcount - 1} [10]\n"
                    + "bitloop:\n"
                    + "  in pins, 1\n"
                    + "  jmp x-- bitloop [6]\n"
                    + "  jmp pin good_stop\n"
                    + "  wait 1 pin 0\n"  # Skip IRQ
                    + "  jmp start\n"
                    + "good_stop:\n"
                    + "  push\n"
                )
                self.rx_pio = rp2pio.StateMachine(
                    rx_code,
                    first_in_pin=rx,
                    jmp_pin=rx,
                    frequency=8 * baudrate,
                    auto_push=False,
                    push_threshold=self.bitcount,
                )

        self.tx_pio = None
        if tx:
            stop_delay = stop * 8 - 1
            if cts:
                # An 8n1 UART transmit program with hardware flow control
                # OUT pin 0 and side-set pin 0 are both mapped to UART TX pin.
                # IN pin 0 is mapped to GPIO pin used as CTS# (Clear To Send),

                # Line by line explanation:
                # * Assert stop bit, or stall with line in idle state
                # * Wait for CTS# before transmitting
                # * Preload bit counter, assert start bit for 8 clocks
                # * This loop will run 8 times (8n1 UART)
                # * Shift 1 bit from OSR to the first OUT pin
                # * Each loop iteration is 8 cycles.
                tx_code = adafruit_pioasm.Program(
                    ".program uart_tx\n"
                    + ".side_set 1 opt\n"
                    + f"  pull side 1 [{stop_delay}]\n"
                    + "wait 0 pin 0\n"
                    + f"  set x, {self.bitcount - 1} side 0 [7]\n"
                    + "bitloop:\n"
                    + "  out pins, 1\n"
                    + "  jmp x-- bitloop [6]\n"
                )
                self.tx_pio = rp2pio.StateMachine(
                    tx_code.assembled,
                    first_out_pin=tx,
                    first_sideset_pin=tx,
                    frequency=8 * baudrate,
                    initial_sideset_pin_state=1,
                    initial_sideset_pin_direction=1,
                    initial_out_pin_state=1,
                    initial_out_pin_direction=1,
                    first_in_pin=cts,
                    pull_in_pin_up=True,
                    wait_for_txstall=False,
                    **tx_code.pio_kwargs,
                )
            else:
                # An 8n1 UART transmit program.
                # OUT pin 0 and side-set pin 0 are both mapped to UART TX pin.

                # Line by line explanation:
                # * Assert stop bit, or stall with line in idle state
                # * Preload bit counter, assert start bit for 8 clocks
                # * This loop will run 8 times (8n1 UART)
                # * Shift 1 bit from OSR to the first OUT pin
                # * Each loop iteration is 8 cycles.
                tx_code = adafruit_pioasm.Program(
                    ".program uart_tx\n"
                    + ".side_set 1 opt\n"
                    + f"  pull side 1 [{stop_delay}]\n"
                    + f"  set x, {self.bitcount - 1} side 0 [7]\n"
                    + "bitloop:\n"
                    + "  out pins, 1\n"
                    + "  jmp x-- bitloop [6]\n"
                )
                self.tx_pio = rp2pio.StateMachine(
                    tx_code.assembled,
                    first_out_pin=tx,
                    first_sideset_pin=tx,
                    frequency=8 * baudrate,
                    initial_sideset_pin_state=1,
                    initial_sideset_pin_direction=1,
                    initial_out_pin_state=1,
                    initial_out_pin_direction=1,
                    **tx_code.pio_kwargs,
                )

    def deinit(self):
        """De-initialize the UART object."""
        if self.rx_pio:
            self.rx_pio.deinit()
        if self.tx_pio:
            self.tx_pio.deinit()

    @property
    def timeout(self):
        """Return the UART timeout."""
        return self._timeout

    @timeout.setter
    def timeout(self, value):
        """Set the UART timeout."""
        self._timeout = value

    @property
    def baudrate(self):
        """Return the UART baudrate."""
        if self.tx_pio:
            return self.tx_pio.frequency // 8
        return self.rx_pio.frequency // 8

    @baudrate.setter
    def baudrate(self, frequency):
        """Set the UART baudrate."""
        if self.rx_pio:
            self.rx_pio.frequency = frequency * 8
        if self.tx_pio:
            self.tx_pio.frequency = frequency * 8

    @property
    def in_waiting(self):
        """Return whether the UART is waiting."""
        return self.rx_pio.in_waiting

    def reset_input_buffer(self):
        """Clear the UART input buffer."""
        self.rx_pio.clear_rxfifo()

    def readinto(self, buf):
        """Read UART data into buf and return the number of bytes read."""
        if self.bitcount > 8:
            raw_in = array.array("H")
            for _ in range(len(buf)):
                raw_in.append(0)
        else:
            raw_in = buf
        mem_view = memoryview(raw_in)
        count = 0
        start_time = time.monotonic()
        while count < len(buf) and (
            self.timeout == 0 or (time.monotonic() - start_time) < self.timeout
        ):
            waiting = min(len(buf) - count, self.rx_pio.in_waiting)
            self.rx_pio.readinto(mem_view[count : count + waiting])
            if self.timeout == 0 and waiting == 0:
                return None if count == 0 else count
            count += waiting

        if self.parity is not None:
            for i in range(count):
                # TODO: Check parity bits instead of just masking them.
                buf[i] = (raw_in[i] >> self.shift) & self.mask
        return count

    def read(self, n):
        """Read and return an array of up to n values from the UART."""
        if self.bits > 8:
            buf = array.array(n)
        else:
            buf = bytearray(n)
        n = self.readinto(buf)
        if n == 0 or n is None:
            return None
        if n < len(buf):
            return buf[:n]
        return buf

    def write(self, buf):
        """Write the contents of buf to the UART."""
        # Compute parity if we need to
        if self.parity:
            if self.bitcount > 8:
                a = array.array("H")  # pylint: disable=invalid-name
            for i, v in enumerate(buf):  # pylint: disable=invalid-name
                a.append(v)
                ones = 0
                for pos in range(self.bitcount - 1):
                    if (v & (1 << pos)) != 0:
                        ones += 1
                parity = 0
                if self.parity == self.Parity.ODD:
                    if (ones % 2) == 0:
                        parity = 1
                elif (ones % 2) == 1:
                    # even parity needs another one if the data is odd
                    parity = 1
                a[i] |= parity << (self.bitcount - 1)
            buf = a
        return self.tx_pio.write(buf)
