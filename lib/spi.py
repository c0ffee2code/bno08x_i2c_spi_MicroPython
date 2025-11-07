# BNO08X Micropython SPI Interface by BradCar
#
# Adapted from original Adafruit CircuitPython library
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# SPDX-License-Identifier: MIT
#
"""
Subclass of `BNO08X` to use SPI
"""
from struct import pack_into

from utime import ticks_ms, sleep_ms, sleep_us

from bno08x import BNO08X, Packet, PacketError, DATA_BUFFER_SIZE, _elapsed_sec


class BNO08X_SPI(BNO08X):
    """Library for the BNO08x IMUs on SPI

    Args:
        spi_bus: SPI bus object
        cs_pin: SPI CS pin to signal reads or writes
        reset_pin: optionl reset to BNO08x
        int_pin=None: optional int_pin to get signal when BNO08x is ready
        baudrate: (default 1 MHz, max 3 MHz)
        debug: Enables optional logging prints used for debugging driver
    """

    def __init__(self, spi_bus, cs_pin, reset_pin=None, int_pin=None, baudrate=1_000_000, debug=False):

        # BNO08X Data sheet (1.2.4.2 SPI Operation) requires CPOL = 1 and CPHA = 1.
        # CPOL is SPI polarity=1 and CPHA is SPI phase=1.
        self._spi = spi_bus
        self._spi.init(baudrate=baudrate, polarity=1, phase=1)
        
        self._reset = reset_pin
        self._int = int_pin
        
        self._cs = cs_pin  # ensure CS is de-asserted by default
        try:
            self._cs.value(1)
        except AttributeError:
            pass

        super().__init__(reset_pin=reset_pin, int_pin=int_pin, debug=debug)


    def hard_reset(self):

        print("\t\t *** Hard Reset start...")
        self._reset.value(1)
        sleep_ms(10)
        self._reset.value(0)
        sleep_ms(1)  # only 10 ns required
        self._reset.value(1)
        sleep_ms(100)  # 6.5.3 says need 90 ms + 4 ms
        self._wait_for_int()
        sleep_ms(50)
        self._read_packet()

    def _wait_for_int(self):
        start_time = ticks_ms()
        while _elapsed_sec(start_time) < 3.0:
            # BNO08x INT line is active low
            if self._int.value() == 0:
                print("self._int.value() went active low")
                return
            # small sleep to avoid busy-looping
            sleep_ms(1)

        # timeout: device did not assert INT
        raise RuntimeError("Timeout waiting for INT to go low")

    def soft_reset(self):
        """Reset the sensor to an initial unconfigured state"""
        for _i in range(3):
            try:
                _packet = self._read_packet()
            except PacketError:
                sleep_ms(100)


    def _read_into(self, buf, start=0, end=None):
        self._wait_for_int()

        if end is None:
            end = len(buf)
        if end <= start:
            return  # nothing to read or invalid range
        # with self._spi as spi:
        #     spi.readinto(buf, start=start, end=end, write_value=0x00)
        # print("SPI Read buffer (", end-start, "b )", [hex(i) for i in buf[start:end]])
        self._cs.value(0)
        sleep_us(1)
        self._spi.readinto(memoryview(buf)[start:end], 0x00)
        self._cs.value(1)
        #self._dbg(f"SPI read {end - start} bytes:", [hex(x) for x in buf[start:end]])

    def _read_header(self):
        """Reads the first 4 bytes available as a header"""
        self._wait_for_int()

        # read header
        # with self._spi as spi:
        #     spi.readinto(self._data_buffer, end=4, write_value=0x00)
        self._cs.value(0)
        sleep_us(1)
        self._spi.readinto(memoryview(self._data_buffer)[:4], 0x00)
        self._cs.value(1)
        self._dbg("")
        #self._dbg("SHTP READ packet header: ", [hex(x) for x in self._data_buffer[:4]])

    def _read_packet(self):
        self._read_header()
        halfpacket = False

        if self._data_buffer[1] & 0x80:
            halfpacket = True
        header = Packet.header_from_buffer(self._data_buffer)
        packet_byte_count = header.packet_byte_count
        channel_number = header.channel_number
        sequence_number = header.sequence_number

        self._sequence_number[channel_number] = sequence_number
        if packet_byte_count == 0:
            raise PacketError("No packet available")

        self._dbg("channel %d has %d bytes available" % (channel_number, packet_byte_count - 4))

        if packet_byte_count > DATA_BUFFER_SIZE:
            self._data_buffer = bytearray(packet_byte_count)

        # re-read header bytes since this is going to be a new transaction
        self._read_into(self._data_buffer, start=0, end=packet_byte_count)

        # print("Packet: ", [hex(i) for i in self._data_buffer[0:packet_byte_count]])

        if halfpacket:
            raise PacketError("read partial packet")

        new_packet = Packet(self._data_buffer)
        if self._debug:
            print(new_packet)
        self._update_sequence_number(new_packet)
        return new_packet

    def _read(self, requested_read_length):
        self._dbg("trying to read", requested_read_length, "bytes")
        unread_bytes = 0
        # +4 for the header
        total_read_length = requested_read_length + 4
        if total_read_length > DATA_BUFFER_SIZE:
            unread_bytes = total_read_length - DATA_BUFFER_SIZE
            total_read_length = DATA_BUFFER_SIZE

        # with self._spi as spi:
        #     spi.readinto(self._data_buffer, end=total_read_length)
        self._cs.value(0)
        sleep_us(1)
        self._spi.readinto(memoryview(self._data_buffer)[0:total_read_length], 0x00)
        self._cs.value(1)
        return unread_bytes > 0

    def _send_packet(self, channel, data):
        data_length = len(data)
        write_length = data_length + 4

        pack_into("<H", self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel
        self._data_buffer[3] = self._sequence_number[channel]
        for idx, send_byte in enumerate(data):
            self._data_buffer[4 + idx] = send_byte

        self._wait_for_int()
        # with self._spi as spi:
        #     spi.write(self._data_buffer, end=write_length)
        self._cs.value(0)
        self._spi.write(self._data_buffer[:write_length])
        self._cs.value(1)

        self._dbg("Sending: ", [hex(x) for x in self._data_buffer[0:write_length]])
        self._sequence_number[channel] = (self._sequence_number[channel] + 1) % 256
        return self._sequence_number[channel]

    @property
    def _data_ready(self):
        try:
            self._wait_for_int()
            return True
        except RuntimeError:
            return False
