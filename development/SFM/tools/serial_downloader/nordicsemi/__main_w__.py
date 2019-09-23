#
# Copyright (c) 2016 Nordic Semiconductor ASA
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
#   1. Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
#   2. Redistributions in binary form must reproduce the above copyright notice, this
#   list of conditions and the following disclaimer in the documentation and/or
#   other materials provided with the distribution.
#
#   3. Neither the name of Nordic Semiconductor ASA nor the names of other
#   contributors to this software may be used to endorse or promote products
#   derived from this software without specific prior written permission.
#
#   4. This software must only be used in or with a processor manufactured by Nordic
#   Semiconductor ASA, or in or with a processor manufactured by a third party that
#   is used in combination with a processor manufactured by Nordic Semiconductor.
#
#   5. Any software provided in binary or object form under this license must not be
#   reverse engineered, decompiled, modified and/or disassembled.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
import ipaddress
import signal

"""sdfu_w command line tool."""
SDFU_W_VERSION = "3.4.0"

# Python standard library
import sys
import click
import time
import logging
import os
import shutil
import tempfile
import binascii
import abc
import binascii
import struct
import datetime
import json
from enum import Enum
from array import array
from binascii import hexlify, unhexlify
from bisect import bisect_right

# 3rd party libraries
from serial import Serial
from zipfile import ZipFile
import hashlib
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
try:
    from ecdsa import SigningKey
    from ecdsa.curves import NIST256p
    from ecdsa.keys import sigencode_string
except Exception:
    print("Failed to import ecdsa, cannot do signing")

logger = logging.getLogger(__name__)

################################################################################
#from pc_ble_driver_py.exceptions import NordicSemiException
#ref https://github.com/NordicSemiconductor/pc-ble-driver-py/blob/master/python/pc_ble_driver_py/exceptions.py
################################################################################
class NordicSemiException(Exception):
    """
    Exception used as based exception for other exceptions defined in this package.
    """
    pass


class NotImplementedException(NordicSemiException):
    """
    Exception used when functionality has not been implemented yet.
    """
    pass


class InvalidArgumentException(NordicSemiException):
    """"
    Exception used when a argument is of wrong type
    """
    pass

class MissingArgumentException(NordicSemiException):
    """"
    Exception used when a argument is missing
    """
    pass


class IllegalStateException(NordicSemiException):
    """"
    Exception used when program is in an illegal state
    """
    pass


################################################################################
#from nordicsemi.dfu.util import query_func
#ref util.py
################################################################################
# From http://stackoverflow.com/questions/4472901/python-enum-class-with-tostring-fromstring
class NordicEnum(Enum):
  @classmethod
  def tostring(cls, val):
    for k,v in vars(cls).iteritems():
        if v==val:
            return k

  @classmethod
  def fromstring(cls, str):
      return getattr(cls, str.upper(), None)

# TODO: Create query function that maps query-result strings with functions
def query_func(question, default=False):
    """
    Ask a string question
    No input defaults to "no" which results in False
    """
    valid = {"yes": True, "y": True, "no": False, "n": False}
    if default is True:
        prompt = " [Y/n]"
    else:
        prompt = " [y/N]"

    while True:
        print("%s %s" % (question, prompt))
        choice = raw_input().lower()
        if choice == '':
            return default
        elif choice in valid:
            return valid[choice]
        else:
            print("Please respond with y/n")


def convert_uint16_to_array(value):
    """
    Converts a int to an array of 2 bytes (little endian)

    :param int value: int value to convert to list
    :return list[int]: list with 2 bytes
    """
    byte0 = value & 0xFF
    byte1 = (value >> 8) & 0xFF
    return [byte0, byte1]


def convert_uint32_to_array(value):
    """
    Converts a int to an array of 4 bytes (little endian)

    :param int value: int value to convert to list
    :return list[int]: list with 4 bytes
    """
    byte0 = value & 0xFF
    byte1 = (value >> 8) & 0xFF
    byte2 = (value >> 16) & 0xFF
    byte3 = (value >> 24) & 0xFF
    return [byte0, byte1, byte2, byte3]


def slip_parts_to_four_bytes(seq, dip, rp, pkt_type, pkt_len):
    """
    Creates a SLIP header.

    For a description of the SLIP header go to:
    http://developer.nordicsemi.com/nRF51_SDK/doc/7.2.0/s110/html/a00093.html

    :param int seq: Packet sequence number
    :param int dip: Data integrity check
    :param int rp: Reliable packet
    :param pkt_type: Payload packet
    :param pkt_len: Packet length
    :return: str with SLIP header
    """
    ints = [0, 0, 0, 0]
    ints[0] = seq | (((seq + 1) % 8) << 3) | (dip << 6) | (rp << 7)
    ints[1] = pkt_type | ((pkt_len & 0x000F) << 4)
    ints[2] = (pkt_len & 0x0FF0) >> 4
    ints[3] = (~(sum(ints[0:3])) + 1) & 0xFF

    return ''.join(chr(b) for b in ints)


def int32_to_bytes(value):
    """
    Converts a int to a str with 4 bytes

    :param value: int value to convert
    :return: str with 4 bytes
    """
    ints = [0, 0, 0, 0]
    ints[0] = (value & 0x000000FF)
    ints[1] = (value & 0x0000FF00) >> 8
    ints[2] = (value & 0x00FF0000) >> 16
    ints[3] = (value & 0xFF000000) >> 24
    return ''.join(chr(b) for b in ints)


def int16_to_bytes(value):
    """
    Converts a int to a str with 4 bytes

    :param value: int value to convert
    :return: str with 4 bytes
    """

    ints = [0, 0]
    ints[0] = (value & 0x00FF)
    ints[1] = (value & 0xFF00) >> 8
    return ''.join(chr(b) for b in ints)


def slip_decode_esc_chars(data):
    """Decode esc characters in a SLIP package.

    Replaces 0xDBDC with 0xCO and 0xDBDD with 0xDB.

    :return: str decoded data
    :type str data: data to decode
    """
    result = []
    while len(data):
        char = data.pop(0)
        if char == 0xDB:
            char2 = data.pop(0)
            if char2 == 0xDC:
                result.append(0xC0)
            elif char2 == 0xDD:
                result.append(0xDB)
            else:
                raise NordicSemiException('Char 0xDB NOT followed by 0xDC or 0xDD')
        else:
            result.append(char)
    return result


def slip_encode_esc_chars(data_in):
    """Encode esc characters in a SLIP package.

    Replace 0xCO  with 0xDBDC and 0xDB with 0xDBDD.

     :type str data_in: str to encode
     :return: str with encoded packet
    """
    result = []
    data = []
    for i in data_in:
        data.append(ord(i))

    while len(data):
        char = data.pop(0)
        if char == 0xC0:
            result.extend([0xDB, 0xDC])
        elif char == 0xDB:
            result.extend([0xDB, 0xDD])
        else:
            result.append(char)
    return ''.join(chr(i) for i in result)


################################################################################
#from nordicsemi.dfu.signing import Signing
#ref signing.py
################################################################################
keys_default_pem = """-----BEGIN EC PRIVATE KEY-----
MHcCAQEEIGvsrpXh8m/E9bj1dq/0o1aBPQVAFJQ6Pzusx685URE0oAoGCCqGSM49
AwEHoUQDQgAEaHYrUu/oFKIXN457GH+8IOuv6OIPBRLqoHjaEKM0wIzJZ0lhfO/A
53hKGjKEjYT3VNTQ3Zq1YB3o5QSQMP/LRg==
-----END EC PRIVATE KEY-----"""

class Signing(object):
    """
    Class for singing of hex-files
    """
    def gen_key(self, filename):
        """
        Generate a new Signing key using NIST P-256 curve
        """
        self.sk = SigningKey.generate(curve=NIST256p)

        with open(filename, "w") as sk_file:
            sk_file.write(self.sk.to_pem())

    def load_key(self, filename):
        """
        Load signing key (from pem file)
        """
        default_sk = SigningKey.from_pem(keys_default_pem)

        with open(filename, "r") as sk_file:
            sk_pem = sk_file.read()

        self.sk = SigningKey.from_pem(sk_pem)

        sk_hex = "".join(c.encode('hex') for c in self.sk.to_string())
        return default_sk.to_string() == self.sk.to_string()

    def sign(self, init_packet_data):
        """
        Create signature for init package using P-256 curve and SHA-256 as hashing algorithm
        Returns R and S keys combined in a 64 byte array
        """
        # Add assertion of init_packet
        if self.sk is None:
            raise IllegalStateException("Can't save key. No key created/loaded")

        # Sign the init-packet
        signature = self.sk.sign(init_packet_data, hashfunc=hashlib.sha256, sigencode=sigencode_string)
        return signature[31::-1] + signature[63:31:-1]

    def verify(self, init_packet, signature):
        """
        Verify init packet
        """
        # Add assertion of init_packet
        if self.sk is None:
            raise IllegalStateException("Can't save key. No key created/loaded")

        vk = self.sk.get_verifying_key()

        # Verify init packet
        try:
            vk.verify(signature, init_packet, hashfunc=hashlib.sha256)
        except:
            return False

        return True

    def get_vk(self, output_type, dbg):
        """
        Get public key (as hex, code or pem)
        """
        if self.sk is None:
            raise IllegalStateException("Can't get key. No key created/loaded")

        if output_type is None:
            raise InvalidArgumentException("Invalid output type for public key.")
        elif output_type == 'hex':
            return self.get_vk_hex()
        elif output_type == 'code':
            return self.get_vk_code(dbg)
        elif output_type == 'pem':
            return self.get_vk_pem()
        else:
            raise InvalidArgumentException("Invalid argument. Can't get key")

    def get_sk(self, output_type, dbg):
        """
        Get private key (as hex, code or pem)
        """
        if self.sk is None:
            raise IllegalStateException("Can't get key. No key created/loaded")

        if output_type is None:
            raise InvalidArgumentException("Invalid output type for private key.")
        elif output_type == 'hex':
            return self.get_sk_hex()
        elif output_type == 'code':
            raise InvalidArgumentException("Private key cannot be shown as code")
        elif output_type == 'pem':
            return self.sk.to_pem()
        else:
            raise InvalidArgumentException("Invalid argument. Can't get key")

    def get_sk_hex(self):
        """
        Get the verification key as hex
        """
        if self.sk is None:
            raise IllegalStateException("Can't get key. No key created/loaded")

        sk_hexlify = binascii.hexlify(self.sk.to_string())

        sk_hexlify_list = []
        for i in xrange(len(sk_hexlify)-2, -2, -2):
            sk_hexlify_list.append(sk_hexlify[i:i+2])

        sk_hexlify_list_str = ''.join(sk_hexlify_list)

        vk_hex = "Private (signing) key sk:\n{0}".format(sk_hexlify_list_str)

        return vk_hex

    def get_vk_hex(self):
        """
        Get the verification key as hex
        """
        if self.sk is None:
            raise IllegalStateException("Can't get key. No key created/loaded")

        vk = self.sk.get_verifying_key()
        vk_hexlify = binascii.hexlify(vk.to_string())

        vk_hexlify_list = []
        for i in xrange(len(vk_hexlify[0:64])-2, -2, -2):
            vk_hexlify_list.append(vk_hexlify[i:i+2])

        for i in xrange(len(vk_hexlify)-2, 62, -2):
            vk_hexlify_list.append(vk_hexlify[i:i+2])

        vk_hexlify_list_str = ''.join(vk_hexlify_list)

        vk_hex = "Public (verification) key pk:\n{0}".format(vk_hexlify_list_str)

        return vk_hex

    def wrap_code(self, key_code, dbg):

        header = """
/* This file was automatically generated by nrfutil on {0} */

#include "stdint.h"
#include "compiler_abstraction.h"
""".format(datetime.datetime.now().strftime("%Y-%m-%d (YY-MM-DD) at %H:%M:%S"))

        dbg_header="""
/* This file was generated with a throwaway private key, that is only inteded for a debug version of the DFU project.
  Please see https://github.com/NordicSemiconductor/pc-nrfutil/blob/master/README.md to generate a valid public key. */

#ifdef NRF_DFU_DEBUG_VERSION 
"""
        dbg_footer="""
#else
#error "Debug public key not valid for production. Please see https://github.com/NordicSemiconductor/pc-nrfutil/blob/master/README.md to generate it"
#endif
"""
        if dbg:
            code = header + dbg_header + key_code + dbg_footer
        else:
            code = header + key_code
        return code


    def get_vk_code(self, dbg):
        """
        Get the verification key as code
        """
        if self.sk is None:
            raise IllegalStateException("Can't get key. No key created/loaded")

        vk = self.sk.get_verifying_key()
        vk_hex = binascii.hexlify(vk.to_string())

        vk_x_separated = ""
        vk_x_str = vk_hex[0:64]
        for i in xrange(0, len(vk_x_str), 2):
            vk_x_separated = "0x" + vk_x_str[i:i+2] + ", " + vk_x_separated

        vk_y_separated = ""
        vk_y_str = vk_hex[64:128]
        for i in xrange(0, len(vk_y_str), 2):
            vk_y_separated = "0x" + vk_y_str[i:i+2] + ", " + vk_y_separated
        vk_y_separated = vk_y_separated[:-2]
        
        key_code ="""
/** @brief Public key used to verify DFU images */
__ALIGN(4) const uint8_t pk[64] =
{{
    {0}
    {1}
}};
"""
        key_code = key_code.format(vk_x_separated, vk_y_separated)
        vk_code = self.wrap_code(key_code, dbg)

        return vk_code

    def get_vk_pem(self):
        """
        Get the verification key as PEM
        """
        if self.sk is None:
            raise IllegalStateException("Can't get key. No key created/loaded")

        vk = self.sk.get_verifying_key()
        vk_pem = vk.to_pem()

        return vk_pem


################################################################################
#from .compat import asbytes, asstr
#ref compat.py
################################################################################
if sys.version_info[0] >= 3:
    def asbytes(s):
        if isinstance(s, bytes):
            return s
        return s.encode('latin1')
    def asstr(s):
        if isinstance(s, str):
            return s
        return s.decode('latin1')
else:
    asbytes = str
    asstr = str


################################################################################
#from nordicsemi.dfu.dfu_transport import DfuEvent
#ref dfu_transport.py
################################################################################
class DfuEvent:
    PROGRESS_EVENT = 1


class DfuTransport(object):
    """
    This class as an abstract base class inherited from when implementing transports.

    The class is generic in nature, the underlying implementation may have missing semantic
    than this class describes. But the intent is that the implementer shall follow the semantic as
    best as she can.
    """
    __metaclass__ = abc.ABCMeta

    OP_CODE = {
        'CreateObject'          : 0x01,
        'SetPRN'                : 0x02,
        'CalcChecSum'           : 0x03,
        'Execute'               : 0x04,
        'ReadObject'            : 0x06,
        'Response'              : 0x60,
    }

    RES_CODE = {
        'InvalidCode'           : 0x00,
        'Success'               : 0x01,
        'NotSupported'          : 0x02,
        'InvalidParameter'      : 0x03,
        'InsufficientResources' : 0x04,
        'InvalidObject'         : 0x05,
        'InvalidSignature'      : 0x06,
        'UnsupportedType'       : 0x07,
        'OperationNotPermitted' : 0x08,
        'OperationFailed'       : 0x0A,
        'ExtendedError'         : 0x0B,
    }

    EXT_ERROR_CODE = [
        "No extended error code has been set. This error indicates an implementation problem.",
        "Invalid error code. This error code should never be used outside of development.",
        "The format of the command was incorrect. This error code is not used in the current implementation, because @ref NRF_DFU_RES_CODE_OP_CODE_NOT_SUPPORTED and @ref NRF_DFU_RES_CODE_INVALID_PARAMETER cover all possible format errors.",
        "The command was successfully parsed, but it is not supported or unknown.",
        "The init command is invalid. The init packet either has an invalid update type or it is missing required fields for the update type (for example, the init packet for a SoftDevice update is missing the SoftDevice size field).",
        "The firmware version is too low. For an application, the version must be greater than the current application. For a bootloader, it must be greater than or equal to the current version. This requirement prevents downgrade attacks.""",
        "The hardware version of the device does not match the required hardware version for the update.",
        "The array of supported SoftDevices for the update does not contain the FWID of the current SoftDevice.",
        "The init packet does not contain a signature, but this bootloader requires all updates to have one.",
        "The hash type that is specified by the init packet is not supported by the DFU bootloader.",
        "The hash of the firmware image cannot be calculated.",
        "The type of the signature is unknown or not supported by the DFU bootloader.",
        "The hash of the received firmware image does not match the hash in the init packet.",
        "The available space on the device is insufficient to hold the firmware.",
        "The requested firmware to update was already present on the system.",
    ]

    @abc.abstractmethod
    def __init__(self):
        self.callbacks = {}


    @abc.abstractmethod
    def open(self):
        """
        Open a port if appropriate for the transport.
        :return:
        """
        pass


    @abc.abstractmethod
    def close(self):
        """
        Close a port if appropriate for the transport.
        :return:
        """
        pass

    @abc.abstractmethod
    def send_init_packet(self, init_packet):
        """
        Send init_packet to device.

        This call will block until init_packet is sent and transfer of packet is complete.

        :param init_packet: Init packet as a str.
        :return:
        """
        pass


    @abc.abstractmethod
    def send_firmware(self, firmware):
        """
        Start sending firmware to device.

        This call will block until transfer of firmware is complete.

        :param firmware:
        :return:
        """
        pass


    def register_events_callback(self, event_type, callback):
        """
        Register a callback.

        :param DfuEvent callback:
        :return: None
        """
        if event_type not in self.callbacks:
            self.callbacks[event_type] = []

        self.callbacks[event_type].append(callback)


    def _send_event(self, event_type, **kwargs):
        """
        Method for sending events to registered callbacks.

        If callbacks throws exceptions event propagation will stop and this method be part of the track trace.

        :param DfuEvent event_type:
        :param kwargs: Arguments to callback function
        :return:
        """
        if event_type in self.callbacks.keys():
            for callback in self.callbacks[event_type]:
                callback(**kwargs)


################################################################################
#from nordicsemi.dfu.dfu_transport_serial import DfuTransportSerial
#ref dfu_transport_serial.py
################################################################################
from datetime import datetime, timedelta
class ValidationException(NordicSemiException):
    """"
    Exception used when validation failed
    """
    pass


class Slip(object):
    SLIP_BYTE_END             = 0o300
    SLIP_BYTE_ESC             = 0o333
    SLIP_BYTE_ESC_END         = 0o334
    SLIP_BYTE_ESC_ESC         = 0o335

    SLIP_STATE_DECODING                 = 1
    SLIP_STATE_ESC_RECEIVED             = 2
    SLIP_STATE_CLEARING_INVALID_PACKET  = 3

    @staticmethod
    def encode(data):
        newData = []
        for elem in data:
            if elem == Slip.SLIP_BYTE_END:
                newData.append(Slip.SLIP_BYTE_ESC)
                newData.append(Slip.SLIP_BYTE_ESC_END)
            elif elem == Slip.SLIP_BYTE_ESC:
                newData.append(Slip.SLIP_BYTE_ESC)
                newData.append(Slip.SLIP_BYTE_ESC_ESC)
            else:
                newData.append(elem)
        newData.append(Slip.SLIP_BYTE_END)
        return newData

    @staticmethod
    def decode_add_byte(c, decoded_data, current_state):
        finished = False
        if current_state == Slip.SLIP_STATE_DECODING:
            if c == Slip.SLIP_BYTE_END:
                finished = True
            elif c == Slip.SLIP_BYTE_ESC:
                current_state = Slip.SLIP_STATE_ESC_RECEIVED
            else:
                decoded_data.append(c)
        elif current_state == Slip.SLIP_STATE_ESC_RECEIVED:
            if c == Slip.SLIP_BYTE_ESC_END:
                decoded_data.append(Slip.SLIP_BYTE_END)
                current_state = Slip.SLIP_STATE_DECODING
            elif c == Slip.SLIP_BYTE_ESC_ESC:
                decoded_data.append(Slip.SLIP_BYTE_ESC)
                current_state = Slip.SLIP_STATE_DECODING
            else:
                current_state = Slip.SLIP_STATE_CLEARING_INVALID_PACKET
        elif current_state == Slip.SLIP_STATE_CLEARING_INVALID_PACKET:
            if c == Slip.SLIP_BYTE_END:
                current_state = Slip.SLIP_STATE_DECODING
                decoded_data = []

        return (finished, current_state, decoded_data)

class DFUAdapter(object):
    def __init__(self, serial_port):
        self.serial_port = serial_port

    def send_message(self, data):
        packet = Slip.encode(data)
        self.serial_port.write(packet)

    def get_message(self):
        current_state = Slip.SLIP_STATE_DECODING
        finished = False
        decoded_data = []

        while finished == False:
            byte = self.serial_port.read(1)
            if byte:
                (byte) = struct.unpack('B', byte)[0]
                (finished, current_state, decoded_data) \
                   = Slip.decode_add_byte(byte, decoded_data, current_state)
            else:
                current_state = Slip.SLIP_STATE_CLEARING_INVALID_PACKET
                return None

        return decoded_data

class DfuTransportSerial(DfuTransport):

    DEFAULT_BAUD_RATE = 115200
    DEFAULT_FLOW_CONTROL = True
    DEFAULT_SERIAL_PORT_TIMEOUT = 1.0  # Timeout time on serial port read
    DEFAULT_PRN                 = 0
    DEFAULT_DO_PING = True

    OP_CODE = {
        'CreateObject'          : 0x01,
        'SetPRN'                : 0x02,
        'CalcChecSum'           : 0x03,
        'Execute'               : 0x04,
        'ReadError'             : 0x05,
        'ReadObject'            : 0x06,
        'GetSerialMTU'          : 0x07,
        'WriteObject'           : 0x08,
        'Ping'                  : 0x09,
        'Response'              : 0x60,
    }

    def __init__(self,
                 com_port,
                 baud_rate=DEFAULT_BAUD_RATE,
                 flow_control=DEFAULT_FLOW_CONTROL,
                 timeout=DEFAULT_SERIAL_PORT_TIMEOUT,
                 prn=DEFAULT_PRN,
                 do_ping=DEFAULT_DO_PING):

        super(DfuTransportSerial, self).__init__()
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.flow_control = 1 if flow_control else 0
        self.timeout = timeout
        self.prn         = prn
        self.serial_port = None
        self.dfu_adapter = None
        self.ping_id     = 0
        self.do_ping     = do_ping

        self.mtu         = 0

        """:type: serial.Serial """


    def open(self):
        super(DfuTransportSerial, self).open()

        try:
            self.serial_port = Serial(port=self.com_port,
                baudrate=self.baud_rate, rtscts=self.flow_control, timeout=self.timeout)
            self.dfu_adapter = DFUAdapter(self.serial_port)
        except Exception as e:
            raise NordicSemiException("Serial port could not be opened on {0}"
            + ". Reason: {1}".format(self.com_port, e.message))

        if self.do_ping:
            ping_success = False
            start = datetime.now()
            while datetime.now() - start < timedelta(seconds=self.timeout):
                if self.__ping() == True:
                    ping_success = True
                time.sleep(1)

            if ping_success == False:
                raise NordicSemiException("No ping response after opening COM port")

        self.__set_prn()
        self.__get_mtu()

    def close(self):
        super(DfuTransportSerial, self).close()
        self.serial_port.close()

    def send_init_packet(self, init_packet):
        def try_to_recover():
            if response['offset'] == 0 or response['offset'] > len(init_packet):
                # There is no init packet or present init packet is too long.
                return False

            expected_crc = (binascii.crc32(init_packet[:response['offset']]) & 0xFFFFFFFF)

            if expected_crc != response['crc']:
                # Present init packet is invalid.
                return False

            if len(init_packet) > response['offset']:
                # Send missing part.
                try:
                    self.__stream_data(data     = init_packet[response['offset']:],
                                       crc      = expected_crc,
                                       offset   = response['offset'])
                except ValidationException:
                    return False

            self.__execute()
            return True

        response = self.__select_command()
        assert len(init_packet) <= response['max_size'], 'Init command is too long'

        if try_to_recover():
            return

        try:
            self.__create_command(len(init_packet))
            self.__stream_data(data=init_packet)
            self.__execute()
        except ValidationException:
            raise NordicSemiException("Failed to send init packet")

    def send_firmware(self, firmware):
        def try_to_recover():
            if response['offset'] == 0:
                # Nothing to recover
                return

            expected_crc = binascii.crc32(firmware[:response['offset']]) & 0xFFFFFFFF
            remainder    = response['offset'] % response['max_size']

            if expected_crc != response['crc']:
                # Invalid CRC. Remove corrupted data.
                response['offset'] -= remainder if remainder != 0 else response['max_size']
                response['crc']     = \
                        binascii.crc32(firmware[:response['offset']]) & 0xFFFFFFFF
                return

            if (remainder != 0) and (response['offset'] != len(firmware)):
                # Send rest of the page.
                try:
                    to_send             = firmware[response['offset'] : response['offset']
                                                + response['max_size'] - remainder]
                    response['crc']     = self.__stream_data(data   = to_send,
                                                             crc    = response['crc'],
                                                             offset = response['offset'])
                    response['offset'] += len(to_send)
                except ValidationException:
                    # Remove corrupted data.
                    response['offset'] -= remainder
                    response['crc']     = \
                        binascii.crc32(firmware[:response['offset']]) & 0xFFFFFFFF
                    return

            self.__execute()
            self._send_event(event_type=DfuEvent.PROGRESS_EVENT, progress=response['offset'])

        response = self.__select_data()
        try_to_recover()
        for i in range(response['offset'], len(firmware), response['max_size']):
            data = firmware[i:i+response['max_size']]
            try:
                self.__create_data(len(data))
                response['crc'] = self.__stream_data(data=data, crc=response['crc'], offset=i)
                self.__execute()
            except ValidationException:
                raise NordicSemiException("Failed to send firmware")

            self._send_event(event_type=DfuEvent.PROGRESS_EVENT, progress=len(data))

    def __set_prn(self):
        logger.debug("Serial: Set Packet Receipt Notification {}".format(self.prn))
        self.dfu_adapter.send_message([DfuTransportSerial.OP_CODE['SetPRN']]
            + map(ord, struct.pack('<H', self.prn)))
        self.__get_response(DfuTransportSerial.OP_CODE['SetPRN'])

    def __get_mtu(self):
        self.dfu_adapter.send_message([DfuTransportSerial.OP_CODE['GetSerialMTU']])
        response = self.__get_response(DfuTransportSerial.OP_CODE['GetSerialMTU'])

        self.mtu = struct.unpack('<H', bytearray(response))[0]

    def __ping(self):
        self.ping_id = (self.ping_id + 1) % 256

        self.dfu_adapter.send_message([DfuTransportSerial.OP_CODE['Ping'], self.ping_id])
        resp = self.dfu_adapter.get_message() # Receive raw reponse to check return code

        if (resp == None):
            logger.debug('Serial: No ping response')
            return False

        if resp[0] != DfuTransportSerial.OP_CODE['Response']:
            logger.debug('Serial: No Response: 0x{:02X}'.format(resp[0]))
            return False

        if resp[1] != DfuTransportSerial.OP_CODE['Ping']:
            logger.debug('Serial: Unexpected Executed OP_CODE.\n' \
                + 'Expected: 0x{:02X} Received: 0x{:02X}'.format(DfuTransportSerial.OP_CODE['Ping'], resp[1]))
            return False

        if resp[2] != DfuTransport.RES_CODE['Success']:
            # Returning an error code is seen as good enough. The bootloader is up and running
            return True
        else:
            if struct.unpack('B', bytearray(resp[3:]))[0] == self.ping_id:
                return True
            else:
                return False

    def __create_command(self, size):
        self.__create_object(0x01, size)

    def __create_data(self, size):
        self.__create_object(0x02, size)

    def __create_object(self, object_type, size):
        self.dfu_adapter.send_message([DfuTransportSerial.OP_CODE['CreateObject'], object_type]\
                                            + map(ord, struct.pack('<L', size)))
        self.__get_response(DfuTransportSerial.OP_CODE['CreateObject'])

    def __calculate_checksum(self):
        self.dfu_adapter.send_message([DfuTransportSerial.OP_CODE['CalcChecSum']])
        response = self.__get_response(DfuTransportSerial.OP_CODE['CalcChecSum'])

        (offset, crc) = struct.unpack('<II', bytearray(response))
        return {'offset': offset, 'crc': crc}

    def __execute(self):
        self.dfu_adapter.send_message([DfuTransportSerial.OP_CODE['Execute']])
        self.__get_response(DfuTransportSerial.OP_CODE['Execute'])

    def __select_command(self):
        return self.__select_object(0x01)

    def __select_data(self):
        return self.__select_object(0x02)

    def __select_object(self, object_type):
        logger.debug("Serial: Selecting Object: type:{}".format(object_type))
        self.dfu_adapter.send_message([DfuTransportSerial.OP_CODE['ReadObject'], object_type])

        response = self.__get_response(DfuTransportSerial.OP_CODE['ReadObject'])
        (max_size, offset, crc)= struct.unpack('<III', bytearray(response))

        logger.debug("Serial: Object selected: " +
            " max_size:{} offset:{} crc:{}".format(max_size, offset, crc))
        return {'max_size': max_size, 'offset': offset, 'crc': crc}

    def __get_checksum_response(self):
        resp = self.__get_response(DfuTransportSerial.OP_CODE['CalcChecSum'])

        (offset, crc) = struct.unpack('<II', bytearray(resp))
        return {'offset': offset, 'crc': crc}

    def __stream_data(self, data, crc=0, offset=0):
        logger.debug("Serial: Streaming Data: " +
            "len:{0} offset:{1} crc:0x{2:08X}".format(len(data), offset, crc))
        def validate_crc():
            if (crc != response['crc']):
                raise ValidationException('Failed CRC validation.\n'\
                                + 'Expected: {} Recieved: {}.'.format(crc, response['crc']))
            if (offset != response['offset']):
                raise ValidationException('Failed offset validation.\n'\
                                + 'Expected: {} Recieved: {}.'.format(offset, response['offset']))

        current_pnr     = 0

        for i in range(0, len(data), (self.mtu-1)/2 - 1):
            # append the write data opcode to the front
            # here the maximum data size is self.mtu/2,
            # due to the slip encoding which at maximum doubles the size
            to_transmit = data[i:i + (self.mtu-1)/2 - 1 ]
            to_transmit = struct.pack('B',DfuTransportSerial.OP_CODE['WriteObject']) + to_transmit

            self.dfu_adapter.send_message(map(ord, to_transmit))
            crc     = binascii.crc32(to_transmit[1:], crc) & 0xFFFFFFFF
            offset += len(to_transmit) - 1
            current_pnr    += 1
            if self.prn == current_pnr:
                current_pnr = 0
                response    = self.__get_checksum_response()
                validate_crc()
        response = self.__calculate_checksum()
        validate_crc()
        return crc

    def __get_response(self, operation):
        def get_dict_key(dictionary, value):
            return next((key for key, val in dictionary.items() if val == value), None)

        resp = self.dfu_adapter.get_message()

        if resp == None:
            return None

        if resp[0] != DfuTransportSerial.OP_CODE['Response']:
            raise NordicSemiException('No Response: 0x{:02X}'.format(resp[0]))

        if resp[1] != operation:
            raise NordicSemiException('Unexpected Executed OP_CODE.\n' \
                             + 'Expected: 0x{:02X} Received: 0x{:02X}'.format(operation, resp[1]))

        if resp[2] == DfuTransport.RES_CODE['Success']:
            return resp[3:]

        elif resp[2] == DfuTransport.RES_CODE['ExtendedError']:
            try:
                data = DfuTransport.EXT_ERROR_CODE[resp[3]]
            except IndexError:
                data = "Unsupported extended error type {}".format(resp[3])
            raise NordicSemiException('Extended Error 0x{:02X}: {}'.format(resp[3], data))
        else:
            raise NordicSemiException('Response Code {}'.format(
                get_dict_key(DfuTransport.RES_CODE, resp[2])))


################################################################################
#from nordicsemi.dfu.crc16 import *
#ref crc16.py
################################################################################
def calc_crc16(binary_data, crc=0xffff):
    """
    Calculates CRC16 on binary_data

    :param int crc: CRC value to start calculation with
    :param bytearray binary_data: Array with data to run CRC16 calculation on
    :return int: Calculated CRC value of binary_data
    """

    for b in binary_data:
        crc = (crc >> 8 & 0x00FF) | (crc << 8 & 0xFF00)
        crc ^= ord(b)
        crc ^= (crc & 0x00FF) >> 4
        crc ^= (crc << 8) << 4
        crc ^= ((crc & 0x00FF) << 4) << 1
    return crc & 0xFFFF


################################################################################
#from nordicsemi.dfu.model import HexType, FirmwareKeys
#ref model.py
################################################################################
class HexType(Enum):
    SOFTDEVICE = 1
    BOOTLOADER = 2
    SD_BL = 3
    APPLICATION = 4


class FirmwareKeys(Enum):
    ENCRYPT = 1
    FIRMWARE_FILENAME = 2
    BIN_FILENAME = 3
    DAT_FILENAME = 4
    INIT_PACKET_DATA = 5
    SD_SIZE = 6
    BL_SIZE = 7


################################################################################
#from nordicsemi.dfu.manifest import ManifestGenerator, Manifest
#ref manifest.py
################################################################################
class ManifestGenerator(object):
    def __init__(self, firmwares_data):
        """
        The Manifest Generator constructor. Needs a data structure to generate a manifest from.

        :type dict firmwares_data: The firmwares data structure describing the Nordic DFU package
        """
        self.firmwares_data = firmwares_data
        self.manifest = None

    def generate_manifest(self):
        self.manifest = Manifest()

        for key in self.firmwares_data:
            firmware_dict = self.firmwares_data[key]

            if key == HexType.SD_BL:
                _firmware = SoftdeviceBootloaderFirmware()
                _firmware.info_read_only_metadata = FWMetaData()
                _firmware.info_read_only_metadata.bl_size = firmware_dict[FirmwareKeys.BL_SIZE]
                _firmware.info_read_only_metadata.sd_size = firmware_dict[FirmwareKeys.SD_SIZE]
            else:
                _firmware = Firmware()


            # Strip path, add only filename
            _firmware.bin_file = os.path.basename(firmware_dict[FirmwareKeys.BIN_FILENAME])
            _firmware.dat_file = os.path.basename(firmware_dict[FirmwareKeys.DAT_FILENAME])

            if key == HexType.APPLICATION:
                self.manifest.application = _firmware
            elif key == HexType.BOOTLOADER:
                self.manifest.bootloader = _firmware
            elif key == HexType.SOFTDEVICE:
                self.manifest.softdevice = _firmware
            elif key == HexType.SD_BL:
                self.manifest.softdevice_bootloader = _firmware
            else:
                raise NotImplementedException("Support for firmware type {0} not implemented yet.".format(key))

        return self.to_json()

    def to_json(self):
        def remove_none_entries(d):
            if not isinstance(d, dict):
                return d

            return dict((k, remove_none_entries(v)) for k, v in d.iteritems() if v is not None)

        return json.dumps({'manifest': self.manifest},
                          default=lambda o: remove_none_entries(o.__dict__),
                          sort_keys=True, indent=4,
                          separators=(',', ': '))


class FWMetaData(object):
    def __init__(self,
                 is_debug=None,
                 hw_version=None,
                 fw_version=None,
                 softdevice_req=None,
                 sd_size=None,
                 bl_size=None
                 ):
        """
        The FWMetaData data model.

        :param bool is_debug:  debug mode on
        :param int hw_version:  hardware version
        :param int fw_version:  application or bootloader version
        :param list softdevice_req: softdevice requirements
        :param int sd_size SoftDevice size
        :param int bl_size Bootloader size
        :return:FWMetaData 
        """
        self.is_debug = is_debug
        self.hw_version = hw_version
        self.fw_version = fw_version
        self.softdevice_req = softdevice_req
        self.sd_size = sd_size
        self.bl_size = bl_size


class Firmware(object):
    def __init__(self,
                 bin_file=None,
                 dat_file=None,
                 info_read_only_metadata=None):
        """
        The firmware datamodel

        :param str bin_file: Firmware binary file
        :param str dat_file: Firmware .dat file (init packet for Nordic DFU)
        :param int info_read_only_metadata: The metadata about this firwmare image
        :return:
        """
        self.dat_file = dat_file
        self.bin_file = bin_file

        if info_read_only_metadata:
            self.info_read_only_metadata = FWMetaData(**info_read_only_metadata)
        else:
            self.info_read_only_metadata = None


class SoftdeviceBootloaderFirmware(Firmware):
    def __init__(self,
                 bin_file=None,
                 dat_file=None,
                 info_read_only_metadata=None):
        """
        The SoftdeviceBootloaderFirmware data model

        :param str bin_file: Firmware binary file
        :param str dat_file: Firmware .dat file (init packet for Nordic DFU)
        :param int info_read_only_metadata: The metadata about this firwmare image
        :return: SoftdeviceBootloaderFirmware
        """
        super(SoftdeviceBootloaderFirmware, self).__init__(
            bin_file,
            dat_file,
            info_read_only_metadata)

class Manifest:
    def __init__(self,
                 application=None,
                 bootloader=None,
                 softdevice=None,
                 softdevice_bootloader=None):
        """
        The Manifest data model.

        :param dict application: Application firmware in package
        :param dict bootloader: Bootloader firmware in package
        :param dict softdevice: Softdevice firmware in package
        :param dict softdevice_bootloader: Combined softdevice and bootloader firmware in package
        :return: Manifest
        """
        self.softdevice_bootloader = \
            SoftdeviceBootloaderFirmware(**softdevice_bootloader) if softdevice_bootloader else None

        self.softdevice = Firmware(**softdevice) if softdevice else None
        self.bootloader = Firmware(**bootloader) if bootloader else None
        self.application = Firmware(**application) if application else None

    @staticmethod
    def from_json(data):
        """
        Parses a manifest according to Nordic DFU package specification.

        :param str data: The manifest in string format
        :return: Manifest
        """
        kwargs = json.loads(data)
        return Manifest(**kwargs['manifest'])


################################################################################
#from . import dfu_cc_pb2 as pb
#ref dfu_cc_pb2.py
################################################################################
class pb(Enum):
    _sym_db = _symbol_database.Default()

    DESCRIPTOR = _descriptor.FileDescriptor(
      name='dfu-cc.proto',
      package='dfu',
      serialized_pb=_b('\n\x0c\x64\x66u-cc.proto\x12\x03\x64\x66u\"6\n\x04Hash\x12 \n\thash_type\x18\x01 \x02(\x0e\x32\r.dfu.HashType\x12\x0c\n\x04hash\x18\x02 \x02(\x0c\"\xca\x01\n\x0bInitCommand\x12\x12\n\nfw_version\x18\x01 \x01(\r\x12\x12\n\nhw_version\x18\x02 \x01(\r\x12\x12\n\x06sd_req\x18\x03 \x03(\rB\x02\x10\x01\x12\x19\n\x04type\x18\x04 \x01(\x0e\x32\x0b.dfu.FwType\x12\x0f\n\x07sd_size\x18\x05 \x01(\r\x12\x0f\n\x07\x62l_size\x18\x06 \x01(\r\x12\x10\n\x08\x61pp_size\x18\x07 \x01(\r\x12\x17\n\x04hash\x18\x08 \x01(\x0b\x32\t.dfu.Hash\x12\x17\n\x08is_debug\x18\t \x01(\x08:\x05\x66\x61lse\"\x1f\n\x0cResetCommand\x12\x0f\n\x07timeout\x18\x01 \x02(\r\"i\n\x07\x43ommand\x12\x1c\n\x07op_code\x18\x01 \x01(\x0e\x32\x0b.dfu.OpCode\x12\x1e\n\x04init\x18\x02 \x01(\x0b\x32\x10.dfu.InitCommand\x12 \n\x05reset\x18\x03 \x01(\x0b\x32\x11.dfu.ResetCommand\"m\n\rSignedCommand\x12\x1d\n\x07\x63ommand\x18\x01 \x02(\x0b\x32\x0c.dfu.Command\x12*\n\x0esignature_type\x18\x02 \x02(\x0e\x32\x12.dfu.SignatureType\x12\x11\n\tsignature\x18\x03 \x02(\x0c\"S\n\x06Packet\x12\x1d\n\x07\x63ommand\x18\x01 \x01(\x0b\x32\x0c.dfu.Command\x12*\n\x0esigned_command\x18\x02 \x01(\x0b\x32\x12.dfu.SignedCommand*\x1d\n\x06OpCode\x12\t\n\x05RESET\x10\x00\x12\x08\n\x04INIT\x10\x01*T\n\x06\x46wType\x12\x0f\n\x0b\x41PPLICATION\x10\x00\x12\x0e\n\nSOFTDEVICE\x10\x01\x12\x0e\n\nBOOTLOADER\x10\x02\x12\x19\n\x15SOFTDEVICE_BOOTLOADER\x10\x03*D\n\x08HashType\x12\x0b\n\x07NO_HASH\x10\x00\x12\x07\n\x03\x43RC\x10\x01\x12\n\n\x06SHA128\x10\x02\x12\n\n\x06SHA256\x10\x03\x12\n\n\x06SHA512\x10\x04*3\n\rSignatureType\x12\x15\n\x11\x45\x43\x44SA_P256_SHA256\x10\x00\x12\x0b\n\x07\x45\x44\x32\x35\x35\x31\x39\x10\x01')
    )
    _sym_db.RegisterFileDescriptor(DESCRIPTOR)

    _OPCODE = _descriptor.EnumDescriptor(
      name='OpCode',
      full_name='dfu.OpCode',
      filename=None,
      file=DESCRIPTOR,
      values=[
        _descriptor.EnumValueDescriptor(
          name='RESET', index=0, number=0,
          options=None,
          type=None),
        _descriptor.EnumValueDescriptor(
          name='INIT', index=1, number=1,
          options=None,
          type=None),
      ],
      containing_type=None,
      options=None,
      serialized_start=618,
      serialized_end=647,
    )
    _sym_db.RegisterEnumDescriptor(_OPCODE)

    OpCode = enum_type_wrapper.EnumTypeWrapper(_OPCODE)
    _FWTYPE = _descriptor.EnumDescriptor(
      name='FwType',
      full_name='dfu.FwType',
      filename=None,
      file=DESCRIPTOR,
      values=[
        _descriptor.EnumValueDescriptor(
          name='APPLICATION', index=0, number=0,
          options=None,
          type=None),
        _descriptor.EnumValueDescriptor(
          name='SOFTDEVICE', index=1, number=1,
          options=None,
          type=None),
        _descriptor.EnumValueDescriptor(
          name='BOOTLOADER', index=2, number=2,
          options=None,
          type=None),
        _descriptor.EnumValueDescriptor(
          name='SOFTDEVICE_BOOTLOADER', index=3, number=3,
          options=None,
          type=None),
      ],
      containing_type=None,
      options=None,
      serialized_start=649,
      serialized_end=733,
    )
    _sym_db.RegisterEnumDescriptor(_FWTYPE)

    FwType = enum_type_wrapper.EnumTypeWrapper(_FWTYPE)
    _HASHTYPE = _descriptor.EnumDescriptor(
      name='HashType',
      full_name='dfu.HashType',
      filename=None,
      file=DESCRIPTOR,
      values=[
        _descriptor.EnumValueDescriptor(
          name='NO_HASH', index=0, number=0,
          options=None,
          type=None),
        _descriptor.EnumValueDescriptor(
          name='CRC', index=1, number=1,
          options=None,
          type=None),
        _descriptor.EnumValueDescriptor(
          name='SHA128', index=2, number=2,
          options=None,
          type=None),
        _descriptor.EnumValueDescriptor(
          name='SHA256', index=3, number=3,
          options=None,
          type=None),
        _descriptor.EnumValueDescriptor(
          name='SHA512', index=4, number=4,
          options=None,
          type=None),
      ],
      containing_type=None,
      options=None,
      serialized_start=735,
      serialized_end=803,
    )
    _sym_db.RegisterEnumDescriptor(_HASHTYPE)

    HashType = enum_type_wrapper.EnumTypeWrapper(_HASHTYPE)
    _SIGNATURETYPE = _descriptor.EnumDescriptor(
      name='SignatureType',
      full_name='dfu.SignatureType',
      filename=None,
      file=DESCRIPTOR,
      values=[
        _descriptor.EnumValueDescriptor(
          name='ECDSA_P256_SHA256', index=0, number=0,
          options=None,
          type=None),
        _descriptor.EnumValueDescriptor(
          name='ED25519', index=1, number=1,
          options=None,
          type=None),
      ],
      containing_type=None,
      options=None,
      serialized_start=805,
      serialized_end=856,
    )
    _sym_db.RegisterEnumDescriptor(_SIGNATURETYPE)

    SignatureType = enum_type_wrapper.EnumTypeWrapper(_SIGNATURETYPE)
    RESET = 0
    INIT = 1
    APPLICATION = 0
    SOFTDEVICE = 1
    BOOTLOADER = 2
    SOFTDEVICE_BOOTLOADER = 3
    NO_HASH = 0
    CRC = 1
    SHA128 = 2
    SHA256 = 3
    SHA512 = 4
    ECDSA_P256_SHA256 = 0
    ED25519 = 1



    _HASH = _descriptor.Descriptor(
      name='Hash',
      full_name='dfu.Hash',
      filename=None,
      file=DESCRIPTOR,
      containing_type=None,
      fields=[
        _descriptor.FieldDescriptor(
          name='hash_type', full_name='dfu.Hash.hash_type', index=0,
          number=1, type=14, cpp_type=8, label=2,
          has_default_value=False, default_value=0,
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
        _descriptor.FieldDescriptor(
          name='hash', full_name='dfu.Hash.hash', index=1,
          number=2, type=12, cpp_type=9, label=2,
          has_default_value=False, default_value=_b(""),
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
      ],
      extensions=[
      ],
      nested_types=[],
      enum_types=[
      ],
      options=None,
      is_extendable=False,
      extension_ranges=[],
      oneofs=[
      ],
      serialized_start=21,
      serialized_end=75,
    )


    _INITCOMMAND = _descriptor.Descriptor(
      name='InitCommand',
      full_name='dfu.InitCommand',
      filename=None,
      file=DESCRIPTOR,
      containing_type=None,
      fields=[
        _descriptor.FieldDescriptor(
          name='fw_version', full_name='dfu.InitCommand.fw_version', index=0,
          number=1, type=13, cpp_type=3, label=1,
          has_default_value=False, default_value=0,
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
        _descriptor.FieldDescriptor(
          name='hw_version', full_name='dfu.InitCommand.hw_version', index=1,
          number=2, type=13, cpp_type=3, label=1,
          has_default_value=False, default_value=0,
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
        _descriptor.FieldDescriptor(
          name='sd_req', full_name='dfu.InitCommand.sd_req', index=2,
          number=3, type=13, cpp_type=3, label=3,
          has_default_value=False, default_value=[],
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\020\001'))),
        _descriptor.FieldDescriptor(
          name='type', full_name='dfu.InitCommand.type', index=3,
          number=4, type=14, cpp_type=8, label=1,
          has_default_value=False, default_value=0,
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
        _descriptor.FieldDescriptor(
          name='sd_size', full_name='dfu.InitCommand.sd_size', index=4,
          number=5, type=13, cpp_type=3, label=1,
          has_default_value=False, default_value=0,
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
        _descriptor.FieldDescriptor(
          name='bl_size', full_name='dfu.InitCommand.bl_size', index=5,
          number=6, type=13, cpp_type=3, label=1,
          has_default_value=False, default_value=0,
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
        _descriptor.FieldDescriptor(
          name='app_size', full_name='dfu.InitCommand.app_size', index=6,
          number=7, type=13, cpp_type=3, label=1,
          has_default_value=False, default_value=0,
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
        _descriptor.FieldDescriptor(
          name='hash', full_name='dfu.InitCommand.hash', index=7,
          number=8, type=11, cpp_type=10, label=1,
          has_default_value=False, default_value=None,
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
        _descriptor.FieldDescriptor(
          name='is_debug', full_name='dfu.InitCommand.is_debug', index=8,
          number=9, type=8, cpp_type=7, label=1,
          has_default_value=True, default_value=False,
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
      ],
      extensions=[
      ],
      nested_types=[],
      enum_types=[
      ],
      options=None,
      is_extendable=False,
      extension_ranges=[],
      oneofs=[
      ],
      serialized_start=78,
      serialized_end=280,
    )


    _RESETCOMMAND = _descriptor.Descriptor(
      name='ResetCommand',
      full_name='dfu.ResetCommand',
      filename=None,
      file=DESCRIPTOR,
      containing_type=None,
      fields=[
        _descriptor.FieldDescriptor(
          name='timeout', full_name='dfu.ResetCommand.timeout', index=0,
          number=1, type=13, cpp_type=3, label=2,
          has_default_value=False, default_value=0,
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
      ],
      extensions=[
      ],
      nested_types=[],
      enum_types=[
      ],
      options=None,
      is_extendable=False,
      extension_ranges=[],
      oneofs=[
      ],
      serialized_start=282,
      serialized_end=313,
    )


    _COMMAND = _descriptor.Descriptor(
      name='Command',
      full_name='dfu.Command',
      filename=None,
      file=DESCRIPTOR,
      containing_type=None,
      fields=[
        _descriptor.FieldDescriptor(
          name='op_code', full_name='dfu.Command.op_code', index=0,
          number=1, type=14, cpp_type=8, label=1,
          has_default_value=False, default_value=0,
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
        _descriptor.FieldDescriptor(
          name='init', full_name='dfu.Command.init', index=1,
          number=2, type=11, cpp_type=10, label=1,
          has_default_value=False, default_value=None,
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
        _descriptor.FieldDescriptor(
          name='reset', full_name='dfu.Command.reset', index=2,
          number=3, type=11, cpp_type=10, label=1,
          has_default_value=False, default_value=None,
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
      ],
      extensions=[
      ],
      nested_types=[],
      enum_types=[
      ],
      options=None,
      is_extendable=False,
      extension_ranges=[],
      oneofs=[
      ],
      serialized_start=315,
      serialized_end=420,
    )


    _SIGNEDCOMMAND = _descriptor.Descriptor(
      name='SignedCommand',
      full_name='dfu.SignedCommand',
      filename=None,
      file=DESCRIPTOR,
      containing_type=None,
      fields=[
        _descriptor.FieldDescriptor(
          name='command', full_name='dfu.SignedCommand.command', index=0,
          number=1, type=11, cpp_type=10, label=2,
          has_default_value=False, default_value=None,
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
        _descriptor.FieldDescriptor(
          name='signature_type', full_name='dfu.SignedCommand.signature_type', index=1,
          number=2, type=14, cpp_type=8, label=2,
          has_default_value=False, default_value=0,
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
        _descriptor.FieldDescriptor(
          name='signature', full_name='dfu.SignedCommand.signature', index=2,
          number=3, type=12, cpp_type=9, label=2,
          has_default_value=False, default_value=_b(""),
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
      ],
      extensions=[
      ],
      nested_types=[],
      enum_types=[
      ],
      options=None,
      is_extendable=False,
      extension_ranges=[],
      oneofs=[
      ],
      serialized_start=422,
      serialized_end=531,
    )


    _PACKET = _descriptor.Descriptor(
      name='Packet',
      full_name='dfu.Packet',
      filename=None,
      file=DESCRIPTOR,
      containing_type=None,
      fields=[
        _descriptor.FieldDescriptor(
          name='command', full_name='dfu.Packet.command', index=0,
          number=1, type=11, cpp_type=10, label=1,
          has_default_value=False, default_value=None,
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
        _descriptor.FieldDescriptor(
          name='signed_command', full_name='dfu.Packet.signed_command', index=1,
          number=2, type=11, cpp_type=10, label=1,
          has_default_value=False, default_value=None,
          message_type=None, enum_type=None, containing_type=None,
          is_extension=False, extension_scope=None,
          options=None),
      ],
      extensions=[
      ],
      nested_types=[],
      enum_types=[
      ],
      options=None,
      is_extendable=False,
      extension_ranges=[],
      oneofs=[
      ],
      serialized_start=533,
      serialized_end=616,
    )

    _HASH.fields_by_name['hash_type'].enum_type = _HASHTYPE
    _INITCOMMAND.fields_by_name['type'].enum_type = _FWTYPE
    _INITCOMMAND.fields_by_name['hash'].message_type = _HASH
    _COMMAND.fields_by_name['op_code'].enum_type = _OPCODE
    _COMMAND.fields_by_name['init'].message_type = _INITCOMMAND
    _COMMAND.fields_by_name['reset'].message_type = _RESETCOMMAND
    _SIGNEDCOMMAND.fields_by_name['command'].message_type = _COMMAND
    _SIGNEDCOMMAND.fields_by_name['signature_type'].enum_type = _SIGNATURETYPE
    _PACKET.fields_by_name['command'].message_type = _COMMAND
    _PACKET.fields_by_name['signed_command'].message_type = _SIGNEDCOMMAND
    DESCRIPTOR.message_types_by_name['Hash'] = _HASH
    DESCRIPTOR.message_types_by_name['InitCommand'] = _INITCOMMAND
    DESCRIPTOR.message_types_by_name['ResetCommand'] = _RESETCOMMAND
    DESCRIPTOR.message_types_by_name['Command'] = _COMMAND
    DESCRIPTOR.message_types_by_name['SignedCommand'] = _SIGNEDCOMMAND
    DESCRIPTOR.message_types_by_name['Packet'] = _PACKET
    DESCRIPTOR.enum_types_by_name['OpCode'] = _OPCODE
    DESCRIPTOR.enum_types_by_name['FwType'] = _FWTYPE
    DESCRIPTOR.enum_types_by_name['HashType'] = _HASHTYPE
    DESCRIPTOR.enum_types_by_name['SignatureType'] = _SIGNATURETYPE

    Hash = _reflection.GeneratedProtocolMessageType('Hash', (_message.Message,), dict(
      DESCRIPTOR = _HASH,
      __module__ = 'dfu_cc_pb2'
      # @@protoc_insertion_point(class_scope:dfu.Hash)
      ))
    _sym_db.RegisterMessage(Hash)

    InitCommand = _reflection.GeneratedProtocolMessageType('InitCommand', (_message.Message,), dict(
      DESCRIPTOR = _INITCOMMAND,
      __module__ = 'dfu_cc_pb2'
      # @@protoc_insertion_point(class_scope:dfu.InitCommand)
      ))
    _sym_db.RegisterMessage(InitCommand)

    ResetCommand = _reflection.GeneratedProtocolMessageType('ResetCommand', (_message.Message,), dict(
      DESCRIPTOR = _RESETCOMMAND,
      __module__ = 'dfu_cc_pb2'
      # @@protoc_insertion_point(class_scope:dfu.ResetCommand)
      ))
    _sym_db.RegisterMessage(ResetCommand)

    Command = _reflection.GeneratedProtocolMessageType('Command', (_message.Message,), dict(
      DESCRIPTOR = _COMMAND,
      __module__ = 'dfu_cc_pb2'
      # @@protoc_insertion_point(class_scope:dfu.Command)
      ))
    _sym_db.RegisterMessage(Command)

    SignedCommand = _reflection.GeneratedProtocolMessageType('SignedCommand', (_message.Message,), dict(
      DESCRIPTOR = _SIGNEDCOMMAND,
      __module__ = 'dfu_cc_pb2'
      # @@protoc_insertion_point(class_scope:dfu.SignedCommand)
      ))
    _sym_db.RegisterMessage(SignedCommand)

    Packet = _reflection.GeneratedProtocolMessageType('Packet', (_message.Message,), dict(
      DESCRIPTOR = _PACKET,
      __module__ = 'dfu_cc_pb2'
      # @@protoc_insertion_point(class_scope:dfu.Packet)
      ))
    _sym_db.RegisterMessage(Packet)


    _INITCOMMAND.fields_by_name['sd_req'].has_options = True
    _INITCOMMAND.fields_by_name['sd_req']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\020\001'))


################################################################################
#from from nordicsemi.dfu.init_packet_pb import *
#ref init_packet_pb.py
################################################################################
class SigningTypes(Enum):
    ECDSA_P256_SHA256 = pb.ECDSA_P256_SHA256
    ED25519 = pb.ED25519

class CommandTypes(Enum):
    RESET = pb.RESET
    INIT = pb.INIT

class HashTypes(Enum):
    NONE = pb.NO_HASH
    CRC = pb.CRC
    SHA128 = pb.SHA128
    SHA256 = pb.SHA256
    SHA512 = pb.SHA512


class DFUType(Enum):
    APPLICATION = pb.APPLICATION
    SOFTDEVICE = pb.SOFTDEVICE
    BOOTLOADER = pb.BOOTLOADER
    SOFTDEVICE_BOOTLOADER = pb.SOFTDEVICE_BOOTLOADER


class InitPacketPB(object):
    def __init__(self,
                 from_bytes = None,
                 hash_bytes = None,
                 hash_type = None,
                 dfu_type = None,
                 is_debug=False,
                 fw_version=0xffffffff,
                 hw_version=0xffffffff,
                 sd_size=0,
                 app_size=0,
                 bl_size=0,
                 sd_req=None
                 ):

        if from_bytes is not None:
            # construct from a protobuf string/buffer
            self.packet = pb.Packet()
            self.packet.ParseFromString(from_bytes)

            if self.packet.HasField('signed_command'):
                self.init_command = self.packet.signed_command.command.init
            else:
                self.init_command = self.packet.command.init

        else:
            # construct from input variables
            if not sd_req:
                sd_req = [0xfffe]  # Set to default value
            self.packet = pb.Packet()

            # By default, set the packet's command to an unsigned command
            # If a signature is set (via set_signature), this will get overwritten
            # with an instance of SignedCommand instead.
            self.packet.command.op_code = pb.INIT

            self.init_command = pb.InitCommand()
            self.init_command.hash.hash_type = hash_type.value
            self.init_command.type = dfu_type.value
            self.init_command.hash.hash = hash_bytes
            self.init_command.is_debug = is_debug
            self.init_command.fw_version = fw_version
            self.init_command.hw_version = hw_version
            self.init_command.sd_req.extend(list(set(sd_req)))
            self.init_command.sd_size = sd_size
            self.init_command.bl_size = bl_size
            self.init_command.app_size = app_size

            self.packet.command.init.CopyFrom(self.init_command)

        self._validate()

    def _validate(self):
        if self.init_command.type == pb.APPLICATION and self.init_command.app_size == 0:
            raise RuntimeError("app_size is not set. It must be set when type is APPLICATION")
        elif self.init_command.type == pb.SOFTDEVICE and self.init_command.sd_size == 0:
            raise RuntimeError("sd_size is not set. It must be set when type is SOFTDEVICE")
        elif self.init_command.type == pb.BOOTLOADER and self.init_command.bl_size == 0:
            raise RuntimeError("bl_size is not set. It must be set when type is BOOTLOADER")
        elif self.init_command.type == pb.SOFTDEVICE_BOOTLOADER and \
                (self.init_command.sd_size == 0 or self.init_command.bl_size == 0):
            raise RuntimeError("Either sd_size or bl_size is not set. Both must be set when type "
                               "is SOFTDEVICE_BOOTLOADER")

        if self.init_command.fw_version < 0 or self.init_command.fw_version > 0xffffffff or \
           self.init_command.hw_version < 0 or self.init_command.hw_version > 0xffffffff:
            raise RuntimeError("Invalid range of firmware argument. [0 - 0xffffffff] is valid range")

    def _is_valid(self):
        try:
            self._validate()
        except RuntimeError:
            return False

        return self.signed_command.signature is not None

    def get_init_packet_pb_bytes(self):
        return self.packet.SerializeToString()

    def get_init_command_bytes(self):
        return self.init_command.SerializeToString()

    def set_signature(self, signature, signature_type):
        new_packet = pb.Packet()
        new_packet.signed_command.signature = signature
        new_packet.signed_command.signature_type = signature_type.value
        new_packet.signed_command.command.CopyFrom(self.packet.command)

        self.packet = new_packet

    def __str__(self):
        return str(self.init_command)

################################################################################
#from nordicsemi.dfu.nrfhex import *
#ref nordicsemi/dfu/intelhex/__init__.py
################################################################################
class _DeprecatedParam(object):
    pass

_DEPRECATED = _DeprecatedParam()


class IntelHex(object):
    ''' Intel HEX file reader. '''

    def __init__(self, source=None):
        ''' Constructor. If source specified, object will be initialized
        with the contents of source. Otherwise the object will be empty.

        @param  source      source for initialization
                            (file name of HEX file, file object, addr dict or
                             other IntelHex object)
        '''
        # public members
        self.padding = 0x0FF
        # Start Address
        self.start_addr = None

        # private members
        self._buf = {}
        self._offset = 0

        if source is not None:
            if isinstance(source, basestring) or getattr(source, "read", None):
                # load hex file
                self.loadhex(source)
            elif isinstance(source, dict):
                self.fromdict(source)
            elif isinstance(source, IntelHex):
                self.padding = source.padding
                if source.start_addr:
                    self.start_addr = source.start_addr.copy()
                self._buf = source._buf.copy()
            else:
                raise ValueError("source: bad initializer type")

    def _decode_record(self, s, line=0):
        '''Decode one record of HEX file.

        @param  s       line with HEX record.
        @param  line    line number (for error messages).

        @raise  EndOfFile   if EOF record encountered.
        '''
        s = s.rstrip('\r\n')
        if not s:
            return          # empty line

        if s[0] == ':':
            try:
                bin = array('B', unhexlify(asbytes(s[1:])))
            except (TypeError, ValueError):
                # this might be raised by unhexlify when odd hexascii digits
                raise HexRecordError(line=line)
            length = len(bin)
            if length < 5:
                raise HexRecordError(line=line)
        else:
            raise HexRecordError(line=line)

        record_length = bin[0]
        if length != (5 + record_length):
            raise RecordLengthError(line=line)

        addr = bin[1]*256 + bin[2]

        record_type = bin[3]
        if not (0 <= record_type <= 5):
            raise RecordTypeError(line=line)

        crc = sum(bin)
        crc &= 0x0FF
        if crc != 0:
            raise RecordChecksumError(line=line)

        if record_type == 0:
            # data record
            addr += self._offset
            for i in xrange(4, 4+record_length):
                if not self._buf.get(addr, None) is None:
                    raise AddressOverlapError(address=addr, line=line)
                self._buf[addr] = bin[i]
                addr += 1   # FIXME: addr should be wrapped 
                            # BUT after 02 record (at 64K boundary)
                            # and after 04 record (at 4G boundary)

        elif record_type == 1:
            # end of file record
            if record_length != 0:
                raise EOFRecordError(line=line)
            raise _EndOfFile

        elif record_type == 2:
            # Extended 8086 Segment Record
            if record_length != 2 or addr != 0:
                raise ExtendedSegmentAddressRecordError(line=line)
            self._offset = (bin[4]*256 + bin[5]) * 16

        elif record_type == 4:
            # Extended Linear Address Record
            if record_length != 2 or addr != 0:
                raise ExtendedLinearAddressRecordError(line=line)
            self._offset = (bin[4]*256 + bin[5]) * 65536

        elif record_type == 3:
            # Start Segment Address Record
            if record_length != 4 or addr != 0:
                raise StartSegmentAddressRecordError(line=line)
            if self.start_addr:
                raise DuplicateStartAddressRecordError(line=line)
            self.start_addr = {'CS': bin[4]*256 + bin[5],
                               'IP': bin[6]*256 + bin[7],
                              }

        elif record_type == 5:
            # Start Linear Address Record
            if record_length != 4 or addr != 0:
                raise StartLinearAddressRecordError(line=line)
            if self.start_addr:
                raise DuplicateStartAddressRecordError(line=line)
            self.start_addr = {'EIP': (bin[4]*16777216 +
                                       bin[5]*65536 +
                                       bin[6]*256 +
                                       bin[7]),
                              }

    def loadhex(self, fobj):
        """Load hex file into internal buffer. This is not necessary
        if object was initialized with source set. This will overwrite
        addresses if object was already initialized.

        @param  fobj        file name or file-like object
        """
        if getattr(fobj, "read", None) is None:
            fobj = open(fobj, "r")
            fclose = fobj.close
        else:
            fclose = None

        self._offset = 0
        line = 0

        try:
            decode = self._decode_record
            try:
                for s in fobj:
                    line += 1
                    decode(s, line)
            except _EndOfFile:
                pass
        finally:
            if fclose:
                fclose()

    def loadbin(self, fobj, offset=0):
        """Load bin file into internal buffer. Not needed if source set in
        constructor. This will overwrite addresses without warning
        if object was already initialized.

        @param  fobj        file name or file-like object
        @param  offset      starting address offset
        """
        fread = getattr(fobj, "read", None)
        if fread is None:
            f = open(fobj, "rb")
            fread = f.read
            fclose = f.close
        else:
            fclose = None

        try:
            self.frombytes(array('B', asbytes(fread())), offset=offset)
        finally:
            if fclose:
                fclose()

    def loadfile(self, fobj, format):
        """Load data file into internal buffer. Preferred wrapper over
        loadbin or loadhex.

        @param  fobj        file name or file-like object
        @param  format      file format ("hex" or "bin")
        """
        if format == "hex":
            self.loadhex(fobj)
        elif format == "bin":
            self.loadbin(fobj)
        else:
            raise ValueError('format should be either "hex" or "bin";'
                ' got %r instead' % format)

    # alias (to be consistent with method tofile)
    fromfile = loadfile

    def fromdict(self, dikt):
        """Load data from dictionary. Dictionary should contain int keys
        representing addresses. Values should be the data to be stored in
        those addresses in unsigned char form (i.e. not strings).
        The dictionary may contain the key, ``start_addr``
        to indicate the starting address of the data as described in README.

        The contents of the dict will be merged with this object and will
        overwrite any conflicts. This function is not necessary if the
        object was initialized with source specified.
        """
        s = dikt.copy()
        start_addr = s.get('start_addr')
        if start_addr is not None:
            del s['start_addr']
        for k in s.keys():
            if type(k) not in (int, long) or k < 0:
                raise ValueError('Source dictionary should have only int keys')
        self._buf.update(s)
        if start_addr is not None:
            self.start_addr = start_addr

    def frombytes(self, bytes, offset=0):
        """Load data from array or list of bytes.
        Similar to loadbin() method but works directly with iterable bytes.
        """
        for b in bytes:
            self._buf[offset] = b
            offset += 1

    def _get_start_end(self, start=None, end=None, size=None):
        """Return default values for start and end if they are None.
        If this IntelHex object is empty then it's error to
        invoke this method with both start and end as None. 
        """
        if (start,end) == (None,None) and self._buf == {}:
            raise EmptyIntelHexError
        if size is not None:
            if None not in (start, end):
                raise ValueError("tobinarray: you can't use start,end and size"
                                 " arguments in the same time")
            if (start, end) == (None, None):
                start = self.minaddr()
            if start is not None:
                end = start + size - 1
            else:
                start = end - size + 1
                if start < 0:
                    raise ValueError("tobinarray: invalid size (%d) "
                                     "for given end address (%d)" % (size,end))
        else:
            if start is None:
                start = self.minaddr()
            if end is None:
                end = self.maxaddr()
            if start > end:
                start, end = end, start
        return start, end

    def tobinarray(self, start=None, end=None, pad=_DEPRECATED, size=None):
        ''' Convert this object to binary form as array. If start and end 
        unspecified, they will be inferred from the data.
        @param  start   start address of output bytes.
        @param  end     end address of output bytes (inclusive).
        @param  pad     [DEPRECATED PARAMETER, please use self.padding instead]
                        fill empty spaces with this value
                        (if pad is None then this method uses self.padding).
        @param  size    size of the block, used with start or end parameter.
        @return         array of unsigned char data.
        '''
        if not isinstance(pad, _DeprecatedParam):
            print("IntelHex.tobinarray: 'pad' parameter is deprecated.")
            if pad is not None:
                print("Please, use IntelHex.padding attribute instead.")
            else:
                print("Please, don't pass it explicitly.")
                print("Use syntax like this: ih.tobinarray(start=xxx, end=yyy, size=zzz)")
        else:
            pad = None
        return self._tobinarray_really(start, end, pad, size)

    def _tobinarray_really(self, start, end, pad, size):
        if pad is None:
            pad = self.padding

        bin = array('B')

        if self._buf == {} and None in (start, end):
            return bin

        if size is not None and size <= 0:
            raise ValueError("tobinarray: wrong value for size")

        start, end = self._get_start_end(start, end, size)

        for i in xrange(start, end+1):
            bin.append(self._buf.get(i, pad))

        return bin

    def tobinstr(self, start=None, end=None, pad=_DEPRECATED, size=None):
        ''' Convert to binary form and return as a string.
        @param  start   start address of output bytes.
        @param  end     end address of output bytes (inclusive).
        @param  pad     [DEPRECATED PARAMETER, please use self.padding instead]
                        fill empty spaces with this value
                        (if pad is None then this method uses self.padding).
        @param  size    size of the block, used with start or end parameter.
        @return         string of binary data.
        '''
        if not isinstance(pad, _DeprecatedParam):
            print("IntelHex.tobinstr: 'pad' parameter is deprecated.")
            if pad is not None:
                print("Please, use IntelHex.padding attribute instead.")
            else:
                print("Please, don't pass it explicitly.")
                print("Use syntax like this: ih.tobinstr(start=xxx, end=yyy, size=zzz)")
        else:
            pad = None
        return self._tobinstr_really(start, end, pad, size)

    def _tobinstr_really(self, start, end, pad, size):
        return asstr(self._tobinarray_really(start, end, pad, size).tostring())

    def tobinfile(self, fobj, start=None, end=None, pad=_DEPRECATED, size=None):
        '''Convert to binary and write to file.

        @param  fobj    file name or file object for writing output bytes.
        @param  start   start address of output bytes.
        @param  end     end address of output bytes (inclusive).
        @param  pad     [DEPRECATED PARAMETER, please use self.padding instead]
                        fill empty spaces with this value
                        (if pad is None then this method uses self.padding).
        @param  size    size of the block, used with start or end parameter.
        '''
        if not isinstance(pad, _DeprecatedParam):
            print("IntelHex.tobinfile: 'pad' parameter is deprecated.")
            if pad is not None:
                print("Please, use IntelHex.padding attribute instead.")
            else:
                print("Please, don't pass it explicitly.")
                print("Use syntax like this: ih.tobinfile(start=xxx, end=yyy, size=zzz)")
        else:
            pad = None
        if getattr(fobj, "write", None) is None:
            fobj = open(fobj, "wb")
            close_fd = True
        else:
            close_fd = False

        fobj.write(self._tobinstr_really(start, end, pad, size))

        if close_fd:
            fobj.close()

    def todict(self):
        '''Convert to python dictionary.

        @return         dict suitable for initializing another IntelHex object.
        '''
        r = {}
        r.update(self._buf)
        if self.start_addr:
            r['start_addr'] = self.start_addr
        return r

    def addresses(self):
        '''Returns all used addresses in sorted order.
        @return         list of occupied data addresses in sorted order. 
        '''
        aa = self._buf.keys()
        aa.sort()
        return aa

    def minaddr(self):
        '''Get minimal address of HEX content.
        @return         minimal address or None if no data
        '''
        aa = self._buf.keys()
        if aa == []:
            return None
        else:
            return min(aa)

    def maxaddr(self):
        '''Get maximal address of HEX content.
        @return         maximal address or None if no data
        '''
        aa = self._buf.keys()
        if aa == []:
            return None
        else:
            return max(aa)

    def __getitem__(self, addr):
        ''' Get requested byte from address.
        @param  addr    address of byte.
        @return         byte if address exists in HEX file, or self.padding
                        if no data found.
        '''
        t = type(addr)
        if t in (int, long):
            if addr < 0:
                raise TypeError('Address should be >= 0.')
            return self._buf.get(addr, self.padding)
        elif t == slice:
            addresses = self._buf.keys()
            ih = IntelHex()
            if addresses:
                addresses.sort()
                start = addr.start or addresses[0]
                stop = addr.stop or (addresses[-1]+1)
                step = addr.step or 1
                for i in xrange(start, stop, step):
                    x = self._buf.get(i)
                    if x is not None:
                        ih[i] = x
            return ih
        else:
            raise TypeError('Address has unsupported type: %s' % t)

    def __setitem__(self, addr, byte):
        """Set byte at address."""
        t = type(addr)
        if t in (int, long):
            if addr < 0:
                raise TypeError('Address should be >= 0.')
            self._buf[addr] = byte
        elif t == slice:
            if not isinstance(byte, (list, tuple)):
                raise ValueError('Slice operation expects sequence of bytes')
            start = addr.start
            stop = addr.stop
            step = addr.step or 1
            if None not in (start, stop):
                ra = range(start, stop, step)
                if len(ra) != len(byte):
                    raise ValueError('Length of bytes sequence does not match '
                        'address range')
            elif (start, stop) == (None, None):
                raise TypeError('Unsupported address range')
            elif start is None:
                start = stop - len(byte)
            elif stop is None:
                stop = start + len(byte)
            if start < 0:
                raise TypeError('start address cannot be negative')
            if stop < 0:
                raise TypeError('stop address cannot be negative')
            j = 0
            for i in xrange(start, stop, step):
                self._buf[i] = byte[j]
                j += 1
        else:
            raise TypeError('Address has unsupported type: %s' % t)

    def __delitem__(self, addr):
        """Delete byte at address."""
        t = type(addr)
        if t in (int, long):
            if addr < 0:
                raise TypeError('Address should be >= 0.')
            del self._buf[addr]
        elif t == slice:
            addresses = self._buf.keys()
            if addresses:
                addresses.sort()
                start = addr.start or addresses[0]
                stop = addr.stop or (addresses[-1]+1)
                step = addr.step or 1
                for i in xrange(start, stop, step):
                    x = self._buf.get(i)
                    if x is not None:
                        del self._buf[i]
        else:
            raise TypeError('Address has unsupported type: %s' % t)

    def __len__(self):
        """Return count of bytes with real values."""
        return len(self._buf.keys())

    def write_hex_file(self, f, write_start_addr=True):
        """Write data to file f in HEX format.

        @param  f                   filename or file-like object for writing
        @param  write_start_addr    enable or disable writing start address
                                    record to file (enabled by default).
                                    If there is no start address in obj, nothing
                                    will be written regardless of this setting.
        """
        fwrite = getattr(f, "write", None)
        if fwrite:
            fobj = f
            fclose = None
        else:
            fobj = open(f, 'w')
            fwrite = fobj.write
            fclose = fobj.close

        # Translation table for uppercasing hex ascii string.
        # timeit shows that using hexstr.translate(table)
        # is faster than hexstr.upper():
        # 0.452ms vs. 0.652ms (translate vs. upper)
        if sys.version_info[0] >= 3:
            table = bytes(range(256)).upper()
        else:
            table = ''.join(chr(i).upper() for  i in range(256))



        # start address record if any
        if self.start_addr and write_start_addr:
            keys = self.start_addr.keys()
            keys.sort()
            bin = array('B', asbytes('\0'*9))
            if keys == ['CS','IP']:
                # Start Segment Address Record
                bin[0] = 4      # reclen
                bin[1] = 0      # offset msb
                bin[2] = 0      # offset lsb
                bin[3] = 3      # rectyp
                cs = self.start_addr['CS']
                bin[4] = (cs >> 8) & 0x0FF
                bin[5] = cs & 0x0FF
                ip = self.start_addr['IP']
                bin[6] = (ip >> 8) & 0x0FF
                bin[7] = ip & 0x0FF
                bin[8] = (-sum(bin)) & 0x0FF    # chksum
                fwrite(':' +
                       asstr(hexlify(bin.tostring()).translate(table)) +
                       '\n')
            elif keys == ['EIP']:
                # Start Linear Address Record
                bin[0] = 4      # reclen
                bin[1] = 0      # offset msb
                bin[2] = 0      # offset lsb
                bin[3] = 5      # rectyp
                eip = self.start_addr['EIP']
                bin[4] = (eip >> 24) & 0x0FF
                bin[5] = (eip >> 16) & 0x0FF
                bin[6] = (eip >> 8) & 0x0FF
                bin[7] = eip & 0x0FF
                bin[8] = (-sum(bin)) & 0x0FF    # chksum
                fwrite(':' +
                       asstr(hexlify(bin.tostring()).translate(table)) +
                       '\n')
            else:
                if fclose:
                    fclose()
                raise InvalidStartAddressValueError(start_addr=self.start_addr)

        # data
        addresses = self._buf.keys()
        addresses.sort()
        addr_len = len(addresses)
        if addr_len:
            minaddr = addresses[0]
            maxaddr = addresses[-1]

            if maxaddr > 65535:
                need_offset_record = True
            else:
                need_offset_record = False
            high_ofs = 0

            cur_addr = minaddr
            cur_ix = 0

            while cur_addr <= maxaddr:
                if need_offset_record:
                    bin = array('B', asbytes('\0'*7))
                    bin[0] = 2      # reclen
                    bin[1] = 0      # offset msb
                    bin[2] = 0      # offset lsb
                    bin[3] = 4      # rectyp
                    high_ofs = int(cur_addr>>16)
                    b = divmod(high_ofs, 256)
                    bin[4] = b[0]   # msb of high_ofs
                    bin[5] = b[1]   # lsb of high_ofs
                    bin[6] = (-sum(bin)) & 0x0FF    # chksum
                    fwrite(':' +
                           asstr(hexlify(bin.tostring()).translate(table)) +
                           '\n')

                while True:
                    # produce one record
                    low_addr = cur_addr & 0x0FFFF
                    # chain_len off by 1
                    chain_len = min(15, 65535-low_addr, maxaddr-cur_addr)

                    # search continuous chain
                    stop_addr = cur_addr + chain_len
                    if chain_len:
                        ix = bisect_right(addresses, stop_addr,
                                          cur_ix,
                                          min(cur_ix+chain_len+1, addr_len))
                        chain_len = ix - cur_ix     # real chain_len
                        # there could be small holes in the chain
                        # but we will catch them by try-except later
                        # so for big continuous files we will work
                        # at maximum possible speed
                    else:
                        chain_len = 1               # real chain_len

                    bin = array('B', asbytes('\0'*(5+chain_len)))
                    b = divmod(low_addr, 256)
                    bin[1] = b[0]   # msb of low_addr
                    bin[2] = b[1]   # lsb of low_addr
                    bin[3] = 0          # rectype
                    try:    # if there is small holes we'll catch them
                        for i in range(chain_len):
                            bin[4+i] = self._buf[cur_addr+i]
                    except KeyError:
                        # we catch a hole so we should shrink the chain
                        chain_len = i
                        bin = bin[:5+i]
                    bin[0] = chain_len
                    bin[4+chain_len] = (-sum(bin)) & 0x0FF    # chksum
                    fwrite(':' +
                           asstr(hexlify(bin.tostring()).translate(table)) +
                           '\n')

                    # adjust cur_addr/cur_ix
                    cur_ix += chain_len
                    if cur_ix < addr_len:
                        cur_addr = addresses[cur_ix]
                    else:
                        cur_addr = maxaddr + 1
                        break
                    high_addr = int(cur_addr>>16)
                    if high_addr > high_ofs:
                        break

        # end-of-file record
        fwrite(":00000001FF\n")
        if fclose:
            fclose()

    def tofile(self, fobj, format):
        """Write data to hex or bin file. Preferred method over tobin or tohex.

        @param  fobj        file name or file-like object
        @param  format      file format ("hex" or "bin")
        """
        if format == 'hex':
            self.write_hex_file(fobj)
        elif format == 'bin':
            self.tobinfile(fobj)
        else:
            raise ValueError('format should be either "hex" or "bin";'
                ' got %r instead' % format)

    def gets(self, addr, length):
        """Get string of bytes from given address. If any entries are blank
        from addr through addr+length, a NotEnoughDataError exception will
        be raised. Padding is not used."""
        a = array('B', asbytes('\0'*length))
        try:
            for i in xrange(length):
                a[i] = self._buf[addr+i]
        except KeyError:
            raise NotEnoughDataError(address=addr, length=length)
        return asstr(a.tostring())

    def puts(self, addr, s):
        """Put string of bytes at given address. Will overwrite any previous
        entries.
        """
        a = array('B', asbytes(s))
        for i in xrange(len(a)):
            self._buf[addr+i] = a[i]

    def getsz(self, addr):
        """Get zero-terminated string from given address. Will raise 
        NotEnoughDataError exception if a hole is encountered before a 0.
        """
        i = 0
        try:
            while True:
                if self._buf[addr+i] == 0:
                    break
                i += 1
        except KeyError:
            raise NotEnoughDataError(msg=('Bad access at 0x%X: '
                'not enough data to read zero-terminated string') % addr)
        return self.gets(addr, i)

    def putsz(self, addr, s):
        """Put string in object at addr and append terminating zero at end."""
        self.puts(addr, s)
        self._buf[addr+len(s)] = 0

    def dump(self, tofile=None):
        """Dump object content to specified file object or to stdout if None.
        Format is a hexdump with some header information at the beginning,
        addresses on the left, and data on right.

        @param  tofile        file-like object to dump to
        """

        if tofile is None:
            tofile = sys.stdout
        # start addr possibly
        if self.start_addr is not None:
            cs = self.start_addr.get('CS')
            ip = self.start_addr.get('IP')
            eip = self.start_addr.get('EIP')
            if eip is not None and cs is None and ip is None:
                tofile.write('EIP = 0x%08X\n' % eip)
            elif eip is None and cs is not None and ip is not None:
                tofile.write('CS = 0x%04X, IP = 0x%04X\n' % (cs, ip))
            else:
                tofile.write('start_addr = %r\n' % start_addr)
        # actual data
        addresses = self._buf.keys()
        if addresses:
            addresses.sort()
            minaddr = addresses[0]
            maxaddr = addresses[-1]
            startaddr = int(minaddr>>4)*16
            endaddr = int((maxaddr>>4)+1)*16
            maxdigits = max(len(str(endaddr)), 4)
            templa = '%%0%dX' % maxdigits
            range16 = range(16)
            for i in xrange(startaddr, endaddr, 16):
                tofile.write(templa % i)
                tofile.write(' ')
                s = []
                for j in range16:
                    x = self._buf.get(i+j)
                    if x is not None:
                        tofile.write(' %02X' % x)
                        if 32 <= x < 127:   # GNU less does not like 0x7F (128 decimal) so we'd better show it as dot
                            s.append(chr(x))
                        else:
                            s.append('.')
                    else:
                        tofile.write(' --')
                        s.append(' ')
                tofile.write('  |' + ''.join(s) + '|\n')

    def merge(self, other, overlap='error'):
        """Merge content of other IntelHex object into current object (self).
        @param  other   other IntelHex object.
        @param  overlap action on overlap of data or starting addr:
                        - error: raising OverlapError;
                        - ignore: ignore other data and keep current data
                                  in overlapping region;
                        - replace: replace data with other data
                                  in overlapping region.

        @raise  TypeError       if other is not instance of IntelHex
        @raise  ValueError      if other is the same object as self 
                                (it can't merge itself)
        @raise  ValueError      if overlap argument has incorrect value
        @raise  AddressOverlapError    on overlapped data
        """
        # check args
        if not isinstance(other, IntelHex):
            raise TypeError('other should be IntelHex object')
        if other is self:
            raise ValueError("Can't merge itself")
        if overlap not in ('error', 'ignore', 'replace'):
            raise ValueError("overlap argument should be either "
                "'error', 'ignore' or 'replace'")
        # merge data
        this_buf = self._buf
        other_buf = other._buf
        for i in other_buf:
            if i in this_buf:
                if overlap == 'error':
                    raise AddressOverlapError(
                        'Data overlapped at address 0x%X' % i)
                elif overlap == 'ignore':
                    continue
            this_buf[i] = other_buf[i]
        # merge start_addr
        if self.start_addr != other.start_addr:
            if self.start_addr is None:     # set start addr from other
                self.start_addr = other.start_addr
            elif other.start_addr is None:  # keep existing start addr
                pass
            else:                           # conflict
                if overlap == 'error':
                    raise AddressOverlapError(
                        'Starting addresses are different')
                elif overlap == 'replace':
                    self.start_addr = other.start_addr
#/IntelHex


class IntelHex16bit(IntelHex):
    """Access to data as 16-bit words. Intended to use with Microchip HEX files."""

    def __init__(self, source=None):
        """Construct class from HEX file
        or from instance of ordinary IntelHex class. If IntelHex object
        is passed as source, the original IntelHex object should not be used
        again because this class will alter it. This class leaves padding
        alone unless it was precisely 0xFF. In that instance it is sign
        extended to 0xFFFF.

        @param  source  file name of HEX file or file object
                        or instance of ordinary IntelHex class.
                        Will also accept dictionary from todict method.
        """
        if isinstance(source, IntelHex):
            # from ihex8
            self.padding = source.padding
            self.start_addr = source.start_addr
            # private members
            self._buf = source._buf
            self._offset = source._offset
        elif isinstance(source, dict):
            raise IntelHexError("IntelHex16bit does not support initialization from dictionary yet.\n"
                                "Patches are welcome.")
        else:
            IntelHex.__init__(self, source)

        if self.padding == 0x0FF:
            self.padding = 0x0FFFF

    def __getitem__(self, addr16):
        """Get 16-bit word from address.
        Raise error if only one byte from the pair is set.
        We assume a Little Endian interpretation of the hex file.

        @param  addr16  address of word (addr8 = 2 * addr16).
        @return         word if bytes exists in HEX file, or self.padding
                        if no data found.
        """
        addr1 = addr16 * 2
        addr2 = addr1 + 1
        byte1 = self._buf.get(addr1, None)
        byte2 = self._buf.get(addr2, None)

        if byte1 != None and byte2 != None:
            return byte1 | (byte2 << 8)     # low endian

        if byte1 == None and byte2 == None:
            return self.padding

        raise BadAccess16bit(address=addr16)

    def __setitem__(self, addr16, word):
        """Sets the address at addr16 to word assuming Little Endian mode.
        """
        addr_byte = addr16 * 2
        b = divmod(word, 256)
        self._buf[addr_byte] = b[1]
        self._buf[addr_byte+1] = b[0]

    def minaddr(self):
        '''Get minimal address of HEX content in 16-bit mode.

        @return         minimal address used in this object
        '''
        aa = self._buf.keys()
        if aa == []:
            return 0
        else:
            return min(aa)>>1

    def maxaddr(self):
        '''Get maximal address of HEX content in 16-bit mode.

        @return         maximal address used in this object 
        '''
        aa = self._buf.keys()
        if aa == []:
            return 0
        else:
            return max(aa)>>1

    def tobinarray(self, start=None, end=None, size=None):
        '''Convert this object to binary form as array (of 2-bytes word data).
        If start and end unspecified, they will be inferred from the data.
        @param  start   start address of output data.
        @param  end     end address of output data (inclusive).
        @param  size    size of the block (number of words),
                        used with start or end parameter.
        @return         array of unsigned short (uint16_t) data.
        '''
        bin = array('H')

        if self._buf == {} and None in (start, end):
            return bin

        if size is not None and size <= 0:
            raise ValueError("tobinarray: wrong value for size")

        start, end = self._get_start_end(start, end, size)

        for addr in xrange(start, end+1):
            bin.append(self[addr])

        return bin


#/class IntelHex16bit


def hex2bin(fin, fout, start=None, end=None, size=None, pad=None):
    """Hex-to-Bin convertor engine.
    @return     0   if all OK

    @param  fin     input hex file (filename or file-like object)
    @param  fout    output bin file (filename or file-like object)
    @param  start   start of address range (optional)
    @param  end     end of address range (inclusive; optional)
    @param  size    size of resulting file (in bytes) (optional)
    @param  pad     padding byte (optional)
    """
    try:
        h = IntelHex(fin)
    except HexReaderError as e:
        txt = "ERROR: bad HEX file: %s" % str(e)
        print(txt)
        return 1

    # start, end, size
    if size != None and size != 0:
        if end == None:
            if start == None:
                start = h.minaddr()
            end = start + size - 1
        else:
            if (end+1) >= size:
                start = end + 1 - size
            else:
                start = 0

    try:
        if pad is not None:
            # using .padding attribute rather than pad argument to function call
            h.padding = pad
        h.tobinfile(fout, start, end)
    except IOError as e:
        txt = "ERROR: Could not write to file: %s: %s" % (fout, str(e))
        print(txt)
        return 1

    return 0
#/def hex2bin


def bin2hex(fin, fout, offset=0):
    """Simple bin-to-hex convertor.
    @return     0   if all OK

    @param  fin     input bin file (filename or file-like object)
    @param  fout    output hex file (filename or file-like object)
    @param  offset  starting address offset for loading bin
    """
    h = IntelHex()
    try:
        h.loadbin(fin, offset)
    except IOError as e:
        txt = 'ERROR: unable to load bin file:', str(e)
        print(txt)
        return 1

    try:
        h.tofile(fout, format='hex')
    except IOError as e:
        txt = "ERROR: Could not write to file: %s: %s" % (fout, str(e))
        print(txt)
        return 1

    return 0
#/def bin2hex


def diff_dumps(ih1, ih2, tofile=None, name1="a", name2="b", n_context=3):
    """Diff 2 IntelHex objects and produce unified diff output for their
    hex dumps.

    @param ih1        first IntelHex object to compare
    @param ih2        second IntelHex object to compare
    @param tofile     file-like object to write output
    @param name1      name of the first hex file to show in the diff header
    @param name2      name of the first hex file to show in the diff header
    @param n_context  number of context lines in the unidiff output
    """
    def prepare_lines(ih):
        from cStringIO import StringIO
        sio = StringIO()
        ih.dump(sio)
        dump = sio.getvalue()
        lines = dump.splitlines()
        return lines
    a = prepare_lines(ih1)
    b = prepare_lines(ih2)
    import difflib
    result = list(difflib.unified_diff(a, b, fromfile=name1, tofile=name2, n=n_context, lineterm=''))
    if tofile is None:
        tofile = sys.stdout
    output = '\n'.join(result)+'\n'
    tofile.write(output)


class Record(object):
    """Helper methods to build valid ihex records."""

    def _from_bytes(bytes):
        """Takes a list of bytes, computes the checksum, and outputs the entire
        record as a string. bytes should be the hex record without the colon
        or final checksum.

        @param  bytes   list of byte values so far to pack into record.
        @return         String representation of one HEX record
        """
        assert len(bytes) >= 4
        # calculate checksum
        s = (-sum(bytes)) & 0x0FF
        bin = array('B', bytes + [s])
        return ':' + asstr(hexlify(bin.tostring())).upper()
    _from_bytes = staticmethod(_from_bytes)

    def data(offset, bytes):
        """Return Data record. This constructs the full record, including
        the length information, the record type (0x00), the
        checksum, and the offset.

        @param  offset  load offset of first byte.
        @param  bytes   list of byte values to pack into record.

        @return         String representation of one HEX record
        """
        assert 0 <= offset < 65536
        assert 0 < len(bytes) < 256
        b = [len(bytes), (offset>>8)&0x0FF, offset&0x0FF, 0x00] + bytes
        return Record._from_bytes(b)
    data = staticmethod(data)

    def eof():
        """Return End of File record as a string.
        @return         String representation of Intel Hex EOF record 
        """
        return ':00000001FF'
    eof = staticmethod(eof)

    def extended_segment_address(usba):
        """Return Extended Segment Address Record.
        @param  usba     Upper Segment Base Address.

        @return         String representation of Intel Hex USBA record.
        """
        b = [2, 0, 0, 0x02, (usba>>8)&0x0FF, usba&0x0FF]
        return Record._from_bytes(b)
    extended_segment_address = staticmethod(extended_segment_address)

    def start_segment_address(cs, ip):
        """Return Start Segment Address Record.
        @param  cs      16-bit value for CS register.
        @param  ip      16-bit value for IP register.

        @return         String representation of Intel Hex SSA record.
        """
        b = [4, 0, 0, 0x03, (cs>>8)&0x0FF, cs&0x0FF,
             (ip>>8)&0x0FF, ip&0x0FF]
        return Record._from_bytes(b)
    start_segment_address = staticmethod(start_segment_address)

    def extended_linear_address(ulba):
        """Return Extended Linear Address Record.
        @param  ulba    Upper Linear Base Address.

        @return         String representation of Intel Hex ELA record.
        """
        b = [2, 0, 0, 0x04, (ulba>>8)&0x0FF, ulba&0x0FF]
        return Record._from_bytes(b)
    extended_linear_address = staticmethod(extended_linear_address)

    def start_linear_address(eip):
        """Return Start Linear Address Record.
        @param  eip     32-bit linear address for the EIP register.

        @return         String representation of Intel Hex SLA record.
        """
        b = [4, 0, 0, 0x05, (eip>>24)&0x0FF, (eip>>16)&0x0FF,
             (eip>>8)&0x0FF, eip&0x0FF]
        return Record._from_bytes(b)
    start_linear_address = staticmethod(start_linear_address)


class _BadFileNotation(Exception):
    """Special error class to use with _get_file_and_addr_range."""
    pass

def _get_file_and_addr_range(s, _support_drive_letter=None):
    """Special method for hexmerge.py script to split file notation
    into 3 parts: (filename, start, end)

    @raise _BadFileNotation  when string cannot be safely split.
    """
    if _support_drive_letter is None:
        _support_drive_letter = (os.name == 'nt')
    drive = ''
    if _support_drive_letter:
        if s[1:2] == ':' and s[0].upper() in ''.join([chr(i) for i in range(ord('A'), ord('Z')+1)]):
            drive = s[:2]
            s = s[2:]
    parts = s.split(':')
    n = len(parts)
    if n == 1:
        fname = parts[0]
        fstart = None
        fend = None
    elif n != 3:
        raise _BadFileNotation
    else:
        fname = parts[0]
        def ascii_hex_to_int(ascii):
            if ascii is not None:
                try:
                    return int(ascii, 16)
                except ValueError:
                    raise _BadFileNotation
            return ascii
        fstart = ascii_hex_to_int(parts[1] or None)
        fend = ascii_hex_to_int(parts[2] or None)
    return drive+fname, fstart, fend


##
# IntelHex Errors Hierarchy:
#
#  IntelHexError    - basic error
#       HexReaderError  - general hex reader error
#           AddressOverlapError - data for the same address overlap
#           HexRecordError      - hex record decoder base error
#               RecordLengthError    - record has invalid length
#               RecordTypeError      - record has invalid type (RECTYP)
#               RecordChecksumError  - record checksum mismatch
#               EOFRecordError              - invalid EOF record (type 01)
#               ExtendedAddressRecordError  - extended address record base error
#                   ExtendedSegmentAddressRecordError   - invalid extended segment address record (type 02)
#                   ExtendedLinearAddressRecordError    - invalid extended linear address record (type 04)
#               StartAddressRecordError     - start address record base error
#                   StartSegmentAddressRecordError      - invalid start segment address record (type 03)
#                   StartLinearAddressRecordError       - invalid start linear address record (type 05)
#                   DuplicateStartAddressRecordError    - start address record appears twice
#                   InvalidStartAddressValueError       - invalid value of start addr record
#       _EndOfFile  - it's not real error, used internally by hex reader as signal that EOF record found
#       BadAccess16bit - not enough data to read 16 bit value (deprecated, see NotEnoughDataError)
#       NotEnoughDataError - not enough data to read N contiguous bytes
#       EmptyIntelHexError - requested operation cannot be performed with empty object

class IntelHexError(Exception):
    '''Base Exception class for IntelHex module'''

    _fmt = 'IntelHex base error'   #: format string

    def __init__(self, msg=None, **kw):
        """Initialize the Exception with the given message.
        """
        self.msg = msg
        for key, value in kw.items():
            setattr(self, key, value)

    def __str__(self):
        """Return the message in this Exception."""
        if self.msg:
            return self.msg
        try:
            return self._fmt % self.__dict__
        except (NameError, ValueError, KeyError) as e:
            return 'Unprintable exception %s: %s' \
                % (repr(e), str(e))

class _EndOfFile(IntelHexError):
    """Used for internal needs only."""
    _fmt = 'EOF record reached -- signal to stop read file'

class HexReaderError(IntelHexError):
    _fmt = 'Hex reader base error'

class AddressOverlapError(HexReaderError):
    _fmt = 'Hex file has data overlap at address 0x%(address)X on line %(line)d'

# class NotAHexFileError was removed in trunk.revno.54 because it's not used


class HexRecordError(HexReaderError):
    _fmt = 'Hex file contains invalid record at line %(line)d'


class RecordLengthError(HexRecordError):
    _fmt = 'Record at line %(line)d has invalid length'

class RecordTypeError(HexRecordError):
    _fmt = 'Record at line %(line)d has invalid record type'

class RecordChecksumError(HexRecordError):
    _fmt = 'Record at line %(line)d has invalid checksum'

class EOFRecordError(HexRecordError):
    _fmt = 'File has invalid End-of-File record'


class ExtendedAddressRecordError(HexRecordError):
    _fmt = 'Base class for extended address exceptions'

class ExtendedSegmentAddressRecordError(ExtendedAddressRecordError):
    _fmt = 'Invalid Extended Segment Address Record at line %(line)d'

class ExtendedLinearAddressRecordError(ExtendedAddressRecordError):
    _fmt = 'Invalid Extended Linear Address Record at line %(line)d'


class StartAddressRecordError(HexRecordError):
    _fmt = 'Base class for start address exceptions'

class StartSegmentAddressRecordError(StartAddressRecordError):
    _fmt = 'Invalid Start Segment Address Record at line %(line)d'

class StartLinearAddressRecordError(StartAddressRecordError):
    _fmt = 'Invalid Start Linear Address Record at line %(line)d'

class DuplicateStartAddressRecordError(StartAddressRecordError):
    _fmt = 'Start Address Record appears twice at line %(line)d'

class InvalidStartAddressValueError(StartAddressRecordError):
    _fmt = 'Invalid start address value: %(start_addr)s'


class NotEnoughDataError(IntelHexError):
    _fmt = ('Bad access at 0x%(address)X: '
            'not enough data to read %(length)d contiguous bytes')

class BadAccess16bit(NotEnoughDataError):
    _fmt = 'Bad access at 0x%(address)X: not enough data to read 16 bit value'

class EmptyIntelHexError(IntelHexError):
    _fmt = "Requested operation cannot be executed with empty object"



################################################################################
#from nordicsemi.dfu.nrfhex import *
#ref nrfhex.py
################################################################################
class nRFArch(Enum):
    NRF51 = 1
    NRF52 = 2
    NRF52840 = 3

class nRFHex(IntelHex):
    """
        Converts and merges .hex and .bin files into one .bin file.
    """

    info_struct_address_base = 0x00003000
    info_struct_address_offset = 0x1000

    info_struct_magic_number = 0x51B1E5DB
    info_struct_magic_number_offset = 0x004

    s1x0_mbr_end_address = 0x1000
    s132_mbr_end_address = 0x3000

    def __init__(self, source, bootloader=None, arch=None):
        """
        Constructor that requires a firmware file path.
        Softdevices can take an optional bootloader file path as parameter.

        :param str source: The file path for the firmware
        :param str bootloader: Optional file path to bootloader firmware
        :return: None
        """
        super(nRFHex, self).__init__()

        self.arch = arch
        self.file_format = 'hex'

        if source.endswith('.bin'):
            self.file_format = 'bin'

        self.loadfile(source, self.file_format)

        self._removeuicr()

        self.bootloaderhex = None

        if bootloader is not None:
            self.bootloaderhex = nRFHex(bootloader)

    def tohexfile(self, dest):
        pass

    def _removeuicr(self):
        uicr_start_address = 0x10000000
        maxaddress = self.maxaddr()
        if maxaddress >= uicr_start_address:
            for i in range(uicr_start_address, maxaddress + 1):
                self._buf.pop(i, 0)

    def address_has_magic_number(self, address):
        try:
            potential_magic_number = self.gets(address, 4)
            potential_magic_number = unpack('I', potential_magic_number)[0]
            return nRFHex.info_struct_magic_number == potential_magic_number
        except Exception:
            return False

    def get_softdevice_variant(self):
        potential_magic_number_address = nRFHex.info_struct_address_base + nRFHex.info_struct_magic_number_offset

        if self.address_has_magic_number(potential_magic_number_address):
            return "s1x0"

        for i in xrange(4):
            potential_magic_number_address += nRFHex.info_struct_address_offset

            if self.address_has_magic_number(potential_magic_number_address):
                return "s132"

        return "unknown"

    def get_mbr_end_address(self):
        softdevice_variant = self.get_softdevice_variant()

        if softdevice_variant == "s132":
            return nRFHex.s132_mbr_end_address
        else:
            return nRFHex.s1x0_mbr_end_address

    def minaddr(self):
        min_address = super(nRFHex, self).minaddr()

        # Lower addresses are reserved for master boot record
        if self.file_format != 'bin':
            min_address = max(self.get_mbr_end_address(), min_address)

        return min_address

    def size(self):
        """
        Returns the size of the source.
        :return: int
        """
        min_address = self.minaddr()
        max_address = self.maxaddr()

        size = max_address - min_address + 1

        # Round up to nearest word
        word_size = 4
        number_of_words = (size + (word_size - 1)) / word_size
        size = number_of_words * word_size

        return size

    def bootloadersize(self):
        """
        Returns the size of the bootloader.
        :return: int
        """
        if self.bootloaderhex is None:
            return 0

        return self.bootloaderhex.size()

    def tobinfile(self, fobj, start=None, end=None, pad=None, size=None):
        """
        Writes a binary version of source and bootloader respectivly to fobj which could be a
        file object or a file path.

        :param str fobj: File path or object the function writes to
        :return: None
        """
        # If there is a bootloader this will make the recursion call use the samme file object.
        if getattr(fobj, "write", None) is None:
            fobj = open(fobj, "wb")
            close_fd = True
        else:
            close_fd = False

        start_address = self.minaddr()
        size = self.size()
        super(nRFHex, self).tobinfile(fobj, start=start_address, size=size)

        if self.bootloaderhex is not None:
            self.bootloaderhex.tobinfile(fobj)

        if close_fd:
            fobj.close()


################################################################################
#from nordicsemi.dfu.package         import Package
#ref package.py
################################################################################
HexTypeToInitPacketFwTypemap = {
    HexType.APPLICATION: DFUType.APPLICATION,
    HexType.BOOTLOADER: DFUType.BOOTLOADER,
    HexType.SOFTDEVICE: DFUType.SOFTDEVICE,
    HexType.SD_BL: DFUType.SOFTDEVICE_BOOTLOADER
}


class PacketField(Enum):
    DEBUG_MODE = 1
    HW_VERSION = 2
    FW_VERSION = 3
    REQUIRED_SOFTDEVICES_ARRAY = 4

class Package(object):
    """
        Packages and unpacks Nordic DFU packages. Nordic DFU packages are zip files that contains firmware and meta-information
        necessary for utilities to perform a DFU on nRF5X devices.

        The internal data model used in Package is a dictionary. The dictionary is expressed like this in
         json format:

         {
            "manifest": {
                "bootloader": {
                    "bin_file": "asdf.bin",
                    "dat_file": "asdf.dat",
                    "init_packet_data": {
                        "application_version": null,
                        "device_revision": null,
                        "device_type": 5,
                        "firmware_hash": "asdfasdkfjhasdkfjashfkjasfhaskjfhkjsdfhasjkhf",
                        "softdevice_req": [
                            17,
                            18
                        ]
                    }
                }
        }

        Attributes application, bootloader, softdevice, softdevice_bootloader shall not be put into the manifest if they are null

    """

    DEFAULT_DEBUG_MODE = False
    DEFAULT_HW_VERSION = 0xFFFFFFFF
    DEFAULT_APP_VERSION = 0xFFFFFFFF
    DEFAULT_BL_VERSION = 0xFFFFFFFF
    DEFAULT_SD_REQ = [0xFFFE]
    DEFAULT_SD_ID = [0xFFFE]
    DEFAULT_DFU_VER = 0.5
    MANIFEST_FILENAME = "manifest.json"

    def __init__(self,
                 debug_mode=DEFAULT_DEBUG_MODE,
                 hw_version=DEFAULT_HW_VERSION,
                 app_version=DEFAULT_APP_VERSION,
                 bl_version=DEFAULT_BL_VERSION,
                 sd_req=DEFAULT_SD_REQ,
                 sd_id=DEFAULT_SD_ID,
                 app_fw=None,
                 bootloader_fw=None,
                 softdevice_fw=None,
                 key_file=None):
        """
        Constructor that requires values used for generating a Nordic DFU package.

        :param int debug_mode: Debug init-packet field
        :param int hw_version: Hardware version init-packet field
        :param int app_version: App version init-packet field
        :param int bl_version: Bootloader version init-packet field
        :param list sd_req: Softdevice Requirement init-packet field
        :param list sd_id: Softdevice Requirement init-packet field for the Application if softdevice_fw is set
        :param str app_fw: Path to application firmware file
        :param str bootloader_fw: Path to bootloader firmware file
        :param str softdevice_fw: Path to softdevice firmware file
        :param str key_file: Path to Signing key file (PEM)
        :return: None
        """

        init_packet_vars = {}
        if debug_mode is not None:
            init_packet_vars[PacketField.DEBUG_MODE] = debug_mode

        if hw_version is not None:
            init_packet_vars[PacketField.HW_VERSION] = hw_version

        if sd_id is not None:
            init_packet_vars[PacketField.REQUIRED_SOFTDEVICES_ARRAY] = sd_id

        self.firmwares_data = {}

        if app_fw:
            self.__add_firmware_info(firmware_type=HexType.APPLICATION,
                                     firmware_version=app_version,
                                     filename=app_fw,
                                     init_packet_data=init_packet_vars)

        if sd_req is not None:
            init_packet_vars[PacketField.REQUIRED_SOFTDEVICES_ARRAY] = sd_req

        if bootloader_fw:
            self.__add_firmware_info(firmware_type=HexType.BOOTLOADER,
                                     firmware_version=bl_version,
                                     filename=bootloader_fw,
                                     init_packet_data=init_packet_vars)

        if softdevice_fw:
            self.__add_firmware_info(firmware_type=HexType.SOFTDEVICE,
                                     firmware_version=0xFFFFFFFF,
                                     filename=softdevice_fw,
                                     init_packet_data=init_packet_vars)

        self.key_file = key_file

        self.work_dir = None
        self.manifest = None

    def __del__(self):
        """
        Destructor removes the temporary working directory
        :return:
        """
        if self.work_dir is not None:
            shutil.rmtree(self.work_dir)
        self.work_dir = None

    def rm_work_dir(self, preserve):
        # Delete the temporary directory
        if self.work_dir is not None:
            if not preserve:
                shutil.rmtree(self.work_dir)

        self.work_dir = None

    def parse_package(self, filename, preserve_work_dir=False):
        self.work_dir = self.__create_temp_workspace()

        self.zip_file = filename
        self.zip_dir  = os.path.join(self.work_dir, 'unpacked_zip')
        self.manifest = Package.unpack_package(filename, self.zip_dir)
        
        self.rm_work_dir(preserve_work_dir)

    def image_str(self, index, hex_type, img):
        type_strs = {HexType.SD_BL : "sd_bl", 
                    HexType.SOFTDEVICE : "softdevice",
                    HexType.BOOTLOADER : "bootloader",
                    HexType.APPLICATION : "application" }

        # parse init packet
        with open(os.path.join(self.zip_dir, img.dat_file), "rb") as imgf:
            initp_bytes = imgf.read()

        initp = InitPacketPB(from_bytes=initp_bytes)

        sd_req = ""
        for x in initp.init_command.sd_req:
            sd_req = sd_req + "0x{0:02X}, ".format(x)

        if len(sd_req) != 0:
            sd_req = sd_req[:-2]

        if (initp.packet.HasField('signed_command')):
            cmd = initp.packet.signed_command.command
            signature_type = SigningTypes(initp.packet.signed_command.signature_type).name
            signature_hex = binascii.hexlify(initp.packet.signed_command.signature)
        else:
            cmd = initp.packet.command
            signature_type = 'UNSIGNED'
            signature_hex = 'N/A'

        s = """|
|- Image #{0}:
   |- Type: {1}
   |- Image file: {2}
   |- Init packet file: {3}
      |
      |- op_code: {4}
      |- signature_type: {5}
      |- signature (little-endian): {6}
      |
      |- fw_version: 0x{7:08X} ({7})
      |- hw_version 0x{8:08X} ({8})
      |- sd_req: {9}
      |- type: {10}
      |- sd_size: {11}
      |- bl_size: {12}
      |- app_size: {13}
      |
      |- hash_type: {14}
      |- hash (little-endian): {15}
      |
      |- is_debug: {16}

""".format(index,
        type_strs[hex_type],
        img.bin_file,
        img.dat_file,
        CommandTypes(cmd.op_code).name,
        signature_type,
        signature_hex,
        cmd.init.fw_version,
        cmd.init.hw_version,
        sd_req,
        DFUType(cmd.init.type).name,
        cmd.init.sd_size,
        cmd.init.bl_size,
        cmd.init.app_size,
        HashTypes(cmd.init.hash.hash_type).name,
        binascii.hexlify(cmd.init.hash.hash),
        cmd.init.is_debug,
        )

        return s

    def __str__(self):
        
        imgs = ""
        i = 0
        if self.manifest.softdevice_bootloader:
            imgs = imgs + self.image_str(i, HexType.SD_BL, self.manifest.softdevice_bootloader)
            i = i + 1

        if self.manifest.softdevice:
            imgs = imgs + self.image_str(i, HexType.SOFTDEVICE, self.manifest.softdevice)
            i = i + 1

        if self.manifest.bootloader:
            imgs = imgs + self.image_str(i, HexType.BOOTLOADER, self.manifest.bootloader)
            i = i + 1

        if self.manifest.application:
            imgs = imgs + self.image_str(i, HexType.APPLICATION, self.manifest.application)
            i = i + 1

        s = """
DFU Package: <{0}>:
|
|- Image count: {1}
""".format(self.zip_file, i)

        s = s + imgs
        return s

    def generate_package(self, filename, preserve_work_dir=False):
        """
        Generates a Nordic DFU package. The package is a zip file containing firmware(s) and metadata required
        for Nordic DFU applications to perform DFU onn nRF5X devices.

        :param str filename: Filename for generated package.
        :param bool preserve_work_dir: True to preserve the temporary working directory.
        Useful for debugging of a package, and if the user wants to look at the generated package without having to
        unzip it.
        :return: None
        """
        self.zip_file = filename
        self.work_dir = self.__create_temp_workspace()

        if Package._is_bootloader_softdevice_combination(self.firmwares_data):
            # Removing softdevice and bootloader data from dictionary and adding the combined later
            softdevice_fw_data = self.firmwares_data.pop(HexType.SOFTDEVICE)
            bootloader_fw_data = self.firmwares_data.pop(HexType.BOOTLOADER)

            softdevice_fw_name = softdevice_fw_data[FirmwareKeys.FIRMWARE_FILENAME]
            bootloader_fw_name = bootloader_fw_data[FirmwareKeys.FIRMWARE_FILENAME]

            new_filename = "sd_bl.bin"
            sd_bl_file_path = os.path.join(self.work_dir, new_filename)

            nrf_hex = nRFHex(softdevice_fw_name, bootloader_fw_name)
            nrf_hex.tobinfile(sd_bl_file_path)

            softdevice_size = nrf_hex.size()
            bootloader_size = nrf_hex.bootloadersize()

            self.__add_firmware_info(firmware_type=HexType.SD_BL,
                                     firmware_version=bootloader_fw_data[FirmwareKeys.INIT_PACKET_DATA][PacketField.FW_VERSION],  # use bootloader version in combination with SD
                                     filename=sd_bl_file_path,
                                     init_packet_data=softdevice_fw_data[FirmwareKeys.INIT_PACKET_DATA],
                                     sd_size=softdevice_size,
                                     bl_size=bootloader_size)

        for key, firmware_data in self.firmwares_data.iteritems():

            # Normalize the firmware file and store it in the work directory
            firmware_data[FirmwareKeys.BIN_FILENAME] = \
                Package.normalize_firmware_to_bin(self.work_dir, firmware_data[FirmwareKeys.FIRMWARE_FILENAME])

            # Calculate the hash for the .bin file located in the work directory
            bin_file_path = os.path.join(self.work_dir, firmware_data[FirmwareKeys.BIN_FILENAME])
            firmware_hash = Package.calculate_sha256_hash(bin_file_path)
            bin_length = int(Package.calculate_file_size(bin_file_path))

            sd_size = 0
            bl_size = 0
            app_size = 0
            if key == HexType.APPLICATION:
                app_size = bin_length
            elif key == HexType.SOFTDEVICE:
                sd_size = bin_length
            elif key == HexType.BOOTLOADER:
                bl_size = bin_length
            elif key == HexType.SD_BL:
                bl_size = firmware_data[FirmwareKeys.BL_SIZE]
                sd_size = firmware_data[FirmwareKeys.SD_SIZE]

            init_packet = InitPacketPB(
                            from_bytes = None,
                            hash_bytes=firmware_hash,
                            hash_type=HashTypes.SHA256,
                            dfu_type=HexTypeToInitPacketFwTypemap[key],
                            is_debug=firmware_data[FirmwareKeys.INIT_PACKET_DATA][PacketField.DEBUG_MODE],
                            fw_version=firmware_data[FirmwareKeys.INIT_PACKET_DATA][PacketField.FW_VERSION],
                            hw_version=firmware_data[FirmwareKeys.INIT_PACKET_DATA][PacketField.HW_VERSION],
                            sd_size=sd_size,
                            app_size=app_size,
                            bl_size=bl_size,
                            sd_req=firmware_data[FirmwareKeys.INIT_PACKET_DATA][PacketField.REQUIRED_SOFTDEVICES_ARRAY])

            if (self.key_file is not None):
                signer = Signing()
                signer.load_key(self.key_file)
                signature = signer.sign(init_packet.get_init_command_bytes())
                init_packet.set_signature(signature, SigningTypes.ECDSA_P256_SHA256)

            # Store the .dat file in the work directory
            init_packet_filename = firmware_data[FirmwareKeys.BIN_FILENAME].replace(".bin", ".dat")

            with open(os.path.join(self.work_dir, init_packet_filename), 'wb') as init_packet_file:
                init_packet_file.write(init_packet.get_init_packet_pb_bytes())

            firmware_data[FirmwareKeys.DAT_FILENAME] = \
                init_packet_filename

        # Store the manifest to manifest.json
        manifest = self.create_manifest()

        with open(os.path.join(self.work_dir, Package.MANIFEST_FILENAME), "w") as manifest_file:
            manifest_file.write(manifest)

        # Package the work_dir to a zip file
        Package.create_zip_package(self.work_dir, filename)

        # Delete the temporary directory
        self.rm_work_dir(preserve_work_dir)

    @staticmethod
    def __create_temp_workspace():
        return tempfile.mkdtemp(prefix="nrf_dfu_pkg_")

    @staticmethod
    def create_zip_package(work_dir, filename):
        files = os.listdir(work_dir)

        with ZipFile(filename, 'w') as package:
            for _file in files:
                file_path = os.path.join(work_dir, _file)
                package.write(file_path, _file)

    @staticmethod
    def calculate_file_size(firmware_filename):
        b = os.path.getsize(firmware_filename)
        return b

    @staticmethod
    def calculate_sha256_hash(firmware_filename):
        read_buffer = 4096

        digest = hashlib.sha256()

        with open(firmware_filename, 'rb') as firmware_file:
            while True:
                data = firmware_file.read(read_buffer)

                if data:
                    digest.update(data)
                else:
                    break

        # return hash in little endian
        sha256 = digest.digest()
        return sha256[31::-1]

    @staticmethod
    def calculate_crc(crc, firmware_filename):
        """
        Calculates CRC16 has on provided firmware filename

        :type str firmware_filename:
        """
        data_buffer = b''
        read_size = 4096

        with open(firmware_filename, 'rb') as firmware_file:
            while True:
                data = firmware_file.read(read_size)

                if data:
                    data_buffer += data
                else:
                    break
        if crc == 16:
            return calc_crc16(data_buffer, 0xffff)
        elif crc == 32:
            return binascii.crc32(data_buffer)
        else:
            raise NordicSemiException("Invalid CRC type")

    def create_manifest(self):
        manifest = ManifestGenerator(self.firmwares_data)
        return manifest.generate_manifest()

    @staticmethod
    def _is_bootloader_softdevice_combination(firmwares):
        return (HexType.BOOTLOADER in firmwares) and (HexType.SOFTDEVICE in firmwares)

    def __add_firmware_info(self, firmware_type, firmware_version, filename, init_packet_data, sd_size=None, bl_size=None):
        self.firmwares_data[firmware_type] = {
            FirmwareKeys.FIRMWARE_FILENAME: filename,
            FirmwareKeys.INIT_PACKET_DATA: init_packet_data.copy(),
            # Copying init packet to avoid using the same for all firmware
            }

        if firmware_type == HexType.SD_BL:
            self.firmwares_data[firmware_type][FirmwareKeys.SD_SIZE] = sd_size
            self.firmwares_data[firmware_type][FirmwareKeys.BL_SIZE] = bl_size
        
        if firmware_version is not None:
            self.firmwares_data[firmware_type][FirmwareKeys.INIT_PACKET_DATA][PacketField.FW_VERSION] = firmware_version

    @staticmethod
    def normalize_firmware_to_bin(work_dir, firmware_path):
        firmware_filename = os.path.basename(firmware_path)
        new_filename = firmware_filename.replace(".hex", ".bin")
        new_filepath = os.path.join(work_dir, new_filename)

        if not os.path.exists(new_filepath):
            temp = nRFHex(firmware_path)
            temp.tobinfile(new_filepath)

        return new_filepath

    @staticmethod
    def unpack_package(package_path, target_dir):
        """
        Unpacks a Nordic DFU package.

        :param str package_path: Path to the package
        :param str target_dir: Target directory to unpack the package to
        :return: Manifest Manifest: Returns a manifest back to the user. The manifest is a parse datamodel
        of the manifest found in the Nordic DFU package.
        """

        if not os.path.isfile(package_path):
            raise NordicSemiException("Package {0} not found.".format(package_path))

        target_dir = os.path.abspath(target_dir)
        target_base_path = os.path.dirname(target_dir)

        if not os.path.exists(target_base_path):
            raise NordicSemiException("Base path to target directory {0} does not exist.".format(target_base_path))

        if not os.path.isdir(target_base_path):
            raise NordicSemiException("Base path to target directory {0} is not a directory.".format(target_base_path))

        if os.path.exists(target_dir):
            raise NordicSemiException(
                "Target directory {0} exists, not able to unpack to that directory.",
                target_dir)

        with ZipFile(package_path, 'r') as pkg:
            pkg.extractall(target_dir)

            with open(os.path.join(target_dir, Package.MANIFEST_FILENAME), 'r') as f:
                _json = f.read()
                """:type :str """

                return Manifest.from_json(_json)


################################################################################
#from nordicsemi.dfu.dfu import Dfu
#ref dfu.py
################################################################################
class Dfu(object):
    """ Class to handle upload of a new hex image to the device. """

    def __init__(self, zip_file_path, dfu_transport):
        """
        Initializes the dfu upgrade, unpacks zip and registers callbacks.

        @param zip_file_path: Path to the zip file with the firmware to upgrade
        @type zip_file_path: str
        @param dfu_transport: Transport backend to use to upgrade
        @type dfu_transport: nordicsemi.dfu.dfu_transport.DfuTransport
        @return
        """
        self.temp_dir           = tempfile.mkdtemp(prefix="nrf_dfu_")
        self.unpacked_zip_path  = os.path.join(self.temp_dir, 'unpacked_zip')
        self.manifest           = Package.unpack_package(zip_file_path, self.unpacked_zip_path)

        self.dfu_transport      = dfu_transport

    def __del__(self):
        """
        Destructor removes the temporary directory for the unpacked zip
        :return:
        """
        shutil.rmtree(self.temp_dir)


    def _dfu_send_image(self, firmware):
        time.sleep(3)
        self.dfu_transport.open()

        start_time = time.time()

        logger.info("Sending init packet...")
        with open(os.path.join(self.unpacked_zip_path, firmware.dat_file), 'rb') as f:
            data    = f.read()
            self.dfu_transport.send_init_packet(data)

        logger.info("Sending firmware file...")
        with open(os.path.join(self.unpacked_zip_path, firmware.bin_file), 'rb') as f:
            data    = f.read()
            self.dfu_transport.send_firmware(data)

        end_time = time.time()
        logger.info("Image sent in {0}s".format(end_time - start_time))

        self.dfu_transport.close()


    def dfu_send_images(self):
        """
        Does DFU for all firmware images in the stored manifest.
        :return:
        """
        if self.manifest.softdevice_bootloader:
            logger.info("Sending SoftDevice+Bootloader image.")
            self._dfu_send_image(self.manifest.softdevice_bootloader)

        if self.manifest.softdevice:
            logger.info("Sending SoftDevice image...")
            self._dfu_send_image(self.manifest.softdevice)

        if self.manifest.bootloader:
            logger.info("Sending Bootloader image.")
            self._dfu_send_image(self.manifest.bootloader)

        if self.manifest.application:
            logger.info("Sending Application image.")
            self._dfu_send_image(self.manifest.application)


    def dfu_get_total_size(self):
        total_size = 0

        if self.manifest.softdevice_bootloader:
            total_size += os.path.getsize(os.path.join(self.unpacked_zip_path,
                                                       self.manifest.softdevice_bootloader.bin_file))

        if self.manifest.softdevice:
            total_size += os.path.getsize(os.path.join(self.unpacked_zip_path,
                                                       self.manifest.softdevice.bin_file))

        if self.manifest.bootloader:
            total_size += os.path.getsize(os.path.join(self.unpacked_zip_path,
                                                       self.manifest.bootloader.bin_file))

        if self.manifest.application:
            total_size += os.path.getsize(os.path.join(self.unpacked_zip_path,
                                                       self.manifest.application.bin_file))

        return total_size


################################################################################
#ref __main__.py
################################################################################
global_bar = None
def update_progress(progress=0):
    if global_bar:
        global_bar.update(progress)


def do_serial(package, port, flow_control, packet_receipt_notification, baud_rate, ping):

    if flow_control is None:
        flow_control = DfuTransportSerial.DEFAULT_FLOW_CONTROL
    if packet_receipt_notification is None:
        packet_receipt_notification = DfuTransportSerial.DEFAULT_PRN
    if baud_rate is None:
        baud_rate = DfuTransportSerial.DEFAULT_BAUD_RATE
    if ping is None:
        ping = False

    logger.info("Using board at serial port: {}".format(port))
    serial_backend = DfuTransportSerial(com_port=str(port), baud_rate=baud_rate,
                    flow_control=flow_control, prn=packet_receipt_notification, do_ping=ping)
    serial_backend.register_events_callback(DfuEvent.PROGRESS_EVENT, update_progress)
    dfu = Dfu(zip_file_path = package, dfu_transport = serial_backend)

    if logger.getEffectiveLevel() > logging.INFO:
        with click.progressbar(length=dfu.dfu_get_total_size()) as bar:
            global global_bar
            global_bar = bar
            dfu.dfu_send_images()
    else:
        dfu.dfu_send_images()

    click.echo("Device programmed.")


default_check_UART_Timeout                      = 0.1
default_check_UART_flowcontrol                  = False
default_check_UART_baud_rate                    = 115200
################################################################################
#wisol commands
################################################################################
check_UART_CMD_ENTER_PARSER             = "W"

#OP CODE
check_UART_CMD_BOOT                     = "X"
check_UART_CMD_REBOOT                   = "R"
check_UART_CMD_ENTER_SERIAL_DFU         = "S"
check_UART_CMD_ENTER_BLE_DFU            = "B"
check_UART_CMD_TEST_LOG                 = "L"
check_UART_CMD_GET_BL_VER               = "V"
check_UART_CMD_CONTROL_DBG_IF           = "D"  #TODO  //need hash

#RESP
check_UART_RESP_OK                      = "K"
check_UART_RESP_ERROR                   = "E"


def check_dfu_serial_response(serial_port):
    response_flag = False
    byte = serial_port.read(1)

    if byte == check_UART_RESP_OK:
        response_flag = True
    return response_flag
    

def do_connect_dfu_serial(serial_port):
    UART_CONNECT_CHK_CNT = 100

    click.echo("Connecting...")
    con_chk_count = 0
    con_resp_OK_count = 0
    do_serial_flag = False
    while con_chk_count < UART_CONNECT_CHK_CNT:
        serial_port.write(check_UART_CMD_ENTER_PARSER)
        resp_ok = check_dfu_serial_response(serial_port)
        if resp_ok:
            con_resp_OK_count += 1
            if con_resp_OK_count > 2:
                break;
        con_chk_count += 1
#            click.echo("Try {0}".format(str(con_chk_count)))
    if con_resp_OK_count > 1:
        do_serial_flag = True;
        click.echo("Connected!")
    return do_serial_flag


def do_serial_dfu(package, port, flow_control, packet_receipt_notification, baud_rate):   
    if flow_control is None:
        flow_control = default_check_UART_flowcontrol
    if packet_receipt_notification is None:
        packet_receipt_notification = 0
    if baud_rate is None:
        baud_rate = default_check_UART_baud_rate

    do_serial_flag = False
    
    try:
        serial_port = Serial(port=port, baudrate=baud_rate, rtscts=flow_control, timeout=default_check_UART_Timeout)

        if do_connect_dfu_serial(serial_port):
            serial_port.write(check_UART_CMD_ENTER_SERIAL_DFU)
            if check_dfu_serial_response(serial_port):
                do_serial_flag = True

        serial_port.close()
        if do_serial_flag:
            click.echo("Do Serial Download.")
            do_serial(package, port, flow_control, packet_receipt_notification, baud_rate, True)
        else:
            click.echo("Error.")

    except Exception as err:
        click.echo("Exception.")
        click.echo("Error.")
        click.echo(err)


def do_bl_ver(port, flow_control, baud_rate):   
    if flow_control is None:
        flow_control = default_check_UART_flowcontrol
    if baud_rate is None:
        baud_rate = default_check_UART_baud_rate

    try:
        serial_port = Serial(port=port, baudrate=baud_rate, rtscts=flow_control, timeout=default_check_UART_Timeout)

        if do_connect_dfu_serial(serial_port):
            serial_port.write(check_UART_CMD_GET_BL_VER)
            serial_port.timeout = 1
            read_result = serial_port.readline()
            click.echo("bl_ver:{}".format(read_result))
            serial_port.write(check_UART_CMD_BOOT)
        else:
            click.echo("Error.")
        serial_port.close()

    except Exception as err:
        click.echo("Exception.")
        click.echo("Error.")
        click.echo(err)


def do_dap_ctrl(enable_flag, key_val, port, flow_control, baud_rate):   
    if flow_control is None:
        flow_control = default_check_UART_flowcontrol
    if baud_rate is None:
        baud_rate = default_check_UART_baud_rate
    if len(key_val) != 8:
        click.echo("Error. Please enter 8 byte key.")
        return

    try:
        serial_port = Serial(port=port, baudrate=baud_rate, rtscts=flow_control, timeout=default_check_UART_Timeout)

        if do_connect_dfu_serial(serial_port):
            serial_port.write(check_UART_CMD_CONTROL_DBG_IF)
            if enable_flag:
                serial_port.write("E")
            else:
                serial_port.write("D")
            print(key_val)
            dates_bytes = key_val.encode('ascii')
            serial_port.write(dates_bytes)
            serial_port.timeout = 1
            if check_dfu_serial_response(serial_port):
                do_serial_flag = True
                click.echo("Done.")
            else:
                click.echo("Error.")
            serial_port.write(check_UART_CMD_REBOOT)
        else:
            click.echo("Error.")
        serial_port.close()

    except Exception as err:
        click.echo("Exception.")
        click.echo("Error.")
        click.echo(err)


@click.group()
@click.option('-v', '--verbose',
              help='Show verbose information.',
              count=True)
@click.option('-o', '--output',
              help='Log output to file',
              metavar='<filename>')
def cli(verbose, output):
    #click.echo('verbosity: %s' % verbose)
    if verbose == 0:
        log_level = logging.ERROR
    elif verbose == 1:
        log_level = logging.INFO
    else:
        log_level = logging.DEBUG

    logging.basicConfig(format='%(asctime)s %(message)s', level=log_level)

    if (output):
        root = logging.getLogger('')
        fh = logging.FileHandler(output)
        fh.setLevel(log_level)
        fh.setFormatter(logging.Formatter('%(asctime)s %(message)s'))
        root.addHandler(fh)


@cli.command()
def version():
    """Display tool's version."""
    click.echo("sdfu_w version {}".format(SDFU_W_VERSION))
    logger.info("PyPi URL: https://pypi.python.org/pypi/nrfutil")
    logger.debug("GitHub URL: https://github.com/NordicSemiconductor/pc-nrfutil")


@cli.command(short_help="Update the firmware on a device over a UART serial connection (connect and run). The DFU target must be a chip using digital I/O pins as an UART.")
@click.option('-pkg', '--package',
              help='Filename of the DFU package.',
              type=click.Path(exists=True, resolve_path=True, file_okay=True, dir_okay=False),
              required=True)
@click.option('-p', '--port',
              help='Serial port address to which the device is connected. (e.g. COM1 in windows systems, /dev/ttyACM0 in linux/mac)',
              type=click.STRING,
              required=True)
@click.option('-fc', '--flow-control',
              help='To enable flow control set this flag to 1',
              type=click.BOOL,
              required=False)
@click.option('-prn', '--packet-receipt-notification',
              help='Set the packet receipt notification value',
              type=click.INT,
              required=False)
@click.option('-b', '--baud-rate',
              help='Set the baud rate',
              type=click.INT,
              required=False)
def serial_dfu(package, port, flow_control, packet_receipt_notification, baud_rate):
    """Perform a Device Firmware Update on a device with a bootloader that supports UART serial DFU for wisol."""
    do_serial_dfu(package, port, flow_control, packet_receipt_notification, baud_rate)


@cli.command(short_help="Get Bootloader Version with UART.")
@click.option('-p', '--port',
              help='Serial port address to which the device is connected. (e.g. COM1 in windows systems, /dev/ttyACM0 in linux/mac)',
              type=click.STRING,
              required=True)
@click.option('-fc', '--flow-control',
              help='To enable flow control set this flag to 1',
              type=click.BOOL,
              required=False)
@click.option('-b', '--baud-rate',
              help='Set the baud rate',
              type=click.INT,
              required=False)
def bl_ver(port, flow_control, baud_rate):
    do_bl_ver(port, flow_control, baud_rate)


@cli.command(short_help="Enable or Disable DAP(Debug Access Port) with UART.")
@click.option('-e', '--enable-flag',
              help='enable val. 1 or 0',
              type=click.BOOL,
              required=True)
@click.option('-k', '--key-val',
              help='key (8byte. e.g. 0248ACE1)',
              type=click.STRING,
              required=True)
@click.option('-p', '--port',
              help='Serial port address to which the device is connected. (e.g. COM1 in windows systems, /dev/ttyACM0 in linux/mac)',
              type=click.STRING,
              required=True)
@click.option('-fc', '--flow-control',
              help='To enable flow control set this flag to 1',
              type=click.BOOL,
              required=False)
@click.option('-b', '--baud-rate',
              help='Set the baud rate',
              type=click.INT,
              required=False)
def dap_ctrl(enable_flag, key_val, port, flow_control, baud_rate):
    do_dap_ctrl(enable_flag, key_val, port, flow_control, baud_rate)


if __name__ == '__main__':
    cli()
