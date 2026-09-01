#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2013 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
""" CRTP UDP Driver. Work either with the UDP server or with an UDP device
See udpserver.py for the protocol.

SWARM-SAFE VERSION:
- connect() now uses the IP/port actually given in the URI, instead of a
  hardcoded address. This is required so each Crazyflie() instance can
  point at a *different* drone.
- connect() binds the local UDP socket to port 0 (OS-assigned ephemeral
  port) instead of a fixed port. Two UdpDriver instances in the same
  process can no longer collide on "address already in use".
"""
import re
import socket
import binascii
import time
from urllib.parse import urlparse

from .crtpdriver import CRTPDriver
from .crtpstack import CRTPPacket
from .exceptions import WrongUriType

__author__ = 'Bitcraze AB'
__all__ = ['UdpDriver']


class UdpDriver(CRTPDriver):

    def __init__(self):
        CRTPDriver.__init__(self)
        self.debug = False
        self.link_error_callback = None
        self.link_quality_callback = None
        self.needs_resending = True
        self.socket = None
        self.addr = None

    def connect(self, uri, linkQualityCallback, linkErrorCallback):
        if not re.search('^udp://', uri):
            raise WrongUriType('Not an UDP URI')

        parse = urlparse(uri)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Use the address actually encoded in the URI. This is what makes
        # multiple simultaneous drones possible -- each Crazyflie() gets
        # its own URI, e.g. udp://192.168.0.11:2390 and udp://192.168.0.12:2390.
        self.addr = (parse.hostname, parse.port)

        # Bind to an OS-assigned ephemeral local port (0 = "pick any free
        # port"). A fixed local port like 2399 can only be bound once per
        # process -- fatal for swarming, where you need N simultaneous
        # UdpDriver instances.
        self.socket.bind(('', 0))
        self.socket.connect(self.addr)

        self.socket.sendto('\xFF\x01\x01\x01'.encode(), self.addr)

        if self.debug:
            local_port = self.socket.getsockname()[1]
            print("Connected to UDP server")
            print("Local port: %d" % local_port)
            print("Server is: %s:%d" % self.addr)

    def receive_packet(self, time=0):
        try:
            data, addr = self.socket.recvfrom(1024)
        except OSError:
            if self.debug:
                print("Socket error: socket might be closed.")
            return None

        if data:
            # take the final byte as the checksum
            cksum_recv = data[len(data)-1]
            # remove the checksum from the data
            data = data[0:(len(data)-1)]
            # calculate checksum and check it with the last byte
            cksum = 0
            for i in data[0:]:
                cksum += i
            cksum %= 256
            if cksum != cksum_recv:
                if self.debug:
                    print("Checksum error {} != {}".format(cksum, cksum_recv))
                return None
            pk = CRTPPacket(data[0], list(data[1:]))
            # print the raw date
            if self.debug:
                print("recv: {}".format(binascii.hexlify(bytearray(data))))
            return pk

        else:
            return None

    def send_packet(self, pk):
        raw = (pk.header,) + pk.datat
        cksum = 0
        for i in raw:
            cksum += i
        cksum %= 256
        raw = raw + (cksum,)
        # change the tuple to bytes
        raw = bytearray(raw)
        self.socket.sendto(raw, self.addr)
        # print the raw date
        if self.debug:
            print("send: {}".format(binascii.hexlify(raw)))

    def close(self):
        # Remove this from the server clients list
        try:
            self.socket.sendto('\xFF\x01\x02\x02'.encode(), self.addr)
        except OSError:
            pass
        if self.debug:
            print("Disconnected from UDP server")
            print("Server is: %s:%d" % self.addr)
        time.sleep(1)
        if self.socket is not None:
            self.socket.close()
        self.socket = None

    def get_name(self):
        return 'udp'

    def scan_interface(self, address):
        # Previously hardcoded to a single drone's address, which made
        # auto-scan useless for a swarm (it could only ever "discover"
        # one fixed IP). If you pass a specific address in, we report
        # that one back; otherwise we don't invent an address, since we
        # have no real broadcast-discovery mechanism for the LiteWing.
        if address:
            return [[address, ''], ]
        return []
