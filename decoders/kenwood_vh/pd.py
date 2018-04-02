##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2018 Jannis Achstetter <kripton@kripserver.net>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

##
## Short summary of the protocol and the algorithm:
## - Two lines, DATA and ACTIVE. ACTIVE is not used since DATA alone is enough
## - Data packets start with DATA line being HIGH for 5ms (= RESET)
## - Every reset is followed by 16 bit of payload
## - DATA line is return-to-zero, bit values are determined by length of the 
##   LOW state. HIGH state is always 2ms (expect RESET, see above)
## - ARBITRARY definition: LOW for 2ms => 0, LOW for 5ms => 1
## - To decode bits, we only have a look at the times between two FALLING edges
##

import sigrokdecode as srd

class Decoder(srd.Decoder):
    api_version = 3
    id = 'kenwood_vh'
    name = 'Kenwood VH'
    longname = 'Kenwood VH'
    desc = 'SYSTEM CONTROL protocol of Kenwood VH HiFi system.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['control_commands']
    channels = (
        {'id': 'data', 'name': 'Kenwood SYSTEM CONTROL data', 'desc': 'Data line of the SYSTEM CONTROL cable'},
    )
    annotations = (
        ('bit', 'Bits'),
        ('word', 'Words'),
        ('command', 'Command'),
    )
    annotation_rows = (
        ('bit', 'Bits', (0,)),
        ('word', 'Words', (1,)),
        ('command', 'Command', (2,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        # Current value of the word being decoded
        self.content = 0
        # Length of one sample in microseconds
        self.sample_usec = None
        # Absolute number of sample containing the last rising edge
        self.lastRising = -1
        # Absolute number of sample containing the last falling edge
        self.lastFalling = -1
        # Absolute number of sample being the start of the command
        self.commandStart = -1
        # Absolute number of sample being the start of the word
        self.wordStart = -1
        # Number of bits of that word we already decoded
        self.num_bits = 0
        # State our decoding state machine is in
        self.state = 'FIND RESET'

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.sample_usec = 1 / value * 1000000

    def contentToDescription(self, content):
        return {
            0x1000: 'System ON',		# From amp
            0x1080: 'System OFF',		# From amp
        }.get(content, '????')			# ???? is default if content didn't match

    def putr(self, data):
        self.put(self.run_start, self.samplenum, self.out_ann, data)

    def decode(self):
        if not self.sample_usec:
            raise SamplerateError('Cannot decode without samplerate.')

        while True:

            # Seek for a high-signal that is at least 4ms long (= RESET)
            if self.state == 'FIND RESET':
                (data,) = self.wait([{0: 'f'},{0: 'r'}])
                if self.matched == (True, False):
                    # Falling edge, check the time since last rising
                    self.lastFalling = self.samplenum
                    timeHigh = (self.lastFalling - self.lastRising) * self.sample_usec
                    if (timeHigh > 4000):
                        self.put(self.lastRising, self.lastFalling, self.out_ann, [0, ['RESET']])
                        self.commandStart = self.lastRising
                        self.wordStart = self.lastFalling
                        self.state = 'BITS'
                        self.content = 0
                        self.num_bits = 0
                elif self.matched == (False, True):
                    # Rising edge, just remember and go ahead
                    self.lastRising = self.samplenum

            # Decode the bit values from the duration of the low phases
            if self.state == 'BITS':
                (data,) = self.wait([{0: 'f'}])
                timeLow = (self.samplenum - self.lastFalling) * self.sample_usec
                if (timeLow > 3000 and timeLow < 5000):
                    self.content = (self.content << 1)
                    self.num_bits = self.num_bits + 1
                    self.put(self.lastFalling, self.samplenum, self.out_ann, [0, ['0']])
                elif (timeLow > 6000 and timeLow < 8000):
                    self.content = (self.content << 1) + 1
                    self.num_bits = self.num_bits + 1
                    self.put(self.lastFalling, self.samplenum, self.out_ann, [0, ['1']])
                self.lastFalling = self.samplenum

                # If we have one word decoded, emit it and go back to FIND RESET state
                if (self.num_bits == 16):
                    self.put(self.wordStart, self.lastFalling, self.out_ann, [1, [format(self.content, '#06x')]])
                    self.put(self.commandStart, self.lastFalling, self.out_ann, [2, [self.contentToDescription(self.content)]])
                    self.state = 'FIND RESET'
