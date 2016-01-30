/*
 * Copyright © 2014 Kevin Mark
 *
 * This file is part of Bee.
 *
 * Bee is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Bee is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Bee.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Bee.h"
#include <assert.h>

Bee::Bee(HardwareSerial *serial) {
    _serial = serial;
    _serialSoft = NULL;
    _currentPacket.offset = _currentPacket.size = _currentPacket.checksum = 0;
    _currentPacket.isEscaped = false;
}

Bee::Bee(SoftwareSerial *serial) {
    _serial = NULL;
    _serialSoft = serial;
    _currentPacket.offset = _currentPacket.size = _currentPacket.checksum = 0;
    _currentPacket.isEscaped = false;
}


void Bee::tick() {
    if(!_available()) {
        return;
    }

    uint8_t c = _read();
    
    if(c == 0x7E) {
        _currentPacket.offset = 0;
        _currentPacket.size = 0;
        _currentPacket.checksum = 0;
        _currentPacket.isEscaped = false;
        memset((void *)_currentPacket.data, 0, sizeof(_currentPacket.data));
        return;
    }
    switch(++_currentPacket.offset) {
        case 1:
            _currentPacket.size = c << 8;
            return;
        case 2:
            _currentPacket.size = (_currentPacket.size | c) + 3;
            // _currentPacket.data has already been zeroed
            _currentPacket.data[0] = 0x7E;
            _currentPacket.data[1] = ((_currentPacket.size - 3) >> 8) & 0xFF;
            _currentPacket.data[2] = c;
            return;
    }
    if(_currentPacket.isEscaped) {
        _currentPacket.isEscaped = false;
        c ^= 0x20;
    }
    else if(c == 0x7D) {
        _currentPacket.isEscaped = true;
        _currentPacket.offset--;
        return;
    }

    _currentPacket.data[_currentPacket.offset] = c;
    
    if(_currentPacket.offset == _currentPacket.size) {
        _currentPacket.checksum = 0xFF - (_currentPacket.checksum & 0xFF);
        if(_currentPacket.checksum == (uint8_t)c) {
            // Process contents of frame
            _processFrame();
        }
        return;
    }

    _currentPacket.checksum += c;
}

void Bee::_processFrame() {
    _pointerFrame.frameType = &_currentPacket.data[3];
    switch(*_pointerFrame.frameType) {
        case ATCommandResp:
            _pointerFrame.data = &_currentPacket.data[5];
            _pointerFrame.dataLength = _currentPacket.size - 5;
            break;
        case ExpRxIndicator:
            _pointerFrame.source64 = (uint64_t*)&_currentPacket.data[4];
            _pointerFrame.source16 = (uint16_t*)&_currentPacket.data[12];
            // 16 and 64 bit source addrs are big endian, Arduino is little
            *_pointerFrame.source16 = (*_pointerFrame.source16 >> 8) |
              (*_pointerFrame.source16 << 8);
            *_pointerFrame.source64 = __builtin_bswap64(*_pointerFrame.source64);
            _pointerFrame.data = &_currentPacket.data[21];
            _pointerFrame.dataLength = _currentPacket.size - 21;
            break;
        case ZigBeeRecv:
            _pointerFrame.source64 = (uint64_t*)&_currentPacket.data[4];
            _pointerFrame.source16 = (uint16_t*)&_currentPacket.data[12];
            // 16 and 64 bit source addrs are big endian, Arduino is little
            *_pointerFrame.source16 = (*_pointerFrame.source16 >> 8) |
              (*_pointerFrame.source16 << 8);
            *_pointerFrame.source64 = __builtin_bswap64(*_pointerFrame.source64);
            _pointerFrame.data = &_currentPacket.data[15];
            _pointerFrame.dataLength = _currentPacket.size - 15;
            break;
    }
    _callback(&_pointerFrame);
}

uint8_t Bee::_checksum(uint8_t *packet, uint16_t size) {
    unsigned int sum = 0;
    for(uint16_t i = 3; i < size; i++) {
        sum += packet[i];
    }
    return 0xFF - (sum & 0xFF);
}

void Bee::sendLocalAT(atcmd command,const uint8_t * params, uint8_t paramLen) {

    uint8_t at[8] = { 0x7E, 0x00, 0x04, 0x08, _frameId, command[0], command[1], 0x00 };
    uint8_t atx[32];
    assert(paramLen <= (sizeof atx - sizeof at));

    memcpy(atx,at,sizeof at);

    if ( paramLen > 0 ){
        memcpy(atx + (sizeof at) -1,params,paramLen);
        atx[2] = 4 + paramLen;
        atx[(sizeof at) -1 + paramLen] = 0x00;
        atx[(sizeof at) -1 + paramLen] = _checksum(atx, (sizeof at) + paramLen);
    } else {
        atx[(sizeof at) -1] = _checksum(atx, sizeof at);
    }
    _write(atx, (sizeof at) + paramLen);
    _frameId = (_frameId+1)%255;
}

void Bee::sendLocalAT(atcmd command) {
    sendLocalAT(command,NULL,0);
}

void Bee::sendData(String s) {
    int len = s.length();
    uint8_t packet[len + 18];
    memset(packet, 0x00, sizeof packet);
    packet[0] = 0x7E;
    packet[1] = 0x00;
    packet[2] = 0x00;
    packet[3] = 0x10;
    packet[4] = 0x01;
    packet[5] = (uint8_t)(addr64 >> 56);
    packet[6] = (uint8_t)(addr64 >> 48);
    packet[7] = (uint8_t)(addr64 >> 40);
    packet[8] = (uint8_t)(addr64 >> 32);
    packet[9] = (uint8_t)(addr64 >> 24);
    packet[10] = (uint8_t)(addr64 >> 16);
    packet[11] = (uint8_t)(addr64 >> 8);
    packet[12] = (uint8_t)(addr64);
    packet[13] = 0xFF;
    packet[14] = 0xFE;
    packet[15] = 0x00;
    packet[16] = 0x00;
    uint8_t *p = packet;
    p += sizeof(uint8_t) * 17;
    s.toCharArray((char *)p, len + 1);
    uint16_t packetLength = 14 + len;
    packet[1] = packetLength >> 8;
    packet[2] = packetLength & 0xFF;
    packet[len + 17] = _checksum(packet, sizeof packet);
    _write(packet, sizeof packet);
}

void Bee::sendData(uint8_t *data, uint16_t size) {
    uint8_t init[17];
    uint16_t packetLength = sizeof(init) + size - 3;
    uint16_t checksum = 0;

    init[0] = 0x7E;
    init[1] = 0x00;
    init[2] = 0x00;
    init[3] = 0x10;
    init[4] = 0x00; // Disable Acks on the FrameId
    init[5] = (uint8_t)(addr64 >> 56);
    init[6] = (uint8_t)(addr64 >> 48);
    init[7] = (uint8_t)(addr64 >> 40);
    init[8] = (uint8_t)(addr64 >> 32);
    init[9] = (uint8_t)(addr64 >> 24);
    init[10] = (uint8_t)(addr64 >> 16);
    init[11] = (uint8_t)(addr64 >> 8);
    init[12] = (uint8_t)(addr64);
    init[13] = 0xFF;
    init[14] = 0xFE;
    init[15] = 0x01; // Set TX Radius
    init[16] = 0x01; // TX Option bit 



    // Write init bytes
    for(uint8_t i = 0; i < sizeof(init); i++) {
        uint8_t ibyte = init[i];
        if(i == 1) { // High length byte
            ibyte = packetLength >> 8;
        } else if(i == 2) { // Low length byte
            ibyte = packetLength & 0xFF;
        }
        if(i != 0) {
            if(_escapeRequired(ibyte)) {
                _write(0x7D);
                ibyte ^= 0x20;
            }
        }

        _write(ibyte);
        if(i > 2) {
            checksum += init[i];
        }
    }


    
    // Write data bytes
    for(uint16_t i = 0; i < size; i++) {
        uint8_t ibyte = data[i];
        checksum += ibyte; // Only pre-escape bytes are used for the checksum
        if(_escapeRequired(ibyte)) {
            _write(0x7D);
            ibyte ^= 0x20;
        }
        _write(ibyte);
    }

    // Write checksum byte
    _write(0xFF - (checksum & 0xFF));


}

bool Bee::_escapeRequired(uint8_t c) {
    switch(c) {
        case 0x11:
        case 0x13:
        case 0x7D:
        case 0x7E:
            return true;
    }
    return false;
}

void Bee::setCallback(BeeCallback callback) {
    _callback = callback;
}

uint16_t Bee::_available() {
    if(_serial == NULL) {
        return _serialSoft->available();
    }
    return _serial->available();
}

uint8_t Bee::_read() {
    if(_serial == NULL) {
        return _serialSoft->read();
    }
    return _serial->read();
}

void Bee::_write(uint8_t c) {

    if(_serial == NULL) {
        _serialSoft->write(c);
        return;
    }
    _serial->write(c);
}

void Bee::_write(uint8_t *c, uint16_t size) {
    
    if(_serial == NULL) {
        _serialSoft->write(c, size);
        return;
    }
    _serial->write(c, size);
}

void Bee::begin(long baud) {
   _baud = baud;
   if(_serial == NULL) {
        _serialSoft->begin(_baud);
        return;
    }
    _serial->begin(_baud);
}

void Bee::end() {
    if(_serial == NULL) {
        _serialSoft->end();
        return;
    }
    _serial->end();
}
