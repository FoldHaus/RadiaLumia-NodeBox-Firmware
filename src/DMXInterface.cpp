
#include "DMXInterface.h"
#include "Arduino.h"

#include <avr/io.h>

#include <CRC8.h>

using namespace Foldhaus;
using namespace libCameron;

libCameron::TripleBuffer<DMXInterface::Message, true> DMXInterface::incoming;

DecPrintFormatter DMXInterface::debug(&DMXInterface::sendByte);

static CRC8 CRC;

void USART_RX_vect() {
  DMXInterface::receiveByte();
}

void DMXInterface::init() {
  // at F_CPU == 16MHz, this is 250kBaud
  UBRR0 = 3;

  // Set default
  UCSR0C = 0b00000110;

  // Set default, clear TxC flag for good measure
  UCSR0A = 0b01000000;

  // Make sure incoming buffer is ready to receive first block
  incoming.markNewestBuffer();

  // Enable transmitter, receiver, and rx interrupt
  UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);

  // Enable Tx output
  DDRD |= 0b10;

  // Enable interrupts
  sei();
}

void DMXInterface::receiveByte() {
  // -2 : init - lastSeen is invalid
  // -1 : wait - Waiting for correct gap
  // Otherwise current channel number
  static int channel = -2;

  // If frame error, that's the start of a DMX512 packet
  if (UCSR0A & (1 << FE0) && UDR0 == 0) {
    channel = -1;
    return;
  }

  // DMX packet type. Always 0
  if (channel == -1) {
    if (UDR0 != 0) {
      channel = -2;
    }
    return;
  }

  incoming.getWriteBuffer()->feed(++channel, UDR0);
}

void DMXInterface::Message::feed(uint16_t channel, uint8_t b) {
  if (dmxOffset) {
    if (channel < dmxOffset) return;
    channel -= dmxOffset;
  }
  if (channel >= length) return;

  raw[channel] = b;

  if (channel == length - 1) {
    incoming.markNewestBuffer();
  }
}

uint8_t DMXInterface::Message::checkCRC() {
  crc c;

  auto data = raw;

  for (uint8_t i = 0; i < length; i++) {
    c << *data++;
  }

  return c.getCRC();
}

void DMXInterface::sendByte(const uint8_t c) {
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  UDR0 = c;
}

// Yes, we're including the .cpp
#include <TripleBuffer.cpp>
template class libCameron::TripleBuffer<DMXInterface::Message, true>;
