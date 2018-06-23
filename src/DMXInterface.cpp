
#include "DMXInterface.h"
#include "Arduino.h"

#include "Board.h"

#include <avr/io.h>

#include <CRC8.h>

using namespace Foldhaus;
using namespace libCameron;
using namespace Board;

libCameron::TripleBuffer<DMXInterface::Message, true> DMXInterface::incoming;

DecPrintFormatter DMXInterface::debug(&DMXInterface::sendByte);

static CRC8 CRC;

void USART_RX_vect() {
  // DebugPin goes on during interrupt handing to easily see visual timing of interrupt servicing
  DebugPin::on();
  DMXInterface::receiveByte();
  DebugPin::off();
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
  // -3 : Init
  // -2 : Wait - Byte seen. Waiting for frame error which indicates a "break".
  // -1 : Packet Start Byte
  // Otherwise current channel number. 0 is first channel. 511 is last channel
  static int channel = -3;

  // UCSRnA must be read before UDRn or data is invalid
  const uint8_t status = UCSR0A;
  const uint8_t incomingByte = UDR0;

  if (channel == -3) {
    // Seen our first byte, so mark it
    channel = -2;
    return;
  }

  // If we're waiting for frame error that marks start of packet
  if (channel == -2) {
    // Check for "break"
    if ((status & (1 << FE0)) && incomingByte == 0) {
      channel = -1;
    }
    return;
  }

  // If we get a frame error during the normal course of operation, new packet!
  if (channel > 24 && status & 1 << FE0) {
    channel = -1;
    return;
  }

  // If we get a data overrun (or frame error too early) wait for another start of DMX packet
  if (status & (1 << FE0 | 1 << DOR0)) {
    channel = -2;
    return;
  }

  // DMX packet type. Always 0
  if (channel == -1) {
    channel = incomingByte ? -2 : 0;
    return;
  }

  incoming.getWriteBuffer()->feed(channel, incomingByte);

  channel++;
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
