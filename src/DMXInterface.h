

#ifndef DMXINTERFACE_H
#define DMXINTERFACE_H

#include <stddef.h>
#include <TripleBuffer.h>
#include <avr/interrupt.h>

#include <CRC8.h>
#include <DecPrintFormatter.h>

ISR(USART_RX_vect);

namespace FoldHaus {

/**
 * Internal block of bytes we're wrapping around
 */
typedef struct {
  uint32_t command :24;
  uint8_t pinspot;
} DMXDataShape;

using namespace libCameron;

class DMXInterface {
  static constexpr unsigned int dmxOffset = 0;

  friend void ::USART_RX_vect();
  inline static void receiveByte() __attribute__((always_inline, hot));
public:
  static void init();
  class Message {
    using crc = CRC8;
    friend class DMXInterface;

    /**
     * fix length of message, including crc
     */
    static constexpr size_t length = sizeof(DMXDataShape) + sizeof(crc);

    union {
      uint8_t raw[length];
      DMXDataShape data;
    };

    /**
     * Feed a new byte to the message parser
     * @param channel the byte number in the packet we're on
     * @param b the byte to parse
     */
    void feed(uint16_t channel, uint8_t b);
  public:
    /**
     * Returns the CRC8 of the full message. A value other than 0 means CRC failed.
     */
    uint8_t checkCRC();

    inline typeof(data.command) getCommand() const {
      return data.command;
    }

    inline typeof(data.pinspot) getPinspot() const {
      return data.pinspot;
    }
  };

private:
 static void sendByte(uint8_t const);

public:
 static libCameron::DecPrintFormatter debug;

  inline static bool isMessageReady() {
    if (!incoming.isNewData()) return false;

    incoming.reserveNewestBufferForReading();

    // HACK: This line make sure the CRC is 0xAA
    return incoming.getReadBuffer()->raw[sizeof(DMXDataShape)] == 0xAA;

    return (incoming.getReadBuffer()->checkCRC() == 0);
  }

  inline static Message const * getMessage() {
    return isMessageReady() ? incoming.getReadBuffer() : nullptr;
  }

private:
  static TripleBuffer<Message, true> incoming;

};

}


#endif
