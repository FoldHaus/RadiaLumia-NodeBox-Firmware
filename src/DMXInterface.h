

#ifndef DMXINTERFACE_H
#define DMXINTERFACE_H

#include <stddef.h>
#include <TripleBuffer.h>
#include <avr/interrupt.h>

#include <CRC8.h>
#include <DecPrintFormatter.h>

#include "Debug.h"
#include "util.h"

ISR(USART_RX_vect);

namespace FoldHaus {

/**
 * Internal block of bytes we're wrapping around
 */
typedef struct {
  uint8_t command;
  uint16_t motor;
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

    static constexpr uint8_t MARKER = 0xAA;

    /**
     * fix length of message, including crc
     */
    static constexpr size_t length = sizeof(MARKER) + sizeof(DMXDataShape) + sizeof(crc);

    union {
      uint8_t raw[length];
      struct {
        uint8_t marker;
        DMXDataShape data;
      };
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

    inline bool hasValidMarker() {
      return marker == MARKER;
    }

    inline typeof(data.command) getCommand() const {
      return data.command;
    }

    inline typeof(data.motor) getMotor() const {
      return data.motor;
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

    const auto msg = incoming.getReadBuffer();
    const auto crc = msg->checkCRC();

    if (Debug::DMX::CRC) {
      if (crc) {
        DMXInterface::debug << PSTR("Computed CRC: ") << crc << endl;
      } else {
        DMXInterface::debug << PSTR("CRC pass!") << endl;
      }
    }

    const bool markerValid = msg->hasValidMarker();

    return markerValid && crc == 0;
  }

  inline static Message const * getMessage() {
    return isMessageReady() ? incoming.getReadBuffer() : nullptr;
  }

private:
  static TripleBuffer<Message, true> incoming;

};

}


#endif
