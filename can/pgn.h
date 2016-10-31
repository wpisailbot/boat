#pragma once
#include <inttypes.h>

struct CANID {
  /* CANID structure:
   * It will generally be represented as a 32 bit number when using SocketCAN.
   * Highest/First 3 bits meaningless (CANIDs are only 29 bits).
   * Next 3 bits priority
   * Next 2 reserved
   * Next byte is higher order byte of PGN or, if <240, the whole PGN.
   * Next byte is either lower order byte of PGN or destination address.
   * Next/Last byte is source address.
   */
  uint8_t __unused : 3;
  uint8_t priority : 3;
  uint8_t __reserved : 2;
  uint8_t dataA;
  uint8_t dataB;
  uint8_t source;
};

union CAN_ID_converter {
  CANID structured;
  uint32_t raw;
};

uint32_t ConstructID(CANID can_id) {
  return *(uint32_t*)&can_id;
}

CANID RetrieveID(uint32_t raw_id) {
  return *(CANID*)&raw_id;
}

