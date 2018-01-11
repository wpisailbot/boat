#pragma once
#include <inttypes.h>

namespace sailbot {
namespace can {

struct CANID {
  CANID() : __reserved(0), __unused(0), __eff_ident(1) {}
  /* CANID structure:
   * It will generally be represented as a 32 bit number when using SocketCAN.
   * Highest/First 3 bits meaningless (CANIDs are only 29 bits).
   * Next 3 bits priority
   * Next 2 reserved
   * Next byte is higher order byte of PGN or, if <240, the whole PGN.
   * Next byte is either lower order byte of PGN or destination address.
   * Next/Last byte is source address.
   */
  // Note that due to endianness, byte order is reversed from above.
  uint8_t source;
  uint8_t PS;
  uint8_t PF;
  uint8_t DP : 1;
  uint8_t __reserved : 1;
  uint8_t priority : 3;
  uint8_t __unused : 2; // Technically, flags for SocketCAN
  uint8_t __eff_ident : 1; // Tells SocketCAN we are using 29 bit frames
};

uint32_t ConstructID(CANID can_id);

CANID RetrieveID(uint32_t raw_id);

uint32_t GetPGN(CANID id);

void SetPGN(CANID *id, int32_t pgn);

}  // namespace sailbot
}  // namespace can
