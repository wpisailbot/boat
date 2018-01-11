#include "can/pgn.h"

namespace sailbot {
namespace can {

uint32_t ConstructID(CANID can_id) {
  // TODO(james): Invalid cast (changes alignment). Switch to union or the such.
  return *(uint32_t*)&can_id;
}

CANID RetrieveID(uint32_t raw_id) {
  return *(CANID*)&raw_id;
}

uint32_t GetPGN(CANID id) {
  return ((uint32_t)id.DP << 16) + ((uint32_t)id.PF << 8) +
         (id.PF < 240 ? 0 : id.PS);
}

void SetPGN(CANID *id, int32_t pgn) {
  uint8_t low = pgn & 0xFF;
  pgn >>= 8;
  id->PF = pgn & 0xFF;
  pgn >>= 8;
  id->DP = pgn & 0x01;
  if (id->PF >= 240) {
    id->PS = low;
  }
}

}  // namespace sailbot
}  // namespace can
