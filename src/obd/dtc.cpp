#include "dtc.h"

namespace roadnode {
namespace obd {

void formatDtc(uint8_t hi, uint8_t lo, char out[6]) {
  static const char sys[4] = {'P', 'C', 'B', 'U'};
  static const char hex[] = "0123456789ABCDEF";
  out[0] = sys[hi >> 6];
  out[1] = (char)('0' + ((hi >> 4) & 0x03));
  out[2] = hex[hi & 0x0F];
  out[3] = hex[lo >> 4];
  out[4] = hex[lo & 0x0F];
  out[5] = 0;
}

bool decodeDtcs(const uint8_t* data, size_t len, DtcList& out) {
  out.count = 0;
  size_t ofs = 0;
  size_t declared = (size_t)-1;
  if (len & 1) {  // count byte present
    declared = data[0];
    ofs = 1;
    if (declared * 2 > len - 1) return false;
  }
  for (; ofs + 1 < len && out.count < DtcList::MAX; ofs += 2) {
    if (declared != (size_t)-1 && ofs >= 1 + declared * 2) break;
    if (data[ofs] == 0 && data[ofs + 1] == 0) continue;  // padding
    out.codes[out.count].raw = (uint16_t)((data[ofs] << 8) | data[ofs + 1]);
    formatDtc(data[ofs], data[ofs + 1], out.codes[out.count].code);
    out.count++;
  }
  return true;
}

}  // namespace obd
}  // namespace roadnode
