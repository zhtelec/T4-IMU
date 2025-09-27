#define _PACKET_CPP_

#include        <stdint.h>
#include        <stdlib.h>

#include        "packet.h"

void
PacketCalcSumAndFillHeader(struct _stPacketGeneric *p, int len)
{
  p->hdr.magic      = PACKET_HEADER;
  p->hdr.len        = len;
  p->hdr.sum        = 0;
  p->hdr.reserved1  = 0;
  p->hdr.reserved2  = 0;

  p->hdr.sum = PacketCalcSum8((uint8_t *)p, len);

  return;
}


uint8_t
PacketCalcSum8(const uint8_t *p, int len)
{
  uint8_t       sum = 0;

  for(int i = 0; i < len; i++) {
    sum += *p++;
  }

  return 0 - sum;
}
