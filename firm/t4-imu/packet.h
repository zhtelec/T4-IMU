#ifndef _PACKET_H_
#define _PACKET_H_

struct _stPacketHeader {
  uint16_t              magic;          // 0x5dac
#define PACKET_HEADER           0x5dac
  uint16_t              len;            // total packet size
  uint8_t               sum;            // calc all byte (include sum) is 0
  uint8_t               type;           // 1: imu data S16, 2: imu data float
#define PACKET_TYPE_IMU_S16             0x11
#define PACKET_TYPE_IMU_FLOAT           0x12
#define PACKET_TYPE_GNSS_PPS            0x20
#define PACKET_TYPE_GNSS_INT64          0x21
#define PACKET_TYPE_XXX                 0xf0

  uint16_t              reserved1;

  uint8_t               id;             // device id
  uint8_t               seq;            // seq  ++1 every send (the value is independent of each device)
  uint8_t               reserved2;      //
  uint8_t               tai_secH;       // TAI ts_sec of [39:32]
  uint32_t              tai_sec;        // TAI ts_sec of [31: 0]
  int32_t               tai_nsec;       // nano sec
  uint32_t              ts_1MHz;        // internal timestamp in MHz
};


struct _stPacketGeneric {
  struct _stPacketHeader        hdr;
  uint32_t                      buf[1];
};


// the packet byte order is little endian
struct _stPacketImuS16 {
  struct _stPacketHeader        hdr;
  uint16_t                      acc[3];
  uint16_t                      gyr[3];
  uint16_t                      temp;
  uint16_t                      tsChip;
};


// the packet byte order is little endian
struct _stPacketImuFloat {
  struct _stPacketHeader        hdr;
  float                         acc[3];
  float                         gyr[3];
  float                         temp;
  uint32_t                      tsChip;
};


struct _stPacketGnssPps {
  struct _stPacketHeader        hdr;
  uint64_t                      epochtime;      // calc from GPS UTC
  uint64_t                      tai;
};


struct _stPacketGnssPvt {
  struct _stPacketHeader        hdr;
  uint64_t                      epochtime;      // calc from GPS UTC

  uint32_t                      iTOW;           // unit ms
  uint16_t                      year;
  uint8_t                       month;
  uint8_t                       day;

  uint8_t                       hour;
  uint8_t                       min;
  uint8_t                       sec;
  uint8_t                       valid;
  int32_t                       tAcc;           // unit ns    time accuracy

  int32_t                       nano;
  uint8_t                       fixType;        // 0:NoFix, 1:DeadReckoningOnly, 2:2dFix, 3:3dFix, 5:TimeOnly
  uint8_t                       flags;
  uint8_t                       flags2;
  uint8_t                       numSV;          // pcs

  int64_t                       lat;            // unit 10^-9 deg

  int64_t                       lon;            // unit 10^-9 deg

  int32_t                       height;         // unit 0.1mm
  int32_t                       hMSL;           // unit 0.1mm  hMSL

  int32_t                       hAcc;           // unit 0.1mm    horizontal accuracy
  int32_t                       vAcc;           // unit 0.1mm    vertical accuracy

  int32_t                       velN;           // unit mm/s  to N is plus
  int32_t                       velE;           // unit mm/s  to E is plus

  int32_t                       velD;           // unit mm/s  to down is plus
  int32_t                       gSpeed;         // unit mm/s

  int32_t                       headMot;        // unit 10^-6 deg
  int32_t                       sAcc;           // unit mm/s  speed accuracy

  int32_t                       headAcc;        // unit 10^-6 deg   head accuracy
  int32_t                       pDOP;

  int32_t                       flags3;
  int32_t                       headVeh;        // unit 10^-6 deg

  int32_t                       magDec;
  int32_t                       magAcc;
};


void            PacketCalcSumAndFillHeader(struct _stPacketGeneric *p, int len);
uint8_t         PacketCalcSum8(const uint8_t *p, int len);


#endif
