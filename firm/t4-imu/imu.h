#ifndef _IMU_H_
#define _IMU_H_


// the packet byte order is little endian
#if 0
struct _stImuFloat {
  uint8_t               header;         // 0x55
  uint8_t               len;            // total packet size
  uint8_t               sum;
  uint8_t               type;           // packet type

  uint8_t               id;             // imu id
  uint8_t               seq;            // seq  ++1 every send
  uint8_t               reserved2;
  uint8_t               ts_secH;        // ts_sec of [32:39]
  uint32_t              ts_sec;         // ts_sec of [31: 0]
  uint32_t              ts_nsec;        // nano sec
  uint32_t              ts_1MHz;        // internal timestamp in MHz
  float                 acc[3];
  float                 gyr[3];
};
#endif


void            lower_trigger(void);


void            ImuInit(void);
void            ImuLoop(void);
void            ImuStart(void);
void            ImuCommand(int ac, char *av[]);


#ifdef _IMU_CPP_

static void     ImuSendValue(struct _stImuValue *p);

#endif

#endif
