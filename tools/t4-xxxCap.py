#!/usr/bin/env python3

import socket
import struct
import time
import os
import sys
import signal
import argparse
from datetime import datetime, timezone

# Parameters
toutSaveInterval     = 60

paramMulticastAddr   = "239.0.0.1"
paramMulticastPort   = 23901
paramMulticastIfAddr = "10.0.0.28"

PKT_HEADER           = 0x5dac
PKT_TYPE_IMU_FLOAT   = 0x12
PKT_TYPE_MAGNETIC_FLOAT   = 0x18
PKT_TYPE_GNSS_PPS    = 0x20
PKT_TYPE_GNSS_PVT    = 0x21
PKT_TYPE_XXX         = 0xf0

cntImu               = {}
cntMagnetic          = {}
cntGnssPps           = {}
cntGnssPvt           = {}

seqPrev              = {}


dirRoot       = 't4-xxxCapture'
dirSubDateText = ''
dirDataDate    = ''
datePrev       = ''
fnameImu       = 't4-xxx_imu.txt'
fnameImu       = 't4-xxx_magnetic.txt'
fnameGnss      = 't4-xxx_gnss.txt'

optionQuiet    = False
optionNoSave   = False
optionCount    = False
optionCheckSeq = False
optionVerbose  = False

sizeStrageMax  = 384 * 1024 * 1024 * 1024

# Global state
multicastRecvBuf = b""
strImu           = ""
strMagnetic      = ""
strGnss          = ""
parsePacketTypeImuDataNsec = 0

# UDP socket
udps = None


def GetStreamLen(sock, n):
    global multicastRecvBuf
    while len(multicastRecvBuf) < n:
        data = sock.recv(65536)
        multicastRecvBuf += data
    out = multicastRecvBuf[:n]
    multicastRecvBuf = multicastRecvBuf[n:]
    return out


def CalcSum(data, sumv):
    for b in data:
        sumv += b
    return sumv & 0xff

def CheckSeq(idv, seq):
    if idv in seqPrev:
        if(((seqPrev[idv]+1) & 0xff) != seq):
            print(f"# seq fail id: {idv}, seq: {seq:02x}, seqPrev: {seqPrev[idv]:02x}")
    seqPrev[idv] = seq


def ParsePacketTypeImuDataFloat(data):
    global parsePacketTypeImuDataNsec

    header, length, sumv, ptype, rsv1                    = struct.unpack_from("<HHBBH",     data,  0)  # packet header
    idv, seq, rsv2, ts_secH, ts_sec, ts_nsec, ts_int1MHz = struct.unpack_from("<BBBBLLL",   data,  8)  # data header
    axf, ayf, azf, gxf, gyf, gzf,  tempf, tsChip         = struct.unpack_from("<fffffffL",  data, 24)  # data

    sec40 = (ts_secH << 32) + ts_sec

    s  = f"{ptype:02x} {idv} {seq:02x} {sec40}.{ts_nsec:09d} {ts_int1MHz:08x}"
    s += f" {axf:0.8f} {ayf:0.8f} {azf:0.8f} {gxf:0.8f} {gyf:0.8f} {gzf:0.8f}"
    s += f" {tempf:0.2f} {tsChip:08x}\n"

    update = False
    if (sec40 % 60) == 0 and parsePacketTypeImuDataNsec > ts_nsec:
        update = True
    parsePacketTypeImuDataNsec = ts_nsec

    cntImu[idv] = cntImu.get(idv, 0) + 1

    if(optionCheckSeq):
        CheckSeq(idv, seq)


    return s, update


def ParsePacketTypeMagneticDataFloat(data):
    global parsePacketTypeMagneticDataNsec

    header, length, sumv, ptype, rsv1                    = struct.unpack_from("<HHBBH",     data,  0)  # packet header
    idv, seq, rsv2, ts_secH, ts_sec, ts_nsec, ts_int1MHz = struct.unpack_from("<BBBBLLL",   data,  8)  # data header
    mxf, myf, mzf, nxf, nyf, nzf,  tempf, tsChip         = struct.unpack_from("<fffffffL",  data, 24)  # data

    sec40 = (ts_secH << 32) + ts_sec

    s  = f"{ptype:02x} {idv} {seq:02x} {sec40}.{ts_nsec:09d} {ts_int1MHz:08x}"
    s += f" {mxf:0.8f} {myf:0.8f} {mzf:0.8f}"
    s += f" {tempf:0.2f} {tsChip:08x}\n"

    update = False
    if (sec40 % 60) == 0 and parsePacketTypeMagneticDataNsec > ts_nsec:
        update = True
    parsePacketTypeMagneticDataNsec = ts_nsec

    cntMagnetic[idv] = cntMagnetic.get(idv, 0) + 1

    if(optionCheckSeq):
        CheckSeq(idv, seq)


    return s, update


def ParsePacketTypeGnssDataPps(data):
    global parsePacketTypeImuDataNsec

    update = False

    header, length, sumv, ptype, rsv1                    = struct.unpack_from("<HHBBH",    data,   0)  # packet header
    idv, seq, rsv2, ts_secH, ts_sec, ts_nsec, ts_int1MHz = struct.unpack_from("<BBBBLLL",  data,   8)  # data header
    [tEpochUtc]                                          = struct.unpack_from("<L",        data,  24)  # data

    sec40 = (ts_secH << 32) + ts_sec

    s  = f"{ptype:02x} {idv} {seq:02x} {sec40}.{ts_nsec:09d} {ts_int1MHz:08x} {tEpochUtc}\n"

    return s, update


def ParsePacketTypeGnssDataPvt(data):
    global parsePacketTypeImuDataNsec

    update = False

    header, length, sumv, ptype, rsv1                    = struct.unpack_from("<HHBBH",    data,   0)  # packet header
    idv, seq, rsv2, ts_secH, ts_sec, ts_nsec, ts_int1MHz = struct.unpack_from("<BBBBLLL",  data,   8)  # data header

    tUtc, iTOW, year, month, day, hour, minutes, sec, valid = struct.unpack_from("<QLHBBBBBB",  data,  24)  # data
    tAcc, nano, fix_type, flag, flag2, numSV                = struct.unpack_from("<LlBBBB",     data,  44)
    latitude, longitude, height, hMSL, hAcc, vAcc           = struct.unpack_from("<QQllLL",     data,  56)
    velN, velE, velD, gSpeed, headMot, sAcc, headAcc        = struct.unpack_from("<lllLLLL",    data,  88)
    pDOP, flag3, headVeh, magDec, magAcc                    = struct.unpack_from("<LLLLL",      data, 116)

    sec40 = (ts_secH << 32) + ts_sec

    s  = f"{ptype:02x} {idv} {seq:02x} {sec40}.{ts_nsec:09d} {ts_int1MHz:08x}"
    s += f" {tUtc} {iTOW:x} {year:04d}{month:02d}{day:02d}{hour:02d}{minutes:02d}{sec:02d} {valid:x}"
    s += f" {tAcc} {nano} {fix_type} {flag:x} {flag2:x} {numSV}"
    s += f" {latitude} {longitude} {height} {hMSL} {hAcc} {vAcc}"
    s += f" {velN} {velE} {velD} {gSpeed} {headMot} {sAcc} {headAcc}"
    s += f" {pDOP} {flag3} {headVeh} {magDec} {magAcc}\n"

    update = False
    if (sec40 % 60) == 0 and parsePacketTypeImuDataNsec > ts_nsec:
        update = True
    parsePacketTypeImuDataNsec = ts_nsec

    return s, update


def NetworkInit():
    global udps
    udps = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    udps.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udps.bind(("0.0.0.0", paramMulticastPort))

    mreq = socket.inet_aton(paramMulticastAddr) + socket.inet_aton(paramMulticastIfAddr)
    udps.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)


def StoreRecvData(s, ddir, fname):
    if not optionNoSave:
        if len(s) > 0:
            os.makedirs(ddir, exist_ok=True)
            path = os.path.join(ddir, fname)
            with open(path, "a+") as fd:
                fd.write(s)


def SaveImu():
    global strImu
    if len(strImu) > 0:
        StoreRecvData(strImu, dirDataDate, fnameImu)
        strImu = ""


def SaveMagnetic():
    global strMagnetic
    if len(strMagnetic) > 0:
        StoreRecvData(strMagnetic, dirDataDate, fnameMagnetic)
        strMagnetic = ""


def SaveGnss():
    global strGnss
    if len(strGnss) > 0:
        StoreRecvData(strGnss, dirDataDate, fnameGnss)
        strGnss = ""


def SaveAll():
    SaveImu()
    SaveGnss()


def ExitRoutine(sig=None, frame=None):
    SaveAll()
    SaveGnss()
    sys.exit(0)


def handle_signals():
    for s in [signal.SIGINT, signal.SIGTERM, signal.SIGHUP,
              signal.SIGQUIT, signal.SIGUSR1, signal.SIGUSR2]:
        signal.signal(s, ExitRoutine)


def parse_options():
    global dirDataDate, optionNoSave, optionCount, optionCheckSeq, optionQuiet, optionVerbose
    global cntImu

    parser = argparse.ArgumentParser()
    parser.add_argument("--datadir", dest="datadir", type=str)
    parser.add_argument("--nosave", action="store_true")
    parser.add_argument("--count", action="store_true")
    parser.add_argument("--checkseq", action="store_true")
    parser.add_argument("-q", action="store_true")
    parser.add_argument("--verbose", action="store_true")

    args = parser.parse_args()
    if args.datadir:
        dirDataDate = args.datadir
    if args.nosave:
        optionNoSave = True
    if args.count:
        optionCount = True
    if args.checkseq:
        optionCheckSeq = True
    if args.q:
        optionQuiet = True
    if args.verbose:
        optionVerbose = True


def main():
    global dirSubDateText, dirDataDate, datePrev, strImu, strGnss
    t1sec = 0;

    handle_signals()

    os.makedirs(dirRoot, exist_ok=True)
#    dirSubDateText = datetime.utcnow().strftime("%Y%m%d_%H%M")
    dirSubDateText = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M")
    dirDataDate = os.path.join(dirRoot, dirSubDateText)
    datePrevDigit = 8
    datePrev = dirSubDateText[0:datePrevDigit]

    parse_options()
    NetworkInit()

    if not optionQuiet:
        print(f'  the data store directory is "{dirDataDate}"', file=sys.stderr)

    tUpdate = int(time.time())

    try:
        while True:
            udps.settimeout(0.1)
            try:
                c = GetStreamLen(udps, 1)
            except socket.timeout:
                c = None

            if c:
                if c[0] != (PKT_HEADER & 0xff):
                    continue
                c2 = GetStreamLen(udps, 1)
                if c2[0] != (PKT_HEADER >> 8):
                    continue

                data = struct.pack("<H", PKT_HEADER) + GetStreamLen(udps, 4)
                header, length, sumv, ptype = struct.unpack("<HHBB", data)
                remain = GetStreamLen(udps, length - 6)
                data += remain
                sm = CalcSum(data, 0)

                if header == PKT_HEADER and length == len(data) and sm == 0:
                    if ptype == PKT_TYPE_IMU_FLOAT:
                        s, update = ParsePacketTypeImuDataFloat(data)
                        strImu += s
                        if update:
                            SaveImu()
                        if optionVerbose and s != "":
                            print(s, end="")

                    elif ptype == PKT_TYPE_MAGNETIC_FLOAT:
                        s, update = ParsePacketTypeMagneticDataFloat(data)
                        strImu += s
                        if update:
                            SaveMagnetic()
                        if optionVerbose and s != "":
                            print(s, end="")

                    elif ptype == PKT_TYPE_GNSS_PPS:
                        s, update = ParsePacketTypeGnssDataPps(data)
                        strGnss += s
                        if optionVerbose and s != "":
                            print(s, end="")

                    elif ptype == PKT_TYPE_GNSS_PVT:
                        s, update = ParsePacketTypeGnssDataPvt(data)
                        strGnss += s
                        if update:
                            SaveGnss()
                        if optionVerbose and s != "":
                            print(s, end="")

#                    elif ptype == PKT_TYPE_XXX:
#                        pass
                    else:
                         print(f"unknown type[{ptype:x}]\n");

            # check directory rollover
            dateUtc = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M")
            if datePrev != dateUtc[0:datePrevDigit]:
                SaveAll()
                dirSubDateText = dateUtc
                dirDataDate = os.path.join(dirRoot, dirSubDateText)
                if not optionQuiet:
                    print(f'the store directory was changed to "{dirDataDate}"')
                datePrev = dateUtc[0:datePrevDigit]

            if (time.time() - tUpdate) >= 60:
                tUpdate = int(time.time())

            if((t1sec - int(-time.time()*1000)) > 999):
                t1sec = int(-time.time()*1000)
                if(optionCount):
                    print('\n\n\n\n\n\n\n\n\n')
                    for key in sorted(cntImu):
                        print(f'IMU{key}: {cntImu.get(key, 0)}')

    except KeyboardInterrupt:
        udps.close()


if __name__ == "__main__":
    main()
