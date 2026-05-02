#pragma once

#include <SCServo.h>

#define CF35_TORQUE_ENABLE 40
#define CF35_MODE 33
#define CF35_ACC 41
#define CF35_GOAL_POSITION_L 42
#define CF35_GOAL_TORQUE_L 44
#define CF35_GOAL_SPEED_L 46
#define CF35_TORQUE_LIMIT_L 48
#define CF35_LOCK 55
#define CF35_PRESENT_POSITION_L 56
#define CF35_PRESENT_SPEED_L 58
#define CF35_PRESENT_LOAD_L 60
#define CF35_PRESENT_VOLTAGE 62
#define CF35_PRESENT_TEMPERATURE 63
#define CF35_PRESENT_CURRENT_L 69
#define CF35_PRESENT_CURRENT_H 70

// Keep the symbol used by the official Gripper_CF example.
#ifndef RSBL_ST_TORQUE_LIMIT_L
#define RSBL_ST_TORQUE_LIMIT_L CF35_TORQUE_LIMIT_L
#endif

class BookArm_CFSCL : public SCSerial {
public:
  BookArm_CFSCL() : SCSerial(0) {}
  BookArm_CFSCL(u8 End) : SCSerial(End) {}
  BookArm_CFSCL(u8 End, u8 Level) : SCSerial(End, Level) {}

  int WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC, u16 Torque) {
    u8 bBuf[7];
    u16 posWord = signedWord(Position, 15);
    bBuf[0] = ACC;
    Host2SCS(bBuf + 1, bBuf + 2, posWord);
    Host2SCS(bBuf + 3, bBuf + 4, Torque);
    Host2SCS(bBuf + 5, bBuf + 6, Speed);
    return genWrite(ID, CF35_ACC, bBuf, 7);
  }

  int RegWritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC, u16 Torque) {
    u8 bBuf[7];
    u16 posWord = signedWord(Position, 15);
    bBuf[0] = ACC;
    Host2SCS(bBuf + 1, bBuf + 2, posWord);
    Host2SCS(bBuf + 3, bBuf + 4, Torque);
    Host2SCS(bBuf + 5, bBuf + 6, Speed);
    return regWrite(ID, CF35_ACC, bBuf, 7);
  }

  void SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[], u16 Torque[]) {
    u8 offbuf[7 * IDN];
    for (u8 i = 0; i < IDN; i++) {
      offbuf[i * 7] = ACC ? ACC[i] : 0;
      Host2SCS(offbuf + i * 7 + 1, offbuf + i * 7 + 2, signedWord(Position[i], 15));
      Host2SCS(offbuf + i * 7 + 3, offbuf + i * 7 + 4, Torque ? Torque[i] : 0);
      Host2SCS(offbuf + i * 7 + 5, offbuf + i * 7 + 6, Speed ? Speed[i] : 0);
    }
    syncWrite(ID, IDN, CF35_ACC, offbuf, 7);
  }

  int WriteSpe(u8 ID, s16 Speed, u8 ACC, u16 Torque) {
    u8 bBuf[5];
    bBuf[0] = ACC;
    Host2SCS(bBuf + 1, bBuf + 2, Torque);
    Host2SCS(bBuf + 3, bBuf + 4, signedWord(Speed, 15));
    return genWrite(ID, CF35_ACC, bBuf, 5);
  }

  int ServoMode(u8 ID) {
    return writeByte(ID, CF35_MODE, 0);
  }

  int EleMode(u8 ID) {
    return writeByte(ID, CF35_MODE, 2);
  }

  int WriteEle(u8 ID, s16 Torque) {
    return writeWord(ID, CF35_GOAL_TORQUE_L, signedWord(Torque, 15));
  }

  int EnableTorque(u8 ID, u8 Enable) {
    return writeByte(ID, CF35_TORQUE_ENABLE, Enable);
  }

  int unLockEprom(u8 ID) {
    EnableTorque(ID, 0);
    return writeByte(ID, CF35_LOCK, 0);
  }

  int LockEprom(u8 ID) {
    return writeByte(ID, CF35_LOCK, 1);
  }

  int CalibrationOfs(u8 ID) {
    return writeByte(ID, CF35_TORQUE_ENABLE, 128);
  }

  int FeedBack(int ID) {
    int nLen = Read(ID, CF35_PRESENT_POSITION_L, Mem, sizeof(Mem));
    if (nLen != sizeof(Mem)) {
      Err = 1;
      return -1;
    }
    Err = 0;
    return nLen;
  }

  int ReadPos(int ID) {
    return readSignedWord(ID, CF35_PRESENT_POSITION_L, 0, 15);
  }

  int ReadSpeed(int ID) {
    return readSignedWord(ID, CF35_PRESENT_SPEED_L, 2, 15);
  }

  int ReadLoad(int ID) {
    return readSignedWord(ID, CF35_PRESENT_LOAD_L, 4, 10);
  }

  int ReadVoltage(int ID) {
    if (ID == -1) {
      return Mem[CF35_PRESENT_VOLTAGE - CF35_PRESENT_POSITION_L];
    }
    Err = 0;
    int value = readByte(ID, CF35_PRESENT_VOLTAGE);
    if (value == -1) {
      Err = 1;
    }
    return value;
  }

  int ReadTemper(int ID) {
    if (ID == -1) {
      return Mem[CF35_PRESENT_TEMPERATURE - CF35_PRESENT_POSITION_L];
    }
    Err = 0;
    int value = readByte(ID, CF35_PRESENT_TEMPERATURE);
    if (value == -1) {
      Err = 1;
    }
    return value;
  }

  int ReadCurrent(int ID) {
    return readSignedWord(ID, CF35_PRESENT_CURRENT_L, 13, 15);
  }

  int ReadMode(int ID) {
    Err = 0;
    int mode = readByte(ID, CF35_MODE);
    if (mode == -1) {
      Err = 1;
    }
    return mode;
  }

private:
  u8 Mem[CF35_PRESENT_CURRENT_H - CF35_PRESENT_POSITION_L + 1];

  u16 signedWord(s16 value, u8 negBit) {
    if (value < 0) {
      return ((u16)(-value)) | (1 << negBit);
    }
    return (u16)value;
  }

  int decodeSignedWord(int value, u8 negBit) {
    if (value == -1) {
      return -1;
    }
    if (negBit && (value & (1 << negBit))) {
      return -(value & ~(1 << negBit));
    }
    return value;
  }

  int readMemWord(u8 offset) {
    return SCS2Host(Mem[offset], Mem[offset + 1]);
  }

  int readSignedWord(int ID, u8 memAddr, u8 memOffset, u8 negBit) {
    int value = -1;
    if (ID == -1) {
      value = readMemWord(memOffset);
    } else {
      Err = 0;
      value = readWord(ID, memAddr);
      if (value == -1) {
        Err = 1;
        return -1;
      }
    }
    return decodeSignedWord(value, negBit);
  }
};
