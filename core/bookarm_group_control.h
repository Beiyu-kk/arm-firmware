#pragma once

const s16 BOOKARM_GRIPPER_OPEN_POS = 2047;
const s16 BOOKARM_GRIPPER_CLOSE_POS = 3150;
const u8 BOOKARM_ALL_MOTOR_COUNT = 7;
const u8 BOOKARM_ARM_ROD_MOTOR_COUNT = 6;
const u8 BOOKARM_SYNC_FEEDBACK_LEN = SMS_STS_PRESENT_CURRENT_H - SMS_STS_PRESENT_POSITION_L + 1;
bool bookArmGripperGripHold = false;

u8 BookArm_resolveAllMotorsTorqueCmd(u8 requestedCMD) {
  requestedCMD = requestedCMD ? 1 : 0;
#if ARM_FORCE_TORQUE_LOCK_ALWAYS_ON
  return 1;
#else
  if (bookArmGripperGripHold && requestedCMD == 0) {
    return 1;
  }
  return requestedCMD;
#endif
}

void BookArm_fillAllMotorIds(u8* ids) {
  ids[0] = BASE_SERVO_ID;
  ids[1] = SHOULDER_DRIVING_SERVO_ID;
  ids[2] = SHOULDER_DRIVEN_SERVO_ID;
  ids[3] = ELBOW_SERVO_ID;
  ids[4] = GRIPPER_SERVO_ID;
  ids[5] = EXT_ROD_SERVO_ID;
  ids[6] = EXT_GRIPPER_SERVO_ID;
}

void BookArm_fillArmRodMotorIds(u8* ids) {
  ids[0] = BASE_SERVO_ID;
  ids[1] = SHOULDER_DRIVING_SERVO_ID;
  ids[2] = SHOULDER_DRIVEN_SERVO_ID;
  ids[3] = ELBOW_SERVO_ID;
  ids[4] = GRIPPER_SERVO_ID;
  ids[5] = EXT_ROD_SERVO_ID;
}

void BookArm_stopConstantMotion() {
  const_cmd_base_x = MOVE_STOP;
  const_cmd_shoulder_y = MOVE_STOP;
  const_cmd_elbow_z = MOVE_STOP;
  const_cmd_eoat_t = MOVE_STOP;
  const_cmd_rod = MOVE_STOP;
}

const char* BookArm_gripperStateName(int pos) {
  return pos <= ((BOOKARM_GRIPPER_OPEN_POS + BOOKARM_GRIPPER_CLOSE_POS) / 2) ? "open" : "closed";
}

u8 BookArm_gripperCloseFromCommand(JsonDocument& doc) {
  JsonVariant value;
  if (doc.containsKey("gripper")) {
    value = doc["gripper"];
  } else if (doc.containsKey("g")) {
    value = doc["g"];
  } else if (doc.containsKey("cmd")) {
    value = doc["cmd"];
  } else {
    return 0;
  }

  if (value.is<const char*>()) {
    const char* state = value.as<const char*>();
    return (strcmp(state, "closed") == 0 || strcmp(state, "close") == 0 || strcmp(state, "1") == 0 || strcmp(state, "true") == 0) ? 1 : 0;
  }
  return value.as<int>() != 0 ? 1 : 0;
}

int BookArm_wordLE(u8* data, u8 offset) {
  return data[offset] | (data[offset + 1] << 8);
}

int BookArm_wordBE(u8* data, u8 offset) {
  return (data[offset] << 8) | data[offset + 1];
}

int BookArm_signedFlag(int value, u8 negBit) {
  if (negBit && (value & (1 << negBit))) {
    return -(value & ~(1 << negBit));
  }
  return value;
}

void BookArm_putWordLE(u8* data, u8 offset, u16 value) {
  data[offset] = value & 0xff;
  data[offset + 1] = (value >> 8) & 0xff;
}

void BookArm_putWordBE(u8* data, u8 offset, u16 value) {
  data[offset] = (value >> 8) & 0xff;
  data[offset + 1] = value & 0xff;
}

void BookArm_parseStsFeedback(byte servoID, u8* data) {
  int feedbackIndex = servoFeedbackIndex(servoID);
  if (feedbackIndex < 0) {
    return;
  }
  servoFeedback[feedbackIndex].status = true;
  servoFeedback[feedbackIndex].pos = BookArm_signedFlag(BookArm_wordLE(data, 0), 15);
  servoFeedback[feedbackIndex].speed = BookArm_signedFlag(BookArm_wordLE(data, 2), 15);
  servoFeedback[feedbackIndex].load = BookArm_signedFlag(BookArm_wordLE(data, 4), 10);
  servoFeedback[feedbackIndex].voltage = data[6];
  servoFeedback[feedbackIndex].temper = data[7];
  servoFeedback[feedbackIndex].current = BookArm_signedFlag(BookArm_wordLE(data, 13), 15);
}

void BookArm_parseRodFeedback(u8* data) {
  externalRodFeedback.status = true;
  externalRodFeedback.pos = BookArm_signedFlag(BookArm_wordLE(data, 0), 15);
  externalRodFeedback.speed = BookArm_signedFlag(BookArm_wordLE(data, 2), 15);
  externalRodFeedback.load = BookArm_signedFlag(BookArm_wordLE(data, 4), 10);
  externalRodFeedback.voltage = data[6];
  externalRodFeedback.temper = data[7];
  externalRodFeedback.current = BookArm_signedFlag(BookArm_wordLE(data, 13), 15);
}

void BookArm_parseGripperFeedback(u8* data) {
  externalGripperFeedback.status = true;
  externalGripperFeedback.pos = BookArm_signedFlag(BookArm_wordLE(data, 0), 15);
  externalGripperFeedback.speed = BookArm_signedFlag(BookArm_wordLE(data, 2), 15);
  externalGripperFeedback.load = BookArm_signedFlag(BookArm_wordLE(data, 4), 10);
  externalGripperFeedback.voltage = data[6];
  externalGripperFeedback.temper = data[7];
  externalGripperFeedback.current = BookArm_signedFlag(BookArm_wordLE(data, 13), 15);
}

bool BookArm_syncReadAllMotorFeedback() {
  u8 ids[BOOKARM_ARM_ROD_MOTOR_COUNT];
  BookArm_fillArmRodMotorIds(ids);

  if (!SharedBus_take("arm rod sync feedback")) {
    return false;
  }

  if (st.pSerial) {
    while (st.pSerial->read() != -1) {}
  }
  st.syncReadPacketTx(ids, BOOKARM_ARM_ROD_MOTOR_COUNT, SMS_STS_PRESENT_POSITION_L, BOOKARM_SYNC_FEEDBACK_LEN);

  bool ok = true;
  for (u8 i = 0; i < BOOKARM_ARM_ROD_MOTOR_COUNT; i++) {
    u8 data[BOOKARM_SYNC_FEEDBACK_LEN];
    bool gotPacket = st.syncReadPacketRx(ids[i], data) == BOOKARM_SYNC_FEEDBACK_LEN;
    ok = ok && gotPacket;
    if (!gotPacket) {
      if (ids[i] >= BASE_SERVO_ID && ids[i] <= GRIPPER_SERVO_ID) {
        int feedbackIndex = servoFeedbackIndex(ids[i]);
        if (feedbackIndex >= 0) {
          servoFeedback[feedbackIndex].status = false;
        }
      } else if (ids[i] == EXT_ROD_SERVO_ID) {
        externalRodFeedback.status = false;
      }
      continue;
    }

    if (ids[i] >= BASE_SERVO_ID && ids[i] <= GRIPPER_SERVO_ID) {
      BookArm_parseStsFeedback(ids[i], data);
    } else if (ids[i] == EXT_ROD_SERVO_ID) {
      BookArm_parseRodFeedback(data);
    }
  }

  SharedBus_release(SHARED_BUS_READ_QUIET_MS);
  bool gripperOk = ExternalGripper_getFeedback(true);
  ok = ok && gripperOk;

  radB = calculateRadByFeedback(servoFeedback[BASE_SERVO_ID - BASE_SERVO_ID].pos, BASE_JOINT);
  radS = calculateRadByFeedback(servoFeedback[SHOULDER_DRIVING_SERVO_ID - BASE_SERVO_ID].pos, SHOULDER_JOINT);
  radE = calculateRadByFeedback(servoFeedback[ELBOW_SERVO_ID - BASE_SERVO_ID].pos, ELBOW_JOINT);
  radG = calculateRadByFeedback(servoFeedback[GRIPPER_SERVO_ID - BASE_SERVO_ID].pos, EOAT_JOINT);
  RoArmM2_computePosbyJointRad(radB, radS, radE, radG);
  if (EEMode == 0) {
    lastT = radG;
  }

  return ok;
}

bool BookArm_syncWriteAllMotorPositions(
  u8* ids,
  s16* positions,
  u16 speedInput,
  u8 accInput,
  u16 gripperTorqueInput
) {
  (void)gripperTorqueInput;
  u8 data[7 * BOOKARM_ARM_ROD_MOTOR_COUNT];
  for (u8 i = 0; i < BOOKARM_ARM_ROD_MOTOR_COUNT; i++) {
    u8 offset = i * 7;
    u16 pos = constrain(positions[i], 0, ARM_SERVO_POS_RANGE - 1);
    data[offset] = accInput;
    BookArm_putWordLE(data, offset + 1, pos);
    BookArm_putWordLE(data, offset + 3, 0);
    BookArm_putWordLE(data, offset + 5, speedInput);
  }

  if (!SharedBus_take("arm rod sync write")) {
    return false;
  }
  st.syncWrite(ids, BOOKARM_ARM_ROD_MOTOR_COUNT, SMS_STS_ACC, data, 7);
  SharedBus_release(SHARED_BUS_ARM_WRITE_QUIET_MS);
  return true;
}

bool BookArm_syncWriteAllMotorsTorque(u8 enableCMD, const char* owner) {
  enableCMD = BookArm_resolveAllMotorsTorqueCmd(enableCMD);
  u8 ids[BOOKARM_ARM_ROD_MOTOR_COUNT];
  u8 data[BOOKARM_ARM_ROD_MOTOR_COUNT];
  BookArm_fillArmRodMotorIds(ids);
  for (u8 i = 0; i < BOOKARM_ARM_ROD_MOTOR_COUNT; i++) {
    data[i] = enableCMD;
  }

  if (!SharedBus_take(owner)) {
    return false;
  }
  st.syncWrite(ids, BOOKARM_ARM_ROD_MOTOR_COUNT, SMS_STS_TORQUE_ENABLE, data, 1);
  RoArmM2_torqueLock = (enableCMD != 0);
  if (RoArmM2_torqueLock) {
    RoArmM2_torqueGuardEnabled = true;
  }
  SharedBus_release(SHARED_BUS_ARM_WRITE_QUIET_MS);
  bool gripperOk = ExternalGripper_torqueCtrl(enableCMD);
  return gripperOk;
}

bool BookArm_syncWriteAllMotorsServoMode(const char* owner) {
  u8 ids[BOOKARM_ARM_ROD_MOTOR_COUNT];
  u8 data[BOOKARM_ARM_ROD_MOTOR_COUNT];
  BookArm_fillArmRodMotorIds(ids);
  for (u8 i = 0; i < BOOKARM_ARM_ROD_MOTOR_COUNT; i++) {
    data[i] = 0;
  }

  if (!SharedBus_take(owner)) {
    return false;
  }
  st.syncWrite(ids, BOOKARM_ARM_ROD_MOTOR_COUNT, SMS_STS_MODE, data, 1);
  SharedBus_release(SHARED_BUS_ARM_WRITE_QUIET_MS);
  return true;
}

bool BookArm_holdAllMotorsTorqueLock(bool force = false) {
  if (!RoArmM2_torqueLock || !RoArmM2_torqueGuardEnabled) {
    return false;
  }

  static unsigned long lastTorqueGuardMs = 0;
  unsigned long nowMs = millis();
  if (!force && (long)(sharedBusQuietUntil - nowMs) > 0) {
    return true;
  }
  if (!force && nowMs - lastTorqueGuardMs < ARM_TORQUE_GUARD_INTERVAL_MS) {
    return true;
  }

  bool ok = BookArm_syncWriteAllMotorsTorque(1, "all motors torque guard");
  if (ok) {
    lastTorqueGuardMs = nowMs;
  }
  return ok;
}

bool BookArm_syncFiveJointsGripperRad(
  double inputBase,
  double inputShoulder,
  double inputElbow,
  double inputHand,
  double inputRod,
  u8 gripperClose,
  u16 speedDegPerSec,
  u8 accDegPerSec2,
  u16 gripperTorqueInput,
  u16 rodTorqueInput
) {
#if ARM_FORCE_MAX_TORQUE_LIMIT_ALWAYS_ON
  gripperTorqueInput = ST_TORQUE_MAX;
  rodTorqueInput = ST_TORQUE_MAX;
#endif
  externalGripperRequestedTorque = constrain(gripperTorqueInput, ST_TORQUE_MIN, ST_TORQUE_MAX);
  externalRodRequestedTorque = constrain(rodTorqueInput, ST_TORQUE_MIN, ST_TORQUE_MAX);
  BookArm_stopConstantMotion();

  bool armOk = BookArm_syncAllJointsRad(
    inputBase,
    inputShoulder,
    inputElbow,
    inputHand,
    inputRod,
    speedDegPerSec,
    accDegPerSec2,
    externalRodRequestedTorque
  );

  s16 gripperTarget = gripperClose ? BOOKARM_GRIPPER_CLOSE_POS : BOOKARM_GRIPPER_OPEN_POS;
  s16 positions[7] = {
    goalPos[0],
    goalPos[1],
    goalPos[2],
    goalPos[3],
    goalPos[4],
    externalRodAngleToPos(extRodGoalAngleDeg),
    gripperTarget
  };

  bool gripperOk = gripperClose ?
    ExternalGripper_close(speedDegPerSec, accDegPerSec2, externalGripperRequestedTorque) :
    ExternalGripper_open(speedDegPerSec, accDegPerSec2, externalGripperRequestedTorque);
  bool ok = armOk && gripperOk;
  if (ok) {
    ExternalGripper_pauseArmConstant(SHARED_BUS_FUSION_CMD_QUIET_MS);
    ExternalRod_pauseArmConstant(SHARED_BUS_FUSION_CMD_QUIET_MS);
    unsigned long quietUntil = millis() + SHARED_BUS_FUSION_CMD_QUIET_MS;
    if ((long)(quietUntil - sharedBusQuietUntil) > 0) {
      sharedBusQuietUntil = quietUntil;
    }
  }
  if (gripperOk) {
    bookArmGripperGripHold = (gripperClose != 0);
  }

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = 1431;
  jsonInfoHttp["ok"] = ok;
  jsonInfoHttp["arm"] = armOk;
  jsonInfoHttp["gripper_ok"] = gripperOk;
  jsonInfoHttp["gripper"] = gripperClose ? "closed" : "open";
  jsonInfoHttp["gripper_hold"] = bookArmGripperGripHold;
  JsonArray posOut = jsonInfoHttp.createNestedArray("pos");
  for (u8 i = 0; i < 7; i++) {
    posOut.add(positions[i]);
  }
  serializeJson(jsonInfoHttp, Serial);
  Serial.println();

  return ok;
}

bool BookArm_allMotorsTorqueCtrl(u8 enableCMD) {
  u8 requestedCMD = enableCMD ? 1 : 0;
  enableCMD = BookArm_resolveAllMotorsTorqueCmd(requestedCMD);

  if (!BookArm_syncWriteAllMotorsTorque(enableCMD, "all motors torque")) {
    jsonInfoHttp.clear();
    jsonInfoHttp["T"] = 1421;
    jsonInfoHttp["ok"] = false;
    jsonInfoHttp["error"] = "servo bus busy";
    serializeJson(jsonInfoHttp, Serial);
    Serial.println();
    return false;
  }

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = 1421;
  jsonInfoHttp["ok"] = true;
  jsonInfoHttp["requested"] = requestedCMD;
  jsonInfoHttp["cmd"] = enableCMD;
  jsonInfoHttp["gripper_hold"] = bookArmGripperGripHold;
  jsonInfoHttp["ids"] = "11,12,13,14,15,16,1";
  serializeJson(jsonInfoHttp, Serial);
  Serial.println();
  return true;
}

void BookArm_allMotorsStateFeedback() {
  bool ok = BookArm_syncReadAllMotorFeedback();

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = 1411;
  jsonInfoHttp["ok"] = ok;

  JsonArray jointsRad = jsonInfoHttp.createNestedArray("joints_rad");
  jointsRad.add(radB);
  jointsRad.add(radS);
  jointsRad.add(radE);
  jointsRad.add(lastT);
  jointsRad.add(externalRodPosToAngle(externalRodFeedback.pos) * M_PI / 180.0);

  JsonArray jointsTorque = jsonInfoHttp.createNestedArray("joints_torque");
  jointsTorque.add(servoFeedback[BASE_SERVO_ID - BASE_SERVO_ID].load);
  jointsTorque.add(servoFeedback[SHOULDER_DRIVING_SERVO_ID - BASE_SERVO_ID].load -
                   servoFeedback[SHOULDER_DRIVEN_SERVO_ID - BASE_SERVO_ID].load);
  jointsTorque.add(servoFeedback[ELBOW_SERVO_ID - BASE_SERVO_ID].load);
  jointsTorque.add(servoFeedback[GRIPPER_SERVO_ID - BASE_SERVO_ID].load);
  jointsTorque.add(externalRodFeedback.load);

  JsonArray rawPos = jsonInfoHttp.createNestedArray("raw_pos");
  rawPos.add(servoFeedback[BASE_SERVO_ID - BASE_SERVO_ID].pos);
  rawPos.add(servoFeedback[SHOULDER_DRIVING_SERVO_ID - BASE_SERVO_ID].pos);
  rawPos.add(servoFeedback[SHOULDER_DRIVEN_SERVO_ID - BASE_SERVO_ID].pos);
  rawPos.add(servoFeedback[ELBOW_SERVO_ID - BASE_SERVO_ID].pos);
  rawPos.add(servoFeedback[GRIPPER_SERVO_ID - BASE_SERVO_ID].pos);
  rawPos.add(externalRodFeedback.pos);
  rawPos.add(externalGripperFeedback.pos);

  JsonArray online = jsonInfoHttp.createNestedArray("online");
  online.add(servoFeedback[BASE_SERVO_ID - BASE_SERVO_ID].status);
  online.add(servoFeedback[SHOULDER_DRIVING_SERVO_ID - BASE_SERVO_ID].status);
  online.add(servoFeedback[SHOULDER_DRIVEN_SERVO_ID - BASE_SERVO_ID].status);
  online.add(servoFeedback[ELBOW_SERVO_ID - BASE_SERVO_ID].status);
  online.add(servoFeedback[GRIPPER_SERVO_ID - BASE_SERVO_ID].status);
  online.add(externalRodFeedback.status);
  online.add(externalGripperFeedback.status);

  jsonInfoHttp["gripper_state"] = BookArm_gripperStateName(externalGripperFeedback.pos);
  jsonInfoHttp["gripper_pos"] = externalGripperFeedback.pos;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}
