#ifndef BOOK_ARM_RAW_POSITION_CALIBRATION_H
#define BOOK_ARM_RAW_POSITION_CALIBRATION_H

const byte BOOKARM_CALIBRATION_SERVO_COUNT = 7;

bool BookArm_validRawPosition(int pos) {
  return pos >= 0 && pos < ARM_SERVO_POS_RANGE;
}

bool BookArm_rawPositionWithin(int pos, s16 target, u16 tolerance) {
  return BookArm_validRawPosition(pos) && abs(pos - target) <= tolerance;
}

bool BookArm_readAllCalibrationRawPositions(int posOut[BOOKARM_CALIBRATION_SERVO_COUNT], bool statusOut[BOOKARM_CALIBRATION_SERVO_COUNT]) {
  byte armIds[5] = {
    BASE_SERVO_ID,
    SHOULDER_DRIVING_SERVO_ID,
    SHOULDER_DRIVEN_SERVO_ID,
    ELBOW_SERVO_ID,
    GRIPPER_SERVO_ID
  };

  bool allReadable = true;
  for (byte i = 0; i < 5; i++) {
    statusOut[i] = getFeedback(armIds[i], true);
    int feedbackIndex = servoFeedbackIndex(armIds[i]);
    posOut[i] = feedbackIndex >= 0 ? servoFeedback[feedbackIndex].pos : -1;
    if (!statusOut[i] || !BookArm_validRawPosition(posOut[i])) {
      allReadable = false;
    }
  }

  statusOut[5] = ExternalGripper_getFeedback(true);
  posOut[5] = externalGripperFeedback.pos;
  if (!statusOut[5] || !BookArm_validRawPosition(posOut[5])) {
    allReadable = false;
  }

  statusOut[6] = ExternalRod_getFeedback(true);
  posOut[6] = externalRodFeedback.pos;
  if (!statusOut[6] || !BookArm_validRawPosition(posOut[6])) {
    allReadable = false;
  }

  return allReadable;
}

bool BookArm_publishRawFeedback(s16 target, u16 tolerance) {
  int pos[BOOKARM_CALIBRATION_SERVO_COUNT];
  bool status[BOOKARM_CALIBRATION_SERVO_COUNT];
  bool readable = BookArm_readAllCalibrationRawPositions(pos, status);

  bool ok = readable;
  for (byte i = 0; i < BOOKARM_CALIBRATION_SERVO_COUNT; i++) {
    if (!BookArm_rawPositionWithin(pos[i], target, tolerance)) {
      ok = false;
    }
  }

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = 5051;
  jsonInfoHttp["target"] = target;
  jsonInfoHttp["tol"] = tolerance;
  jsonInfoHttp["ok"] = ok;
  jsonInfoHttp["p11"] = pos[0];
  jsonInfoHttp["p12"] = pos[1];
  jsonInfoHttp["p13"] = pos[2];
  jsonInfoHttp["p14"] = pos[3];
  jsonInfoHttp["p15"] = pos[4];
  jsonInfoHttp["p1"] = pos[5];
  jsonInfoHttp["p16"] = pos[6];
  serializeJson(jsonInfoHttp, Serial);
  Serial.println();
  return ok;
}

bool BookArm_moveAllCalibrationServosRaw(s16 inputPos, u16 speedInput, u8 accInput) {
  s16 rawPos = constrain(inputPos, 0, ARM_SERVO_POS_RANGE - 1);
  u16 rawSpeed = speedInput == 0 ? ARM_SERVO_INIT_SPEED : speedInput;
  u8 rawAcc = accInput == 0 ? ARM_SERVO_INIT_ACC : accInput;

  u8 armIds[5] = {
    BASE_SERVO_ID,
    SHOULDER_DRIVING_SERVO_ID,
    SHOULDER_DRIVEN_SERVO_ID,
    ELBOW_SERVO_ID,
    GRIPPER_SERVO_ID
  };
  s16 armPos[5] = {rawPos, rawPos, rawPos, rawPos, rawPos};
  u16 armSpeed[5] = {rawSpeed, rawSpeed, rawSpeed, rawSpeed, rawSpeed};
  u8 armAcc[5] = {rawAcc, rawAcc, rawAcc, rawAcc, rawAcc};

  bool armOk = RoArmM2_syncWritePosEx(armIds, 5, armPos, armSpeed, armAcc);

  bool gripperOk = false;
  if (ExternalGripper_takeBus("raw gripper pos")) {
    externalGripperBus.ServoMode(EXT_GRIPPER_SERVO_ID);
    externalGripperBus.WritePosEx(
      EXT_GRIPPER_SERVO_ID,
      rawPos,
      rawSpeed,
      rawAcc,
      externalGripperRequestedTorque
    );
    ExternalGripper_pauseArmConstant();
    ExternalGripper_releaseBus(SHARED_BUS_GRIPPER_CMD_QUIET_MS);
    gripperOk = true;
  }

  bool rodOk = false;
  ExternalRod_setServoMode(EXT_ROD_SERVO_ID);
  if (ExternalRod_takeBus("raw rod pos")) {
    externalRodBus.WritePosEx(EXT_ROD_SERVO_ID, rawPos, rawSpeed, rawAcc);
    ExternalRod_pauseArmConstant();
    ExternalRod_releaseBus(SHARED_BUS_ROD_CMD_QUIET_MS);
    rodOk = true;
  }

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = 5041;
  jsonInfoHttp["pos"] = rawPos;
  jsonInfoHttp["spd"] = rawSpeed;
  jsonInfoHttp["acc"] = rawAcc;
  jsonInfoHttp["arm"] = armOk;
  jsonInfoHttp["external_gripper"] = gripperOk;
  jsonInfoHttp["external_rod"] = rodOk;
  jsonInfoHttp["ids"] = "11,12,13,14,15,1,16";
  serializeJson(jsonInfoHttp, Serial);
  Serial.println();

  return armOk && gripperOk && rodOk;
}

byte BookArm_servoIdFromJointAlias(byte jointInput) {
  switch (jointInput) {
    case BASE_JOINT:
      return BASE_SERVO_ID;
    case SHOULDER_JOINT:
      return SHOULDER_DRIVING_SERVO_ID;
    case ELBOW_JOINT:
      return ELBOW_SERVO_ID;
    case EOAT_JOINT:
      return GRIPPER_SERVO_ID;
    case EXT_ROD_JOINT:
      return EXT_ROD_SERVO_ID;
    case EXT_GRIPPER_JOINT:
      return EXT_GRIPPER_SERVO_ID;
    default:
      return 0;
  }
}

bool BookArm_moveSingleServoRaw(byte servoID, s16 inputPos, u16 speedInput, u8 accInput, u16 torqueInput) {
  s16 rawPos = constrain(inputPos, 0, ARM_SERVO_POS_RANGE - 1);
  u16 rawSpeed = speedInput == 0 ? ARM_SERVO_INIT_SPEED : speedInput;
  u8 rawAcc = accInput == 0 ? ARM_SERVO_INIT_ACC : accInput;
  bool ok = false;
  const char* target = "unknown";

  if (servoID >= BASE_SERVO_ID && servoID <= GRIPPER_SERVO_ID) {
    target = "arm";
    ok = RoArmM2_writePosEx(servoID, rawPos, rawSpeed, rawAcc);
  } else if (servoID == EXT_GRIPPER_SERVO_ID) {
    target = "external_gripper";
    ok = ExternalGripper_moveToPos(rawPos, rawSpeed, rawAcc, torqueInput);
  } else if (servoID == EXT_ROD_SERVO_ID) {
    target = "external_rod";
    ExternalRod_setServoMode(EXT_ROD_SERVO_ID);
    ok = ExternalRod_moveToPos(rawPos, rawSpeed, rawAcc, torqueInput);
  }
  RoArmM2_holdTorqueLock(true);

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = 5081;
  jsonInfoHttp["ok"] = ok;
  jsonInfoHttp["id"] = servoID;
  jsonInfoHttp["target"] = target;
  jsonInfoHttp["pos"] = rawPos;
  jsonInfoHttp["spd"] = rawSpeed;
  jsonInfoHttp["acc"] = rawAcc;
  serializeJson(jsonInfoHttp, Serial);
  Serial.println();

  return ok;
}

byte BookArm_servoIdFromRawCommand(JsonDocument& doc) {
  if (doc.containsKey("id")) {
    return doc["id"].as<byte>();
  }
  return BookArm_servoIdFromJointAlias(doc["joint"] | 0);
}

bool BookArm_moveAllCalibrationServosRawVerified(s16 inputPos, u16 speedInput, u8 accInput, u16 tolerance, unsigned long timeoutMs) {
  s16 rawPos = constrain(inputPos, 0, ARM_SERVO_POS_RANGE - 1);
  u16 rawTolerance = tolerance == 0 ? 1 : tolerance;
  unsigned long rawTimeoutMs = timeoutMs == 0 ? 8000 : timeoutMs;

  BookArm_moveAllCalibrationServosRaw(rawPos, speedInput, accInput);

  unsigned long startedMs = millis();
  bool ok = false;
  int pos[BOOKARM_CALIBRATION_SERVO_COUNT];
  bool status[BOOKARM_CALIBRATION_SERVO_COUNT];
  do {
    delay(250);
    ok = BookArm_readAllCalibrationRawPositions(pos, status);
    for (byte i = 0; i < BOOKARM_CALIBRATION_SERVO_COUNT; i++) {
      if (!BookArm_rawPositionWithin(pos[i], rawPos, rawTolerance)) {
        ok = false;
      }
    }
    if (ok) {
      break;
    }
  } while (millis() - startedMs < rawTimeoutMs);

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = 5061;
  jsonInfoHttp["target"] = rawPos;
  jsonInfoHttp["tol"] = rawTolerance;
  jsonInfoHttp["ok"] = ok;
  jsonInfoHttp["p11"] = pos[0];
  jsonInfoHttp["p12"] = pos[1];
  jsonInfoHttp["p13"] = pos[2];
  jsonInfoHttp["p14"] = pos[3];
  jsonInfoHttp["p15"] = pos[4];
  jsonInfoHttp["p1"] = pos[5];
  jsonInfoHttp["p16"] = pos[6];
  serializeJson(jsonInfoHttp, Serial);
  Serial.println();

  return ok;
}

void BookArm_publishJointZeroMap() {
  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = 5071;
  jsonInfoHttp["base_zero"] = BASE_ZERO_POS;
  jsonInfoHttp["base_dir"] = BASE_POS_DIRECTION;
  jsonInfoHttp["shoulder_drive_zero"] = SHOULDER_DRIVING_ZERO_POS;
  jsonInfoHttp["shoulder_drive_dir"] = SHOULDER_DRIVING_POS_DIRECTION;
  jsonInfoHttp["shoulder_driven_zero"] = SHOULDER_DRIVEN_ZERO_POS;
  jsonInfoHttp["shoulder_driven_dir"] = SHOULDER_DRIVEN_POS_DIRECTION;
  jsonInfoHttp["elbow_zero"] = ELBOW_ZERO_POS;
  jsonInfoHttp["elbow_dir"] = ELBOW_POS_DIRECTION;
  jsonInfoHttp["gripper_zero"] = GRIPPER_ZERO_POS;
  jsonInfoHttp["gripper_dir"] = GRIPPER_POS_DIRECTION;
  jsonInfoHttp["rod_zero"] = EXT_ROD_ZERO_POS;
  jsonInfoHttp["rod_dir"] = EXT_ROD_POS_DIRECTION;
  serializeJson(jsonInfoHttp, Serial);
  Serial.println();
}

#endif
