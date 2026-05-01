#pragma once

#include <SCServo.h>

SMS_STS externalRodBus;

bool ExternalRod_takeBus(const char* owner) {
#if EXT_ROD_USE_ARM_BUS
  return SharedBus_take(owner);
#else
  return true;
#endif
}

void ExternalRod_releaseBus(uint32_t quietMs) {
#if EXT_ROD_USE_ARM_BUS
  SharedBus_release(quietMs);
#else
  (void)quietMs;
#endif
}

void ExternalRod_pauseArmConstant(uint32_t pauseMs = SHARED_BUS_ROD_CONSTANT_PAUSE_MS) {
#if EXT_ROD_USE_ARM_BUS
  SharedBus_pauseConstantMotion(pauseMs);
#else
  (void)pauseMs;
#endif
}

struct ExternalRodFeedback {
  bool status;
  int pos;
  int speed;
  int load;
  float voltage;
  float current;
  float temper;
  byte mode;
};

ExternalRodFeedback externalRodFeedback = {false, 0, 0, 0, 0, 0, 0, 0};
bool externalRodOnline = false;
int externalRodRequestedTorque = EXT_ROD_DEFAULT_TORQUE;
bool externalRodTorqueWarned = false;
bool externalRodPositionModeActive = true;

s16 externalRodAngleToPos(double inputAng) {
  double limitedAng = constrain(inputAng, EXT_ROD_MIN_ANGLE_DEG, EXT_ROD_MAX_ANGLE_DEG);
  double pos = EXT_ROD_ZERO_POS + EXT_ROD_POS_DIRECTION * ((limitedAng / 360.0) * ARM_SERVO_POS_RANGE);
  return constrain((s16)round(pos), 0, ARM_SERVO_POS_RANGE - 1);
}

double externalRodPosToAngle(int inputPos) {
  if (EXT_ROD_POS_DIRECTION == 0 || inputPos < 0 || inputPos >= ARM_SERVO_POS_RANGE) {
    return 0;
  }
  return (inputPos - EXT_ROD_ZERO_POS) * 360.0 / (EXT_ROD_POS_DIRECTION * ARM_SERVO_POS_RANGE);
}

void ExternalRod_servoInit() {
#if EXT_ROD_USE_ARM_BUS
  externalRodBus.pSerial = &Serial1;
  if (InfoPrint == 1) {
    Serial.println("External rod ST3215 uses RoArm bus servo UART.");
  }
#else
  Serial2.begin(1000000, SERIAL_8N1, EXT_ROD_RXD, EXT_ROD_TXD);
  externalRodBus.pSerial = &Serial2;
  delay(20);
  if (InfoPrint == 1) {
    Serial.println("External rod ST3215 init UART2TTL...");
  }
#endif
}

bool ExternalRod_getFeedback(bool quiet = false) {
  if (!ExternalRod_takeBus("rod feedback")) {
    externalRodFeedback.status = false;
    externalRodOnline = false;
    return false;
  }

  bool feedbackOk = externalRodBus.FeedBack(EXT_ROD_SERVO_ID) != -1;
  if (feedbackOk) {
    externalRodFeedback.status = true;
    externalRodFeedback.pos = externalRodBus.ReadPos(-1);
    externalRodFeedback.speed = externalRodBus.ReadSpeed(-1);
    externalRodFeedback.load = externalRodBus.ReadLoad(-1);
    externalRodFeedback.voltage = externalRodBus.ReadVoltage(-1);
    externalRodFeedback.current = externalRodBus.ReadCurrent(-1);
    externalRodFeedback.temper = externalRodBus.ReadTemper(-1);
    externalRodFeedback.mode = externalRodBus.ReadMode(EXT_ROD_SERVO_ID);
    externalRodOnline = true;
    ExternalRod_releaseBus(SHARED_BUS_READ_QUIET_MS);
    return true;
  }

  ExternalRod_releaseBus(SHARED_BUS_READ_QUIET_MS);
  externalRodFeedback.status = false;
  externalRodOnline = false;
  if (!quiet && InfoPrint == 1) {
    Serial.print("External rod servo ID:");
    Serial.print(EXT_ROD_SERVO_ID);
    Serial.println(" status: failed.");
  }
  return false;
}

void ExternalRod_initCheck() {
  if (ExternalRod_getFeedback(true)) {
    if (InfoPrint == 1) {
      Serial.println("External rod ST3215 status checked.");
    }
  } else if (InfoPrint == 1) {
    Serial.println("External rod ST3215 is offline, arm and gripper features remain available.");
  }
}

void ExternalRod_torqueCtrl(u8 enableCMD) {
#if ARM_FORCE_TORQUE_LOCK_ALWAYS_ON
  enableCMD = 1;
#endif
  if (!ExternalRod_takeBus("rod torque")) {
    return;
  }
  externalRodBus.EnableTorque(EXT_ROD_SERVO_ID, enableCMD);
  ExternalRod_pauseArmConstant();
  ExternalRod_releaseBus(SHARED_BUS_ROD_CMD_QUIET_MS);
  RoArmM2_restoreTorqueLock();
}

void ExternalRod_setMiddle(byte servoID) {
  if (!ExternalRod_takeBus("rod set middle")) {
    return;
  }
  externalRodBus.CalibrationOfs(servoID);
  ExternalRod_pauseArmConstant();
  ExternalRod_releaseBus(SHARED_BUS_ROD_CMD_QUIET_MS);
  RoArmM2_restoreTorqueLock();
}

void ExternalRod_setServoMode(byte servoID = EXT_ROD_SERVO_ID) {
  if (!ExternalRod_takeBus("rod servo mode")) {
    return;
  }
  externalRodBus.unLockEprom(servoID);
  externalRodBus.writeByte(servoID, SMS_STS_MODE, 0);
  externalRodBus.LockEprom(servoID);
  if (servoID == EXT_ROD_SERVO_ID) {
    externalRodPositionModeActive = true;
  }
  ExternalRod_pauseArmConstant();
  ExternalRod_releaseBus(SHARED_BUS_ROD_CMD_QUIET_MS);
  RoArmM2_restoreTorqueLock();
}

void ExternalRod_setMotorMode(byte servoID = EXT_ROD_SERVO_ID) {
  if (!ExternalRod_takeBus("rod motor mode")) {
    return;
  }
  externalRodBus.unLockEprom(servoID);
  externalRodBus.WheelMode(servoID);
  externalRodBus.LockEprom(servoID);
  if (servoID == EXT_ROD_SERVO_ID) {
    externalRodPositionModeActive = false;
  }
  ExternalRod_pauseArmConstant();
  ExternalRod_releaseBus(SHARED_BUS_ROD_CMD_QUIET_MS);
  RoArmM2_restoreTorqueLock();
}

void ExternalRod_motorSpeed(s16 speedInput, u8 accInput) {
  if (!ExternalRod_takeBus("rod motor speed")) {
    return;
  }
  externalRodBus.WriteSpe(EXT_ROD_SERVO_ID, constrain(speedInput, -EXT_ROD_MAX_MOTOR_SPEED, EXT_ROD_MAX_MOTOR_SPEED), accInput);
  ExternalRod_pauseArmConstant();
  ExternalRod_releaseBus(SHARED_BUS_ROD_CMD_QUIET_MS);
  RoArmM2_restoreTorqueLock();
}

void ExternalRod_torqueLimit(int inputTorque) {
#if ARM_FORCE_MAX_TORQUE_LIMIT_ALWAYS_ON
  inputTorque = ST_TORQUE_MAX;
#endif
  externalRodRequestedTorque = constrain(inputTorque, ST_TORQUE_MIN, ST_TORQUE_MAX);
  if (!externalRodTorqueWarned && InfoPrint == 1) {
    Serial.println("Note: public SCServo library accepts rod torque requests, but ST3215 torque-limit behavior depends on the servo firmware/register support.");
    externalRodTorqueWarned = true;
  }
}

void ExternalRod_dynamicAdaptation(byte inputMode, int inputTorque) {
  if (inputMode == 0) {
    ExternalRod_torqueLimit(ST_TORQUE_MAX);
  } else if (inputMode == 1) {
    ExternalRod_torqueLimit(inputTorque);
  }
}

bool BookArm_syncAllJointsRad(
  double inputBase,
  double inputShoulder,
  double inputElbow,
  double inputHand,
  double inputRod,
  u16 speedDegPerSec,
  u8 accDegPerSec2,
  u16 rodTorqueInput
) {
#if ARM_FORCE_MAX_TORQUE_LIMIT_ALWAYS_ON
  rodTorqueInput = ST_TORQUE_MAX;
#endif
  externalRodRequestedTorque = constrain(rodTorqueInput, ST_TORQUE_MIN, ST_TORQUE_MAX);
  extRodGoalAngleDeg = constrain(inputRod * 180.0 / M_PI, EXT_ROD_MIN_ANGLE_DEG, EXT_ROD_MAX_ANGLE_DEG);
  if (!externalRodPositionModeActive) {
    ExternalRod_setServoMode(EXT_ROD_SERVO_ID);
  }

  RoArmM2_baseJointCtrlRad(0, inputBase, 0, 0);
  RoArmM2_shoulderJointCtrlRad(0, inputShoulder, 0, 0);
  RoArmM2_elbowJointCtrlRad(0, inputElbow, 0, 0);
  RoArmM2_handJointCtrlRad(0, inputHand, 0, 0);

  u8 ids[6] = {
    BASE_SERVO_ID,
    SHOULDER_DRIVING_SERVO_ID,
    SHOULDER_DRIVEN_SERVO_ID,
    ELBOW_SERVO_ID,
    GRIPPER_SERVO_ID,
    EXT_ROD_SERVO_ID
  };
  s16 positions[6] = {
    goalPos[0],
    goalPos[1],
    goalPos[2],
    goalPos[3],
    goalPos[4],
    externalRodAngleToPos(extRodGoalAngleDeg)
  };
  u16 servoSpd = angleSpeedToServoSteps(speedDegPerSec);
  u8 servoAcc = angleAccToServoAcc(accDegPerSec2);
  u16 speeds[6] = {servoSpd, servoSpd, servoSpd, servoSpd, servoSpd, servoSpd};
  u8 accs[6] = {servoAcc, servoAcc, servoAcc, servoAcc, servoAcc, servoAcc};

  return RoArmM2_syncWritePosEx(ids, 6, positions, speeds, accs);
}

void ExternalRod_moveToAngle(double inputAng, u16 speedDegPerSec, u8 accDegPerSec2, u16 torqueInput) {
#if ARM_FORCE_MAX_TORQUE_LIMIT_ALWAYS_ON
  torqueInput = ST_TORQUE_MAX;
#endif
  extRodGoalAngleDeg = constrain(inputAng, EXT_ROD_MIN_ANGLE_DEG, EXT_ROD_MAX_ANGLE_DEG);
  externalRodRequestedTorque = constrain(torqueInput, ST_TORQUE_MIN, ST_TORQUE_MAX);
  if (!externalRodPositionModeActive) {
    ExternalRod_setServoMode(EXT_ROD_SERVO_ID);
  }
  if (!ExternalRod_takeBus("rod angle")) {
    return;
  }
  externalRodBus.WritePosEx(
    EXT_ROD_SERVO_ID,
    externalRodAngleToPos(extRodGoalAngleDeg),
    angleSpeedToServoSteps(speedDegPerSec),
    angleAccToServoAcc(accDegPerSec2)
  );
  ExternalRod_pauseArmConstant();
  ExternalRod_releaseBus(SHARED_BUS_ROD_CMD_QUIET_MS);
  RoArmM2_restoreTorqueLock();
}

bool ExternalRod_moveToPos(s16 posInput, u16 speedInput, u8 accInput, u16 torqueInput) {
#if ARM_FORCE_MAX_TORQUE_LIMIT_ALWAYS_ON
  torqueInput = ST_TORQUE_MAX;
#endif
  extRodGoalAngleDeg = externalRodPosToAngle(constrain(posInput, 0, ARM_SERVO_POS_RANGE - 1));
  externalRodRequestedTorque = constrain(torqueInput, ST_TORQUE_MIN, ST_TORQUE_MAX);
  if (!externalRodPositionModeActive) {
    ExternalRod_setServoMode(EXT_ROD_SERVO_ID);
  }
  if (!ExternalRod_takeBus("rod raw pos")) {
    return false;
  }
  externalRodBus.WritePosEx(
    EXT_ROD_SERVO_ID,
    constrain(posInput, 0, ARM_SERVO_POS_RANGE - 1),
    speedInput,
    accInput
  );
  ExternalRod_pauseArmConstant();
  ExternalRod_releaseBus(SHARED_BUS_ROD_CMD_QUIET_MS);
  RoArmM2_restoreTorqueLock();
  return true;
}

void ExternalRod_retract(u16 speedInput, u8 accInput, u16 torqueInput) {
  ExternalRod_moveToAngle(EXT_ROD_RETRACT_ANGLE_DEG, speedInput, accInput, torqueInput);
}

void ExternalRod_extend(u16 speedInput, u8 accInput, u16 torqueInput) {
  ExternalRod_moveToAngle(EXT_ROD_EXTEND_ANGLE_DEG, speedInput, accInput, torqueInput);
}

void Fusion_jointsGripperRodAngleCtrl(
  double inputBase,
  double inputShoulder,
  double inputElbow,
  double inputHand,
  double inputGripper,
  double inputRod,
  u16 inputSpd,
  u8 inputAcc,
  u16 gripperSpd,
  u8 gripperAcc,
  u16 gripperTorque,
  u16 rodSpd,
  u8 rodAcc,
  u16 rodTorque
) {
  ExternalRod_pauseArmConstant(SHARED_BUS_FUSION_CMD_QUIET_MS);
  RoArmM2_allJointsAngleCtrl(inputBase, inputShoulder, inputElbow, inputHand, inputSpd, inputAcc);
  delay(5);
  ExternalGripper_moveToAngle(inputGripper, gripperSpd, gripperAcc, gripperTorque);
  delay(5);
  ExternalRod_moveToAngle(inputRod, rodSpd, rodAcc, rodTorque);
  ExternalRod_pauseArmConstant(SHARED_BUS_FUSION_CMD_QUIET_MS);
  RoArmM2_holdTorqueLock(true);
#if EXT_ROD_USE_ARM_BUS
  unsigned long quietUntil = millis() + SHARED_BUS_FUSION_CMD_QUIET_MS;
  if ((long)(quietUntil - sharedBusQuietUntil) > 0) {
    sharedBusQuietUntil = quietUntil;
  }
#endif
}

void ExternalRod_infoFeedback() {
  ExternalRod_getFeedback(true);

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = 1531;
  jsonInfoHttp["target"] = "rod";
  jsonInfoHttp["status"] = externalRodFeedback.status;
  jsonInfoHttp["pos"] = externalRodFeedback.pos;
  jsonInfoHttp["speed"] = externalRodFeedback.speed;
  jsonInfoHttp["load"] = externalRodFeedback.load;
  jsonInfoHttp["voltage"] = externalRodFeedback.voltage;
  jsonInfoHttp["current"] = externalRodFeedback.current;
  jsonInfoHttp["temper"] = externalRodFeedback.temper;
  jsonInfoHttp["mode"] = externalRodFeedback.mode;
  jsonInfoHttp["reqTorque"] = externalRodRequestedTorque;
  jsonInfoHttp["angle"] = externalRodPosToAngle(externalRodFeedback.pos);
  jsonInfoHttp["rad"] = externalRodPosToAngle(externalRodFeedback.pos) * M_PI / 180.0;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}

void BookArm_infoFeedback() {
  ExternalRod_getFeedback(true);

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = 1051;
  jsonInfoHttp["x"] = lastX;
  jsonInfoHttp["y"] = lastY;
  jsonInfoHttp["z"] = lastZ;
  jsonInfoHttp["b"] = radB;
  jsonInfoHttp["s"] = radS;
  jsonInfoHttp["e"] = radE;
  jsonInfoHttp["h"] = radG;
  jsonInfoHttp["wrist"] = radG;
  jsonInfoHttp["t"] = lastT;
  jsonInfoHttp["r"] = externalRodPosToAngle(externalRodFeedback.pos) * M_PI / 180.0;
  jsonInfoHttp["rodAngle"] = externalRodPosToAngle(externalRodFeedback.pos);
  jsonInfoHttp["torB"] = servoFeedback[BASE_SERVO_ID - BASE_SERVO_ID].load;
  jsonInfoHttp["torS"] = servoFeedback[SHOULDER_DRIVING_SERVO_ID - BASE_SERVO_ID].load -
                          servoFeedback[SHOULDER_DRIVEN_SERVO_ID - BASE_SERVO_ID].load;
  jsonInfoHttp["torE"] = servoFeedback[ELBOW_SERVO_ID - BASE_SERVO_ID].load;
  jsonInfoHttp["torH"] = servoFeedback[GRIPPER_SERVO_ID - BASE_SERVO_ID].load;
  jsonInfoHttp["torR"] = externalRodFeedback.load;
  jsonInfoHttp["rodStatus"] = externalRodFeedback.status;
  jsonInfoHttp["rodPos"] = externalRodFeedback.pos;
  jsonInfoHttp["rodId"] = EXT_ROD_SERVO_ID;
  jsonInfoHttp["gripperId"] = EXT_GRIPPER_SERVO_ID;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}

void BookArm_jointsArrayFeedback() {
  ExternalRod_getFeedback(true);

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = 1241;

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

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}
