
#include "dogbot/Strings.hh"
#include "dogbot/protocol.h"

#include <fstream>
#include <chrono>
#include <mutex>
#include <cassert>
#include <memory>
#include <exception>
#include <stdexcept>
#include <iostream>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/unistd.h>

namespace DogBotN {

  //! Convert a fault code to a string
  const char *FaultCodeToString(FaultCodeT faultCode)
  {
    switch(faultCode) {
    case FC_Ok: return "Ok";
    case FC_Unknown: return "Unknown";
    case FC_CalibrationFailed: return "Calibration Failed";
    case FC_DriverFault: return "Driver Fault";
    case FC_Internal: return "Internal error";
    case FC_Internal5VRailOutOfRange: return "5V Rail Out Of Range";
    case FC_InternalStoreFailed: return "Store failed";
    case FC_InternalTiming: return "Timing error";
    case FC_DriverOverTemperature: return "Over Temperature";
    case FC_OverVoltage:    return "Over Voltage";
    case FC_UnderVoltage:   return "Under Voltage";
    case FC_NoSensor:       return "No sensor";
    case FC_NoMotor:        return "No motor";
    case FC_PositionLost:   return "Position Lost";
    case FC_MotorResistanceOutOfRange: return "Resistance out of range.";
    case FC_MotorInducetanceOutOfRange: return "Inductance out of range.";
    case FC_InternalUSB: return "Internal USB error";
    case FC_InternalCAN: return "Internal CAN error";
    case FC_FanOverCurrent:return "Fan over current";
    case FC_MotorOverTemperature: return "Motor Over Temperature";
    case FC_SensorOverCurrent:return "Sensor over current";
    case FC_InvalidCommand:return "Invalid command";
    }

    printf("Invalid fault code %d \n", (int) faultCode);
    return "Invalid";
  }

  //! Convert the calibration state to a string
  const char *HomedStateToString(MotionHomedStateT calibrationState)
  {
    switch(calibrationState) {
      case MHS_Lost: return "Lost";
      case MHS_Measuring: return "Measuring";
      case MHS_Homed: return "Homed";
      case MHS_SoftHomed: return "SoftHomed";
    }
    printf("Unexpected homed state %d \n",(int)calibrationState);
    return "Invalid";
  }

  //! Convert the control mode to a string
  const char *ControlStateToString(ControlStateT controlState)
  {
    switch(controlState) {
    case CS_EmergencyStop: return "Emergency Stop";
    case CS_FactoryCalibrate: return "Factory Calibrate";
    case CS_MotionCalibrate: return "Motion Calibrate";
    case CS_SafeStop: return "Safe Stop";
    case CS_Standby: return "Standby";
    case CS_Ready: return "Ready";
    case CS_Home: return "Auto Home";
    case CS_SelfTest: return "Self Test";
    case CS_Fault: return "Fault";
    case CS_StartUp: return "Power Up";
    case CS_Diagnostic: return "Diagnostic";
    case CS_BootLoader: return "Boot Loader";
    }
    printf("Unexpected state %d \n",(int)controlState);
    return "Invalid";
  }

  //! Convert the control dynamic to a string
  const char *ControlDynamicToString(PWMControlDynamicT dynamic)
  {
    switch(dynamic) {
      case CM_Off: return "Off";
      case CM_Brake : return "Brake";
      case CM_Torque: return "Torque";
      case CM_Velocity: return "Velocity";
      case CM_Position: return "Position";
      case CM_Fault: return "Fault";
      case CM_Final: return "Final";
    }
    printf("Unexpected dynamic value %d \n",(int)dynamic);
    return "Invalid";
  }

  //! Convert coms error to a string
  const char *ComsErrorTypeToString(ComsErrorTypeT errorCode)
  {
    switch(errorCode) {
      case CET_UnknownPacketType: return "Unknown packet type";
      case CET_UnexpectedPacketSize: return "Unexpected packet size";
      case CET_ParameterOutOfRange: return "Parameter out of range ";
      case CET_CANTransmitFailed: return "CAN transmit failed";
      case CET_InternalError: return "Internal error";
      case CET_MotorNotRunning: return "Motor not running";
      case CET_NotImplemented: return "Not implemented";
      case CET_BootLoaderUnexpectedState: return "BootLoader in unexpected state";
      case CET_BootLoaderLostSequence: return "BootLoader lost sequence";
      case CET_BootLoaderErase: return "BootLoader erase";
      case CET_BootLoaderProtected: return "BootLoader protected";
      case CET_BootLoaderBusy: return "BootLoader busy";
      case CET_BootLoaderWriteFailed: return "BootLoader write failed";
      case CET_BootLoaderUnalignedAddress: return "BootLoader unaligned address";
      case CET_UnavailableInCurrentMode: return "Command unavailable in current mode.";
      case CET_MotorDriverWarning: return "Motor driver warning";
    }
    printf("Unexpected error code %d \n",(int)errorCode);
    return "Invalid";
  }

  //! Convert coms packet type to a string
  const char *ComsPacketTypeToString(ComsPacketTypeT packetType)
  {
    switch(packetType) {
      case CPT_NoOp: return "No-op";
      case CPT_EmergencyStop: return "Emergency Stop";
      case CPT_SyncTime: return "SyncTime";
      case CPT_Error: return "Error";
      case CPT_SetParam: return "SetParam";
      case CPT_Servo: return "Servo";
      case CPT_ServoReport: return "ServoReport";
      case CPT_ReportParam: return "ReportParam";
      case CPT_ReadParam: return "ReadParam";
      case CPT_Pong: return "Pong";
      case CPT_Ping: return "Ping";
      case CPT_AnnounceId: return "AnnounceId";
      case CPT_QueryDevices: return "QueryDevices";
      case CPT_SetDeviceId: return "SetDeviceId";
      case CPT_SaveSetup: return "SaveSetup";
      case CPT_LoadSetup: return "LoadSetup";
      case CPT_CalZero: return "CalZero";
      case CPT_Sync: return "Sync";
      case CPT_PWMState: return "PWMState";
      case CPT_BridgeMode: return "BridgeMode";
      case CPT_FlashCmdReset: return "FlashCmdReset";
      case CPT_FlashCmdResult: return "FlashCmdResult";
      case CPT_FlashChecksumResult: return "FlashChecksumResult";
      case CPT_FlashEraseSector: return "FlashEraseSector";
      case CPT_FlashChecksum: return "FlashChecksum";
      case CPT_FlashData: return "FlashData";
      case CPT_FlashWrite: return "FlashWrite";
      case CPT_FlashRead: return "FlashRead";
      case CPT_IMU: return "IMU";
      case CPT_Message: return "Message";
      case CPT_Range: return "Range";
      case CPT_Final:return "!!Final!!";
    }
    printf("Unexpected packet type %d \n",(int)packetType);
    return "Invalid";
  }

  //! Convert coms device type to a string
  const char *ComsDeviceTypeToString(DeviceTypeT deviceType)
  {
    switch(deviceType) {
      case DT_Unknown: return "unknown";
      case DT_PlatformManager: return "PlatformManager";
      case DT_MotorDriver: return "Servo";
      case DT_BootLoader: return "BootLoader";
      case DT_IMU: return "IMU";
    }
    printf("Unexpected device type %d \n",(int)deviceType);
    return "Invalid";
  }


  //! Convert a state change source to a string
  const char *ComsStateChangeSource(enum StateChangeSourceT changeSource)
  {
    switch(changeSource) {
      case SCS_UserRequest: return "UserRequest";
      case SCS_Internal: return "Internal";
      case SCS_Unknown:  return "Unknown";
      case SCS_Fault:   return "Fault";
      case SCS_EStopLostComs: return "EStopLostComs";
      case SCS_EStopSwitch: return "EStopSwitch";
      case SCS_External: return "External";
    }
    printf("Unexpected state change source %d \n",(int)changeSource);
    return "Invalid";
  }


  //! Convert coms safety mode to a string
  const char *SafetyModeToString(SafetyModeT safetyMode)
  {
    switch(safetyMode)
    {
      case SM_Unknown: return "Unknown";
      case SM_GlobalEmergencyStop: return "Global Emergency Stop";
      case SM_MasterEmergencyStop: return "Master Emergency Stop";
      case SM_LocalStop: return "Local Stop";
    }
    printf("Unexpected safety mode %d \n",(int)safetyMode);
    return "Invalid";
  }

  //! Convert an parameter index to a name
  const char *ComsParameterIndexToString(ComsParameterIndexT paramIndex)
  {
    switch(paramIndex)
    {
      case CPI_DeviceType: return "DeviceType";
      case CPI_FirmwareVersion: return "FirmwareVersion";
      case CPI_PWMState: return "PWMState";
      case CPI_PWMMode: return "PWMMode";
      case CPI_PWMFullReport: return "PWMFullReport";
      case CPI_CANBridgeMode: return "CANBridgeMode";
      case CPI_BoardUID: return "BoardUID";
      case CPI_TIM1_SR: return "TIM1_SR";
      case CPI_VSUPPLY: return "VSUPPLY";
      case CPI_ControlState: return "ControlState";
      case CPI_HomedState: return "HomedState";
      case CPI_PositionRef: return "PositionRef";
      case CPI_CalibrationOffset: return "CalibrationOffset";
      case CPI_FaultCode: return "FaultCode";
      case CPI_Indicator: return "Indicator";
      case CPI_DriveTemp: return "DriveTemp";
      case CPI_MotorTemp: return "MotorTemp";
      case CPI_OtherJoint: return "OtherJoint";
      case CPI_OtherJointGain: return "OtherJointGain";
      case CPI_OtherJointOffset: return "OtherJointOffset";
      case CPI_DebugIndex: return "DebugIndex";
      case CPI_MotorResistance: return "MotorResistance";
      case CPI_MotorInductance: return "MotorInductance";
      case CPI_MotorOffsetVoltage: return "MotorOffsetVoltage";
      case CPI_MotorIGain: return "MotorIGain";
      case CPI_MotorPGain: return "MotorPGain";
      case CPI_PhaseVelocity: return "PhaseVelocity";
      case CPI_VelocityPGain: return "VelocityPGain";
      case CPI_VelocityIGain: return "VelocityIGain";
      case CPI_DemandPhaseVelocity: return "DemandPhaseVelocity";
      case CPI_VelocityLimit: return "VelocityLimit";
      case CPI_PositionGain: return "PositionGain";

      case CPI_DRV8305_01: return "DRV8305_01";
      case CPI_DRV8305_02: return "DRV8305_02";
      case CPI_DRV8305_03: return "DRV8305_03";
      case CPI_DRV8305_04: return "DRV8305_04";
      case CPI_DRV8305_05: return "DRV8305_05";

      case CPI_5VRail: return "5VRail";
      case CPI_MaxCurrent: return "MaxCurrent";
      case CPI_homeIndexPosition: return "homeIndexPosition";
      case CPI_HallSensors: return "HallSensors";
      case CPI_MinSupplyVoltage: return "MinSupplyVoltage";
      case CPI_USBPacketDrops: return "USBPacketDrops";
      case CPI_USBPacketErrors: return "USBPacketErrors";
      case CPI_FaultState: return "FaultState";
      case CPI_IndexSensor: return "IndexSensor";

      case CPI_ANGLE_CAL_0: return "ANGLE_CAL_0";
      case CPI_ANGLE_CAL_1: return "ANGLE_CAL_1";
      case CPI_ANGLE_CAL_2: return "ANGLE_CAL_2";
      case CPI_ANGLE_CAL_3: return "ANGLE_CAL_3";
      case CPI_ANGLE_CAL_4: return "ANGLE_CAL_4";
      case CPI_ANGLE_CAL_5: return "ANGLE_CAL_5";
      case CPI_ANGLE_CAL_6: return "ANGLE_CAL_6";
      case CPI_ANGLE_CAL_7: return "ANGLE_CAL_7";
      case CPI_ANGLE_CAL_8: return "ANGLE_CAL_8";
      case CPI_ANGLE_CAL_9: return "ANGLE_CAL_9";
      case CPI_ANGLE_CAL_10: return "ANGLE_CAL_10";
      case CPI_ANGLE_CAL_11: return "ANGLE_CAL_11";
      case CPI_ANGLE_CAL_12: return "ANGLE_CAL_12";
      case CPI_ANGLE_CAL_13: return "ANGLE_CAL_13";
      case CPI_ANGLE_CAL_14: return "ANGLE_CAL_14";
      case CPI_ANGLE_CAL_15: return "ANGLE_CAL_15";
      case CPI_ANGLE_CAL_16: return "ANGLE_CAL_16";
      case CPI_ANGLE_CAL_17: return "ANGLE_CAL_17";

      case CPI_CANPacketDrops: return "CANPacketDrops";
      case CPI_CANPacketErrors: return "CANPacketErrors";
      case CPI_MainLoopTimeout: return "MainLoopTimeout";
      case CPI_JointRelative: return "JointRelative";
      case CPI_FanTemperatureThreshold: return "FanTemperatureThreshold";
      case CPI_FanMode: return "FanMode";
      case CPI_FanState: return "FanState";

      case CPI_SafetyMode: return "SafetyMode";
      case CPI_JointRole: return "JointRole";
      case CPI_EndStopEnable: return "EndStopEnable";
      case CPI_EndStopStart: return "EndStopStart";
      case CPI_EndStopStartBounce: return "EndStopStartBounce";
      case CPI_EndStopFinal: return "EndStopEnd";
      case CPI_EndStopEndBounce: return "EndStopEndBounce";
      case CPI_EndStopTargetBreakForce: return "EndStopTargetBreakForce";
      case CPI_EndStopLimitBreakForce: return "EndStopLimitBreakForce";
      case CPI_JointInertia: return "JointInertia";
      case CPI_ServoReportFrequency: return "ServoReportRate";
      case CPI_PWMFrequency: return "PWMFrequency";
      case CPI_EndStopPhaseAngles: return "EndStopPhaseAngles";
      case CPI_MotionUpdatePeriod: return "MotionUpdatePeriod";
      case CPI_SupplyVoltageScale: return "SupplyVoltageScale";
      case CPI_CurrentLimit: return "CurrentLimit";
      case CPI_PlatformActivity: return "PlatformActivity";
      case CPI_RequestedPlatformActivity: return "RequestedPlatformActivity";
      case CPI_FINAL: return "FINAL";
    }
    printf("Unexpected parameter index %d \n",(int)paramIndex);
    return "Invalid";
  }


  //! Get type information for parameters
  enum ComsParameterIndexTypeT ComsParameterIndexToType(ComsParameterIndexT paramIndex)
  {

    switch(paramIndex)
    {
      case CPI_DeviceType: return CPIT_enum8;
      case CPI_FirmwareVersion: return CPIT_uint8;
      case CPI_PWMState: return CPIT_enum8;
      case CPI_PWMMode: return CPIT_enum8;
      case CPI_PWMFullReport: return CPIT_bool;
      case CPI_CANBridgeMode: return CPIT_bool;
      case CPI_BoardUID: return CPIT_uint32_2;
      case CPI_TIM1_SR: return CPIT_uint16;
      case CPI_VSUPPLY: return CPIT_uint16;
      case CPI_ControlState: return CPIT_enum8;
      case CPI_HomedState: return CPIT_enum8;
      case CPI_PositionRef: return CPIT_enum8;
      case CPI_CalibrationOffset: return CPIT_float32;
      case CPI_FaultCode: return CPIT_enum8;
      case CPI_Indicator: return CPIT_bool;
      case CPI_DriveTemp: return CPIT_float32;
      case CPI_MotorTemp: return CPIT_float32;
      case CPI_OtherJoint: return CPIT_uint8;
      case CPI_OtherJointGain: return CPIT_float32;
      case CPI_OtherJointOffset: return CPIT_float32;
      case CPI_DebugIndex: return CPIT_uint8;
      case CPI_MotorResistance: return CPIT_float32;
      case CPI_MotorInductance: return CPIT_float32;
      case CPI_MotorOffsetVoltage: return CPIT_float32;
      case CPI_MotorIGain: return CPIT_float32;
      case CPI_MotorPGain: return CPIT_float32;
      case CPI_PhaseVelocity: return CPIT_float32;
      case CPI_VelocityPGain: return CPIT_float32;
      case CPI_VelocityIGain: return CPIT_float32;
      case CPI_DemandPhaseVelocity: return CPIT_float32;
      case CPI_VelocityLimit: return CPIT_float32;
      case CPI_PositionGain: return CPIT_float32;

      case CPI_DRV8305_01: return CPIT_uint16;
      case CPI_DRV8305_02: return CPIT_uint16;
      case CPI_DRV8305_03: return CPIT_uint16;
      case CPI_DRV8305_04: return CPIT_uint16;
      case CPI_DRV8305_05: return CPIT_uint16;

      case CPI_5VRail: return CPIT_uint16;
      case CPI_MaxCurrent: return CPIT_float32;
      case CPI_homeIndexPosition: return CPIT_float32;
      case CPI_HallSensors: return CPIT_uint16_3;
      case CPI_MinSupplyVoltage: return CPIT_float32;
      case CPI_USBPacketDrops: return CPIT_uint32;
      case CPI_USBPacketErrors: return CPIT_uint32;
      case CPI_FaultState: return CPIT_uint32;
      case CPI_IndexSensor: return CPIT_bool;

      case CPI_ANGLE_CAL_0: return CPIT_uint16_3;
      case CPI_ANGLE_CAL_1: return CPIT_uint16_3;
      case CPI_ANGLE_CAL_2: return CPIT_uint16_3;
      case CPI_ANGLE_CAL_3: return CPIT_uint16_3;
      case CPI_ANGLE_CAL_4: return CPIT_uint16_3;
      case CPI_ANGLE_CAL_5: return CPIT_uint16_3;
      case CPI_ANGLE_CAL_6: return CPIT_uint16_3;
      case CPI_ANGLE_CAL_7: return CPIT_uint16_3;
      case CPI_ANGLE_CAL_8: return CPIT_uint16_3;
      case CPI_ANGLE_CAL_9: return CPIT_uint16_3;
      case CPI_ANGLE_CAL_10: return CPIT_uint16_3;
      case CPI_ANGLE_CAL_11: return CPIT_uint16_3;
      case CPI_ANGLE_CAL_12: return CPIT_uint16_3;
      case CPI_ANGLE_CAL_13: return CPIT_uint16_3;
      case CPI_ANGLE_CAL_14: return CPIT_uint16_3;
      case CPI_ANGLE_CAL_15: return CPIT_uint16_3;
      case CPI_ANGLE_CAL_16: return CPIT_uint16_3;
      case CPI_ANGLE_CAL_17: return CPIT_uint16_3;

      case CPI_CANPacketDrops: return CPIT_uint32;
      case CPI_CANPacketErrors: return CPIT_uint32;
      case CPI_MainLoopTimeout: return CPIT_uint32;
      case CPI_JointRelative: return CPIT_enum8;
      case CPI_FanTemperatureThreshold: return CPIT_float32;
      case CPI_FanMode: return CPIT_enum8;
      case CPI_FanState: return CPIT_enum8;

      case CPI_SafetyMode: return CPIT_enum8;
      case CPI_JointRole: return CPIT_enum8;
      case CPI_EndStopEnable: return CPIT_bool;
      case CPI_EndStopStart: return CPIT_float32;
      case CPI_EndStopStartBounce: return CPIT_float32;
      case CPI_EndStopFinal: return CPIT_float32;
      case CPI_EndStopEndBounce: return CPIT_float32;
      case CPI_EndStopTargetBreakForce: return CPIT_float32;
      case CPI_EndStopLimitBreakForce: return CPIT_float32;
      case CPI_JointInertia: return CPIT_float32;
      case CPI_ServoReportFrequency: return CPIT_float32;
      case CPI_PWMFrequency: return CPIT_float32;
      case CPI_EndStopPhaseAngles: return CPIT_float32_2;
      case CPI_MotionUpdatePeriod: return CPIT_float32;
      case CPI_SupplyVoltageScale: return CPIT_float32;
      case CPI_CurrentLimit: return CPIT_float32;
      case CPI_PlatformActivity: return CPIT_Custom;
      case CPI_RequestedPlatformActivity: return CPIT_Custom;
      case CPI_FINAL: return CPIT_Invalid;
    }

    printf("Unexpected parameter index %d for type query. \n",(int)paramIndex);
    return CPIT_Unknown;
  }

  //! Convert an parameter index to a name
  const char *ComsPositionRefrenceToString(enum PositionReferenceT referenceType)
  {
    switch(referenceType)
    {
      case PR_Relative: return "Relative";
      case PR_Absolute: return "Absolute";
    }
    printf("Unexpected position reference %d \n",(int)referenceType);
    return "Invalid";
  }

  //! Convert an parameter index to a name
  const char *ComsPlatformActivityToString(enum PlatformActivityT activity)
  {
    switch(activity)
    {
      case PA_Passive:    return "Passive";
      case PA_Idle:       return "Idle";
      case PA_Bootloader: return "Bootloader";
      case PA_Home:       return "Home";
      case PA_Falling:    return "Falling";
      case PA_Stand:      return "Stand";
      case PA_Walking:    return "Walking";
      case PA_Shutdown:   return "Shutdown";
      case PA_ROS:        return "ROS";
      case PA_User:       return "User";
    }
    printf("Unexpected platform activity type %d \n",(int)activity);
    return "Invalid";
  }

}
