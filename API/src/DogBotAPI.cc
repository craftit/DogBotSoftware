
#include "dogbot/DogBotAPI.hh"
#include "dogbot/protocol.h"
#include "dogbot/ComsSerial.hh"
#include "dogbot/ComsZMQClient.hh"
#include "dogbot/ComsUSB.hh"
#include "dogbot/ComsProxy.hh"
#include "dogbot/Joint4BarLinkage.hh"
#include "dogbot/JointRelative.hh"
#include "dogbot/DeviceIMU.hh"
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
      case MHS_ApproxHomed: return "Approximated";
    }
    printf("Unexpected homed state %d",(int)calibrationState);
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
    printf("Unexpected state %d",(int)controlState);
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
    printf("Unexpected dynamic value %d",(int)dynamic);
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
    }
    printf("Unexpected error code %d",(int)errorCode);
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
      case CPT_Final:return "!!Final!!";
    }
    printf("Unexpected packet type %d",(int)packetType);
    return "Invalid";
  }

  //! Convert coms device type to a string
  const char *ComsDeviceTypeToString(DeviceTypeT deviceType)
  {
    switch(deviceType) {
      case DT_Unknown: return "unknown";
      case DT_SystemController: return "system controller";
      case DT_MotorDriver: return "motor driver";
      case DT_BootLoader: return "boot loader";
      case DT_IMU: return "IMU";
    }
    printf("Unexpected device type %d",(int)deviceType);
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
    }
    printf("Unexpected state change source %d",(int)changeSource);
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
    printf("Unexpected safety mode %d",(int)safetyMode);
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
      case CPI_FINAL: return "FINAL";
    }
    printf("Unexpected parameter index %d",(int)paramIndex);
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

      case CPI_ANGLE_CAL_0: return CPIT_uint16;
      case CPI_ANGLE_CAL_1: return CPIT_uint16;
      case CPI_ANGLE_CAL_2: return CPIT_uint16;
      case CPI_ANGLE_CAL_3: return CPIT_uint16;
      case CPI_ANGLE_CAL_4: return CPIT_uint16;
      case CPI_ANGLE_CAL_5: return CPIT_uint16;
      case CPI_ANGLE_CAL_6: return CPIT_uint16;
      case CPI_ANGLE_CAL_7: return CPIT_uint16;
      case CPI_ANGLE_CAL_8: return CPIT_uint16;
      case CPI_ANGLE_CAL_9: return CPIT_uint16;
      case CPI_ANGLE_CAL_10: return CPIT_uint16;
      case CPI_ANGLE_CAL_11: return CPIT_uint16;
      case CPI_ANGLE_CAL_12: return CPIT_uint16;
      case CPI_ANGLE_CAL_13: return CPIT_uint16;
      case CPI_ANGLE_CAL_14: return CPIT_uint16;
      case CPI_ANGLE_CAL_15: return CPIT_uint16;
      case CPI_ANGLE_CAL_16: return CPIT_uint16;
      case CPI_ANGLE_CAL_17: return CPIT_uint16;

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
      case CPI_MotionUpdatePeriod: return CPIT_uint16;
      case CPI_FINAL: return CPIT_Invalid;
    }

    printf("Unexpected parameter index %d for type query",(int)paramIndex);
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
    printf("Unexpected position reference %d ",(int)referenceType);
    return "Invalid";
  }

  // ---------------------------------------------------------

  //! Constructor
  DogBotAPIC::DogBotAPIC()
  {
    m_manageComs = true;
  }

  //! Construct with coms object
  DogBotAPIC::DogBotAPIC(
      const std::shared_ptr<ComsC> &coms,
      const std::string &configurationFile,
      const std::shared_ptr<spdlog::logger> &log,
      bool manageComs,
      DeviceManagerModeT devMasterMode
      )
   : m_coms(coms),
     m_deviceManagerMode(devMasterMode)
  {
    m_manageComs = manageComs;
    m_log = log;
    m_coms->SetLogger(log);
    if(!configurationFile.empty()) {
      LoadConfig(configurationFile);
    }
    Init();
  }

  //! Construct with a string
  DogBotAPIC::DogBotAPIC(
      const std::string &connectionName,
      const std::string &configurationFile,
      const std::shared_ptr<spdlog::logger> &log,
      DeviceManagerModeT devMasterMode
      )
    : m_deviceManagerMode(devMasterMode)
  {
    m_log = log;
    m_manageComs = true;
    if(!connectionName.empty())
      Connect(connectionName);
    if(!configurationFile.empty()) {
      LoadConfig(configurationFile);
    }
    Init();
  }


  DogBotAPIC::~DogBotAPIC()
  {
    m_terminate = true;
    if(m_threadMonitor.joinable())
      m_threadMonitor.join();
  }

  //! Set the logger to use
  void DogBotAPIC::SetLogger(const std::shared_ptr<spdlog::logger> &log)
  {
    m_log = log;
    m_coms->SetLogger(log);
  }


  //! Shutdown controller.
  bool DogBotAPIC::Shutdown()
  {
    m_terminate = true;
    return false;
  }

  //! Connect to a named device
  bool DogBotAPIC::Connect(const std::string &name)
  {
    m_deviceName = name;

    auto colonAt = name.find(':');
    std::string prefix;
    if(colonAt != std::string::npos) {
      prefix = name.substr(0,colonAt);
    }
    bool doOpen = true;
    //std::cerr << "Got prefix '" << prefix << "' " << std::endl;
    if(prefix == "tcp" || prefix == "udp") {
      m_coms = std::make_shared<ComsZMQClientC>(name);
      if(m_deviceManagerMode == DMM_Auto)
        m_deviceManagerMode = DMM_ClientOnly;
    } else  if(name == "local") {
      m_coms = std::make_shared<ComsZMQClientC>("tcp://127.0.0.1");
      if(m_deviceManagerMode == DMM_Auto)
        m_deviceManagerMode = DMM_ClientOnly;
    } else if(name == "none") {
      m_coms = std::make_shared<ComsProxyC>();
      if(m_deviceManagerMode == DMM_Auto)
        m_deviceManagerMode = DMM_ClientOnly;
      doOpen = false;
    } else if(name == "usb") {
      m_coms = std::make_shared<ComsUSBC>();
      if(m_deviceManagerMode == DMM_Auto)
        m_deviceManagerMode = DMM_DeviceManager;
    } else {
      m_coms = std::make_shared<ComsSerialC>();
      if(m_deviceManagerMode == DMM_Auto)
        m_deviceManagerMode = DMM_DeviceManager;
    }
    m_coms->SetLogger(m_log);
    if(doOpen) {
      if(!m_coms->Open(name.c_str()))
        return false;
    }
    {
      std::lock_guard<std::mutex> lock(m_mutexDevices);
      for(auto &a : m_devices)
        if(a) a->UpdateComs(m_coms);
    }
    return true;
  }

  //! Connect to coms object.
  bool DogBotAPIC::Connect(const std::shared_ptr<ComsC> &coms)
  {
    assert(!m_coms);
    m_coms = coms;
    return true;
  }

  //! Access connection
  std::shared_ptr<ComsC> DogBotAPIC::Connection()
  {
    return m_coms;
  }

  //! Read calibration from a device.
  bool DogBotAPIC::ReadCalibration(int deviceId,MotorCalibrationC &cal)
  {
    if(!m_coms)
      return false;

    int toGo = 12;
    std::vector<bool> gotData(toGo,false);
    std::timed_mutex done;
    done.lock();
    CallbackSetC callbacks;
    callbacks += m_coms->SetHandler(CPT_ReportParam,[this,&gotData,&toGo,&cal,&done](const uint8_t *data,int size) mutable
      {
        const struct PacketParam8ByteC *psp = (const struct PacketParam8ByteC *) data;
        int index = psp->m_header.m_index;
        if(index < CPI_ANGLE_CAL || index > CPI_ANGLE_CAL_11)
          return ;
        index -= (int) CPI_ANGLE_CAL;
        //m_log->info("Cal data. {} : {} {} {}",index,psp->m_data.uint16[0],psp->m_data.uint16[1],psp->m_data.uint16[2]);

        cal.SetCal(index,psp->m_data.uint16[0],psp->m_data.uint16[1],psp->m_data.uint16[2]);
        if(!gotData[index]) {
          toGo--;
          gotData[index] = true;
          if(toGo == 0)
            done.unlock();
        }
      });
    bool ret = false;
    for(int i = 0;i < 4;i++) {
      // Send query for each parameter
      for(int i = 0;i < 12;i++) {
        if(!gotData[i])
          m_coms->SendQueryParam(deviceId,static_cast<enum ComsParameterIndexT>(CPI_ANGLE_CAL + i));
      }
      // Did we get them all ?
      if(done.try_lock_for(std::chrono::milliseconds(200))) {
        ret = true;
        //m_log->info("Got calibration data.");
        break;
      }
    }

    return ret;
  }

  //! Write calibration to a device.
  bool DogBotAPIC::WriteCalibration(int deviceId,const MotorCalibrationC &cal)
  {
    if(!m_coms)
      return false;

    int toGo = 12;
    std::vector<bool> gotData(toGo,false);
    std::timed_mutex done;
    done.lock();
    CallbackSetC callbacks;
    callbacks += m_coms->SetHandler(CPT_ReportParam,[this,&gotData,&toGo,&cal,&done](const uint8_t *data,int size) mutable
      {
        const struct PacketParam8ByteC *psp = (const struct PacketParam8ByteC *) data;
        int index = psp->m_header.m_index;
        if(index < CPI_ANGLE_CAL || index > CPI_ANGLE_CAL_11)
          return ;
        index -= (int) CPI_ANGLE_CAL;
        //m_log->info("Cal data. {} : {} {} {}",index,psp->m_data.uint16[0],psp->m_data.uint16[1],psp->m_data.uint16[2]);
        if(!gotData[index]) {

          uint16_t h1,h2,h3;
          cal.GetCal(index,h1,h2,h3);
          bool updateOk = (h1 == psp->m_data.uint16[0]) && (h2 == psp->m_data.uint16[1]) && (h3 == psp->m_data.uint16[2]);
          if(updateOk) {
            toGo--;
            gotData[index] = true;
            if(toGo == 0)
              done.unlock();
          }
        }
      });

    bool ret = false;
    for(int i = 0;i < 4;i++) {
      // Send query for each parameter
      for(int i = 0;i < 12;i++) {
        if(!gotData[i]) {
          //m_coms->SendQueryParam(deviceId,static_cast<enum ComsParameterIndexT>(CPI_ANGLE_CAL + i));
          BufferTypeT buff;
          cal.GetCal(i,buff.uint16[0],buff.uint16[1],buff.uint16[2]);
          m_coms->SendSetParam(deviceId,static_cast<enum ComsParameterIndexT>(CPI_ANGLE_CAL + i),buff,6);
          // Query value to check it was set correctly.
          m_coms->SendQueryParam(deviceId,static_cast<enum ComsParameterIndexT>(CPI_ANGLE_CAL + i));
        }
      }
      // Did we set them all ok ?
      if(done.try_lock_for(std::chrono::milliseconds(200))) {
        ret = true;
        //m_log->info("Got calibration data.");
        break;
      }
    }

    return true;
  }


  //! Load configuration for the robot.

  bool DogBotAPIC::Init()
  {
    m_log->info("Starting API. ");
    if(m_started) {
      m_log->error("Init already called. ");
      return false;
    }

    for(int i = 0;i < m_emergencyStopFlags.size();i++)
      m_emergencyStopFlags[i] = false;

    //! Initialise which serial device to use.
    if(m_deviceName.empty())
      m_deviceName = m_configRoot.get("device","usb").asString();
    if(m_deviceManagerMode == DMM_Auto && !m_deviceName.empty()) {
      if(m_deviceName == "local") {
        m_deviceManagerMode = DMM_ClientOnly;
      } else {
        m_deviceManagerMode = DMM_DeviceManager;
      }
    }
    m_started = true;

    m_threadMonitor = std::move(std::thread { [this]{ RunMonitor(); } });

    return true;
  }


  //! Home from squatting position
  bool DogBotAPIC::HomeSquat()
  {
//            "back_left_knee" : 238.32269287109375,
//            "back_left_pitch" : -58.273262023925781,
//            "back_left_roll" : 0,
//            "back_right_knee" : 121.75420379638672,
//            "back_right_pitch" : 58.361156463623047,
//            "back_right_roll" : 0,
//            "front_left_knee" : 238.30073547363281,
//            "front_left_pitch" : -59.393905639648438,
//            "front_left_roll" : 0,
//            "front_right_knee" : 121.66630554199219,
//            "front_right_pitch" : 59.591667175292969,
//            "front_right_roll" : 0

#if 0
    // First check all the roll joints are in the right place.
    {

      for(int i = 0;i < 4;i++) {
        std::string jointName = legNames[i] + "_roll";
        std::shared_ptr<ServoC> jnt = std::dynamic_pointer_cast<ServoC>(GetJointByName(jointName));
      }

      //m_homeIndexState
    }
#endif


    return true;
  }

  //! Home whole robot;
  bool DogBotAPIC::HomeAll()
  {
    const std::vector<std::string> &legNames = LegNames();
    bool allOk = true;
    {
      std::vector<std::thread> threads;

      // Home roll
      for(int i = 0;i < legNames.size();i++) {
        std::string jointName = legNames[i] + "_roll";
        std::shared_ptr<ServoC> jnt = std::dynamic_pointer_cast<ServoC>(GetJointByName(jointName));
        if(!jnt) {
          m_log->error("Failed to find joint {} ",jointName);
          return false;
        }
        threads.push_back(std::thread([jnt,&allOk](){
          if(!jnt->HomeJoint(false)) {
            allOk = false;
            return false;
          }
          jnt->DemandPosition(0,2.0,PR_Absolute);
          return true;
        }));
      }
      for(auto &thr: threads) {
        if(thr.joinable())
          thr.join();
      }
      if(!allOk)
        return false;
    }

    {
      std::vector<std::thread> threads;
      // Home knees.
      for(int i = 0;i < legNames.size();i++) {
        std::string jointName = legNames[i] + "_knee";
        std::shared_ptr<ServoC> jnt = std::dynamic_pointer_cast<ServoC>(GetJointByName(jointName));
        if(!jnt) {
          m_log->error("Failed to find joint {} ",jointName);
          return false;
        }
        threads.push_back(std::thread([jnt,&allOk](){
          if(!jnt->HomeJoint(false)) {
            allOk = false;
            return false;
          }
          jnt->DemandPosition(0,2.0,PR_Absolute);
          return true;
        }));
      }
      for(auto &thr: threads) {
        if(thr.joinable())
          thr.join();
      }
      if(!allOk)
        return false;
    }

    {
      std::vector<std::thread> threads;
      // Home pitch
      for(int i = 0;i < legNames.size();i++) {
        std::string jointName = legNames[i] + "_pitch";
        std::shared_ptr<ServoC> jnt = std::dynamic_pointer_cast<ServoC>(GetJointByName(jointName));
        if(!jnt) {
          m_log->error("Failed to find joint {} ",jointName);
          return false;
        }
        threads.push_back(std::thread([jnt,&allOk](){
          if(!jnt->HomeJoint(false)) {
            allOk = false;
            return false;
          }
          jnt->DemandPosition(0,2.0,PR_Absolute);
          return true;
        }));
      }
      for(auto &thr: threads) {
        if(thr.joinable())
          thr.join();
      }
      if(!allOk)
        return false;
    }

    return true;
  }

  //! Make an appropriate type of device.
  std::shared_ptr<JointC> DogBotAPIC::MakeJoint(const std::string &deviceType) const
  {
    if(deviceType == "relative") {
      return std::make_shared<JointRelativeC>();
    } else if(deviceType == "4bar") {
      return std::make_shared<Joint4BarLinkageC>();
    }
    m_log->error("Unknown joint type '{}'",deviceType);
    assert(0);
    throw std::runtime_error("Unknown joint type.");
  }


  static bool FileExists(const std::string &filename)
  {
    struct stat sb;
    return stat(filename.c_str(),&sb) ==  0;
  }


  //! Find the default configuration file
  std::string DogBotAPIC::DefaultConfigFile()
  {
    std::string devFilename = "local";
    std::string homeDir = getenv("HOME");
    std::string defaultConfig = homeDir + "/.config/dogbot/robot.json";
    if(!FileExists(defaultConfig))  {
      std::cerr << "Default configuration file '" << defaultConfig << "' doesn't exist. " << std::endl;
      return "";
    }

    return defaultConfig;
  }

  //! Make a new device
  std::shared_ptr<DeviceC> DogBotAPIC::MakeDevice(int deviceId,const std::string &deviceTypeName)
  {
    if(deviceTypeName == "servo")
      return MakeDevice(deviceId,DT_MotorDriver);
    if(deviceTypeName == "imu")
      return MakeDevice(deviceId,DT_IMU);
    if(deviceTypeName == "system")
      return MakeDevice(deviceId,DT_SystemController);
    m_log->error("Asked to create unknown device type '{}' ",deviceTypeName);
    return MakeDevice(deviceId,DT_Unknown);
  }

  //! Make a new device
  std::shared_ptr<DeviceC> DogBotAPIC::MakeDevice(int deviceId,DeviceTypeT deviceType)
  {
    m_log->info("Creating device {} of type '{}' ({}) ",deviceId,ComsDeviceTypeToString(deviceType),(int) deviceType);
    switch(deviceType)
    {
      case DT_Unknown: return std::make_shared<DeviceC>(m_coms,deviceId);
      case DT_MotorDriver: return std::make_shared<ServoC>(m_coms,deviceId);
      case DT_SystemController:
      case DT_BootLoader:
      case DT_IMU: return std::make_shared<DeviceIMUC>(m_coms,deviceId);
        break;
    }
    m_log->warn("Asked to create unknown device type {} ",(int) deviceType);
    return std::make_shared<DeviceC>(m_coms,deviceId);
  }


  //! Make device for given id
  std::shared_ptr<DeviceC> DogBotAPIC::MakeDevice(int deviceId,const PacketDeviceIdC &pkt)
  {
    DeviceTypeT deviceType = (DeviceTypeT) ((int) pkt.m_idBytes[3] >> 4);
    m_log->info("Creating device {} of type '{}' ({}) ID: {} {}  ",deviceId,ComsDeviceTypeToString(deviceType),deviceType,pkt.m_uid[0],pkt.m_uid[1]);
    switch(deviceType)
    {
      case DT_MotorDriver:
      case DT_Unknown: return std::make_shared<ServoC>(m_coms,deviceId,pkt);
      case DT_IMU: return std::make_shared<DeviceIMUC>(m_coms,deviceId,pkt);
      case DT_BootLoader:
      case DT_SystemController: return std::make_shared<DeviceC>(m_coms,deviceId,pkt);
    }
    m_log->warn("Asked to create unknown device type {} ",(int) deviceType);
    return std::make_shared<DeviceC>(m_coms,deviceId,pkt);
  }

  //! Load a configuration file
  bool DogBotAPIC::LoadConfig(const std::string &configFile)
  {
    if(configFile.empty())
      return false;
#if 0
    try {
#endif
      std::ifstream confStrm(configFile,std::ifstream::binary);

      if(!confStrm) {
        m_log->error("Failed to open configuration file '{}' ",configFile);
        return false;
      }

      Json::Value rootConfig;
      confStrm >> rootConfig;

      m_dogBotKinematics.ConfigureFromJSON(rootConfig);

      Json::Value deviceList = rootConfig["devices"];
      if(!deviceList.isNull()) {
        ServoUpdateTypeT op = SUT_Updated;
        for(int i = 0;i < deviceList.size();i++) {
          Json::Value deviceConf = deviceList[i];
          std::string jointType = deviceConf.get("type","").asString();
          if(!jointType.empty() && (
              jointType == "relative" ||
              jointType == "4bar" ||
              jointType == "joint"))
          {
            std::shared_ptr<JointC> joint = MakeJoint(jointType);
            joint->ConfigureFromJSON(*this,deviceConf);
            m_log->info("Loaded joint '{}' of type '{}' ",joint->Name(),jointType);
            m_joints.push_back(joint);
            continue;
          }
          std::shared_ptr<DeviceC> device;
          int deviceId = 0;
          uint32_t uid1 = deviceConf.get("uid1",0u).asUInt();
          uint32_t uid2 = deviceConf.get("uid2",0u).asUInt();

          int reqId = deviceConf.get("deviceId",0u).asInt();
          std::string deviceType = deviceConf.get("device_type","servo").asString();
          // Ignore silly ids
          if(reqId > 255 || reqId < 0)
            reqId = 0;
          {
            std::lock_guard<std::mutex> lock(m_mutexDevices);
            // Is the device present ?
            for(int i = 0;i < m_devices.size();i++) {
              if(!m_devices[i])
                continue;
              if(m_devices[i]->HasUID(uid1,uid2)) {
                deviceId = i;
                device = m_devices[i];
                break;
              }
            }
            // Go with the requested configuration id if we can.
            if(deviceId == 0 && reqId != 0) {
              if(m_devices.size() > reqId) {
                if(!m_devices[reqId]) {
                  deviceId = reqId;
                  if(deviceId > 0) {
                    device = MakeDevice(deviceId,deviceType);
                    m_devices[deviceId] = device;
                  }
                }
              } else {
                while(m_devices.size() < reqId)
                  m_devices.push_back(std::shared_ptr<ServoC>());
                deviceId = reqId;
                device = MakeDevice(deviceId,deviceType);
                m_devices.push_back(device);
                op = SUT_Add;
              }
            }
            // Still no luck... just append it.
            if(deviceId == 0) {
              deviceId = (int) m_devices.size();
              device = MakeDevice(deviceId,deviceType);
              m_devices.push_back(device);
              op = SUT_Add;
            }
          }
          assert(device);
          if(device) {
            device->ConfigureFromJSON(*this,deviceConf);
            m_log->info("Loaded device '{}' of type '{}' ",device->DeviceName(),deviceType);
            DeviceStatusUpdate(device.get(),op);
            std::shared_ptr<JointC> jnt = std::dynamic_pointer_cast<JointC>(device);
            if(jnt) m_joints.push_back(jnt);
          }
        }
      }
#if 0
    } catch(std::exception &ex) {
      m_log->error("Failed to load configuration file. : {}", ex.what());
      return false;
    }
#endif

    return true;
  }

  //! Save configuration of robot

  bool DogBotAPIC::SaveConfig(const std::string &configFile)
  {
    std::ofstream confStrm(configFile,std::ifstream::binary);

    if(!confStrm) {
      m_log->error("Failed to open configuration file '{}' ",configFile);
      return false;
    }
    Json::Value rootConfig;

    {
      std::lock_guard<std::mutex> lock(m_mutexDevices);
      Json::Value deviceList;
      int index =0;
      for(auto &a : m_devices) {
        if(!a)
          continue;
        a->ConfigAsJSON(deviceList[index++]);
      }
      for(auto &a : m_joints) {
        if(!a)
          continue;
        if(a->JointType() == "servo")
          continue;
        a->ConfigAsJSON(deviceList[index++]);
      }
      rootConfig["devices"] = deviceList;

      //! Get the servo configuration as JSON
      m_dogBotKinematics.ConfigAsJSON(rootConfig);
    }

    confStrm << rootConfig;

    return true;
  }


  //! Issue an update notification
  void DogBotAPIC::DeviceStatusUpdate(DeviceC *id,ServoUpdateTypeT op)
  {
    for(auto &a : m_deviceStatusCallbacks.Calls()) {
      if(a) a(id,op);
    }
    JointC *jnt = dynamic_cast<JointC *>(id);
    if(jnt != 0) {
      for(auto &a : m_jointStatusCallbacks.Calls()) {
        if(a) a(jnt,op);
      }
    }
  }

  //! Access servo by deviceId
  std::shared_ptr<ServoC> DogBotAPIC::ServoEntry(int deviceId)
  {
    std::shared_ptr<ServoC> ret;
    std::lock_guard<std::mutex> lock(m_mutexDevices);

    // The device clearly exists, so make sure there is an entry.
    if(deviceId >= m_devices.size() || deviceId < 0)
      return std::shared_ptr<ServoC>();
    ret = std::dynamic_pointer_cast<ServoC>(m_devices[deviceId]);
    if(!ret && m_devices[deviceId])
      m_log->warn("Device {} not a servo. ",deviceId);
    return ret;
  }

  //! Access by device id
  std::shared_ptr<DeviceC> DogBotAPIC::DeviceEntry(int deviceId)
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);

    // The device clearly exists, so make sure there is an entry.
    if(deviceId >= m_devices.size() || deviceId < 0)
      return std::shared_ptr<DeviceC>();
    return m_devices[deviceId];
  }

  void DogBotAPIC::HandlePacketAnnounce(const PacketDeviceIdC &pkt)
  {
    std::shared_ptr<DeviceC> device;
    int deviceId = pkt.m_deviceId;
    DeviceTypeT deviceType = (DeviceTypeT) ((int) pkt.m_idBytes[3] >> 4);
    if(deviceType == DT_Unknown)
      deviceType = DT_MotorDriver;
    m_log->info("Handling device announcement {} {} Type:{} Id:{} ",pkt.m_uid[0],pkt.m_uid[1],ComsDeviceTypeToString(deviceType),(int) pkt.m_deviceId);
    ServoUpdateTypeT updateType = SUT_Updated;
    // Check device id is
    if(deviceId != 0) {
      // Check device ids match
      device = DeviceEntry(deviceId);
      // If no entry found.
      if(!device) {
        assert(deviceId <= 255 && deviceId >= 0);

        // Add entry to table with existing id.
        std::lock_guard<std::mutex> lock(m_mutexDevices);
        while(m_devices.size() <= deviceId)
          m_devices.push_back(nullptr);
        // Look through things previously flagged as unassigned.
        for(int i = 0;i < m_unassignedDevices.size();i++) {
          if(m_unassignedDevices[i]->HasUID(pkt.m_uid[0],pkt.m_uid[1])) {
            // Found it!
            device = m_unassignedDevices[i];
            m_unassignedDevices.erase(m_unassignedDevices.begin() + i);
            break;
          }
        }
        if(!device)
          device = MakeDevice(deviceId,pkt);
        m_devices[deviceId] = device;
        updateType = SUT_Add;
      } else {
        // Check existing device id matches
        if(!device->HasUID(pkt.m_uid[0],pkt.m_uid[1])) {
          // Conflicting id's detected.
          deviceId = 0; // Treat device as if it is unassigned.
          if(m_deviceManagerMode == DMM_DeviceManager)
            m_coms->SendSetDeviceId(0,pkt.m_uid[0],pkt.m_uid[1]); // Stop it using the conflicting address.
          m_log->error(
             "Conflicting ids detected for device {}, with uid {}-{} and existing device {}-{} '{}' ",
             (int) pkt.m_deviceId,pkt.m_uid[0],pkt.m_uid[1],
             device->UId1(),device->UId2(),device->DeviceName()
             );
        }
      }
    }
    if(deviceId == 0) {
      // Look through devices for a matching id.
      std::lock_guard<std::mutex> lock(m_mutexDevices);
      // 0 is a reserved id, so start from 1
      for(int i = 1;i < m_devices.size();i++) {
        if(m_devices[i] && m_devices[i]->HasUID(pkt.m_uid[0],pkt.m_uid[1])) {
          deviceId = i;
          device = m_devices[i];
        }
      }
      // No matching id found ?
      if(!device && m_deviceManagerMode == DMM_DeviceManager) {
        device = MakeDevice(0,pkt);
        m_timeLastUnassignedUpdate = std::chrono::steady_clock::now();
        for(auto &a : m_unassignedDevices) {
          if(a->HasUID(pkt.m_uid[0],pkt.m_uid[1])) {
            return ; // Already in list.
          }
        }
        m_unassignedDevices.push_back(device);
        return ;
      }
    }
    assert(device || m_deviceManagerMode != DMM_DeviceManager);
    if(device) {
      device->HandlePacketAnnounce(pkt,m_deviceManagerMode == DMM_DeviceManager);
      DeviceStatusUpdate(device.get(),updateType);
    }
  }


  //! Monitor thread
  void DogBotAPIC::RunMonitor()
  {
    m_log->info("Running monitor. Manager mode: {} ",((m_deviceManagerMode == DMM_DeviceManager) ? "master" : "client"));
    //logger->info("Starting bark");

    if(!m_coms) {
      m_log->error("No coms object, aborting monitor. ");
      return ;
    }

    CallbackSetC callbacks;

    callbacks += m_coms->SetHandler(
        CPT_Pong, [this](const uint8_t *data,int size) mutable
    {
      struct PacketPingPongC *pkt = (struct PacketPingPongC *) data;
      m_log->info("Got pong from {} ",(int) pkt->m_deviceId);
    });

    callbacks += m_coms->SetHandler(
        CPT_ServoReport,
             [this](const uint8_t *data,int size) mutable
        {
          if(size != sizeof(struct PacketServoReportC)) {
            m_log->error("Unexpected 'ServoReport' packet length {} ",size);
            return;
          }
          const PacketServoReportC *pkt = (const PacketServoReportC *) data;
          std::shared_ptr<ServoC> device = ServoEntry(pkt->m_deviceId);
          if(!device)
            return ;
          if(device->HandlePacketServoReport(*pkt)) {
            DeviceStatusUpdate(device.get(),SUT_Updated);
          }
        }
    );

    callbacks += m_coms->SetHandler(
          CPT_AnnounceId,
           [this](const uint8_t *data,int size) mutable
            {
              if(size != sizeof(struct PacketDeviceIdC)) {
                m_log->error("Unexpected 'AnnounceId' packet length {} ",size);
                return;
              }
              const PacketDeviceIdC *pkt = (const PacketDeviceIdC *) data;
              HandlePacketAnnounce(*pkt);
            }
           );

    callbacks += m_coms->SetHandler(
        CPT_ReportParam,
                       [this](const uint8_t *data,int size) mutable
                        {
                          if(size < sizeof(struct PacketParamHeaderC)) {
                            m_log->error("'ReportParam' packet to short {} ",size);
                            return;
                          }
                          const PacketParam8ByteC *pkt = (const PacketParam8ByteC *) data;
                          // We can only deal with devices after they've been allocated an id.
                          std::shared_ptr<DeviceC> device = DeviceEntry(pkt->m_header.m_deviceId);
                          if(!device)
                            return ;
                          if(device->HandlePacketReportParam(*pkt)) {
                            DeviceStatusUpdate(device.get(),SUT_Updated);
                          }

                        }
                       );

    callbacks += m_coms->SetHandler(CPT_Error,
                       [this](const uint8_t *data,int size) mutable
                        {
                          if(size < sizeof(struct PacketErrorC)) {
                            m_log->error("'Error' packet to short {} ",size);
                            return;
                          }
                          const PacketErrorC *pkt = (const PacketErrorC *) data;

                          m_log->error("Received packet error code from device {} : {} ({}) Packet:{} ({}) Arg:{} ",
                                       (int) pkt->m_deviceId,
                                       DogBotN::ComsErrorTypeToString((enum ComsErrorTypeT)pkt->m_errorCode),
                                       (int) pkt->m_errorCode,
                                       DogBotN::ComsPacketTypeToString((enum ComsPacketTypeT) pkt->m_causeType),(int) pkt->m_causeType,
                                       (int) pkt->m_errorData);
                        }
                       );

    callbacks += m_coms->SetHandler(CPT_IMU,
                       [this](const uint8_t *data,int size) mutable
                        {
                          if(size != sizeof(PacketIMUC)) {
                            m_log->error("'IMU' packet unexpected length {} ",size);
                            return ;
                          }
                          struct PacketIMUC *pkt = (struct PacketIMUC *) data;
                          std::shared_ptr<DeviceC> device = DeviceEntry(pkt->m_deviceId);
                          if(!device)
                            return ;
                          DeviceC *devPtr = device.get();
                          DeviceIMUC *anIMU = dynamic_cast<DeviceIMUC *>(devPtr);
                          if(anIMU == 0) {
                            m_log->error("IMU Packet from non-IMU device. {} ({}) ",(int) pkt->m_deviceId,typeid(*devPtr).name());
                            return;
                          }
                          if(anIMU->HandlePacketIMU(pkt)) {
                            DeviceStatusUpdate(device.get(),SUT_Updated);
                          }
                        });

    callbacks += m_coms->SetHandler(CPT_Message,
                                    [this](const uint8_t *data,int size) mutable
                                     {
                                       if(size < 2) {
                                         m_log->info("Empty message received. ");
                                         return ;
                                       }
                                       char buff[256];
                                       memcpy(buff,&data[2],size-2);
                                       buff[size-2] = 0;
                                       m_log->info("Message from {} : {} ",(int) data[1],buff);
                                     });

    callbacks += m_coms->SetHandler(CPT_EmergencyStop,
                                    [this](const uint8_t *data,int size) mutable
                                     {
                                       PacketEmergencyStopC *es =  (struct PacketEmergencyStopC *) data;
                                       if(size != sizeof(PacketEmergencyStopC)) {
                                         m_log->error("Unexpected emergency stop packet size {} ",size);
                                         return ;
                                       }
                                       if(m_emergencyStopFlags[es->m_deviceId]) // Already seen message?
                                         return ;
                                       m_emergencyStopFlags[es->m_deviceId] = true;
                                       m_log->error("Got emergency stop from device {}, cause type {} ({}) ",es->m_deviceId,ComsStateChangeSource((enum StateChangeSourceT) es->m_cause),es->m_cause);
                                     });

    while(!m_terminate)
    {
      //m_log->info("State. {} ",(int) m_driverState);
      switch(m_driverState)
      {
        case DS_Init: {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          m_driverState = DS_NoConnection;
        } break;
        case DS_NoConnection: {
          if(!m_coms->IsReady()) {
            if(m_deviceName == "none") {
              std::this_thread::sleep_for(std::chrono::seconds(2));
              break;
            }
            if(m_manageComs && !m_coms->Open(m_deviceName.c_str())) {
              m_log->warn("Failed to open coms channel. ");
              std::this_thread::sleep_for(std::chrono::seconds(2));
              break;
            }
            if(!m_coms->IsReady()) {
              std::this_thread::sleep_for(std::chrono::seconds(1));
              break;
            }
          }
          m_driverState = DS_Connected;
          // Send out a device query.
          for(int i = 0;i < 3;i++)
            m_coms->SendSync();
          m_coms->SendPing(0); // Find out what we're connected to.
          m_coms->SendQueryDevices();
        }
        // no break
        case DS_Connected: {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          if(!m_coms->IsReady()) {
            m_driverState = DS_NoConnection;
            break;
          }
          auto now = std::chrono::steady_clock::now();
          //m_coms->SendPing(0); // Find out what we're connected to.
          bool unassignedDevicesFound = false;
          std::vector<std::shared_ptr<DeviceC> > devicesList;
          {
            std::lock_guard<std::mutex> lock(m_mutexDevices);
            devicesList = m_devices;
            if(m_unassignedDevices.size() > 0) {
              std::chrono::duration<double> elapsed_seconds = now-m_timeLastUnassignedUpdate;
              //std::cerr << "Time: " << elapsed_seconds.count() << std::endl;
              unassignedDevicesFound = (elapsed_seconds.count() > 0.5);
            }
          }
          for(auto &a : devicesList) {
            if(!a)
              continue;
            if(a->UpdateTick(now)) {
              DeviceStatusUpdate(a.get(),SUT_Updated);
            }
          }
          if(unassignedDevicesFound)
            ProcessUnassignedDevices();
        } break;
        case DS_Calibrated:
        case DS_Error:
          std::this_thread::sleep_for(std::chrono::seconds(1));
          break;
      }
    }

    m_log->debug("Exiting monitor. ");
  }

  void DogBotAPIC::ProcessUnassignedDevices()
  {
    ServoUpdateTypeT op = SUT_Add;
    std::vector<DeviceC *> updated;
    {
      std::lock_guard<std::mutex> lock(m_mutexDevices);
      while(m_unassignedDevices.size() > 0) {
        auto device = m_unassignedDevices.back();
        m_unassignedDevices.pop_back();
        // Has device been an assigned an id after
        // it was initially queued ?
        if(device->Id() != 0) {
          // A bit of paranoia to make sure things aren't getting confused.
          assert(m_devices.size() > device->Id());
          assert(m_devices[device->Id()] == device);
          continue;
        }
        int deviceId = 0;
        // 0 is a reserved id, so start from 1
        for(int i = 1;i < m_devices.size();i++) {
          if(!m_devices[i]) {
            deviceId = i;
            op = SUT_Updated;
            break;
          }
        }
        if(deviceId == 0) {
          deviceId = m_devices.size();
          m_devices.push_back(device);
        } else {
          m_devices[deviceId] = device;
        }
        m_log->info("Adding new device {} of type '{}' named '{}' ",deviceId,device->DeviceType(),device->DeviceName());
        device->SetId(deviceId);
        updated.push_back(device.get());
        m_coms->SendSetDeviceId(deviceId,device->UId1(),device->UId2());
      }
    }
    for(auto a : updated)
      DeviceStatusUpdate(a,op);
  }


  //! Tell all servos to hold the current position
  void DogBotAPIC::DemandHoldPosition()
  {
    size_t deviceCount = 0;
    {
      std::lock_guard<std::mutex> lock(m_mutexDevices);
      deviceCount = m_devices.size();
    }
    for(int i = 0;i < deviceCount;i++) {
      std::shared_ptr<ServoC> servo;
      {
        std::lock_guard<std::mutex> lock(m_mutexDevices);
        deviceCount = m_devices.size();
        if(i < deviceCount)
          servo = std::dynamic_pointer_cast<ServoC>(m_devices[i]);
      }
      if(!servo || servo->Id() == 0)
        continue;
      m_coms->SendSetParam(servo->Id(),CPI_PWMMode,CM_Position);
      servo->DemandPosition(servo->Position(),servo->DefaultPositionTorque(),servo->PositionReference());
    }
    // Make sure
  }

  void DogBotAPIC::ResetAll()
  {
    m_coms->SendSetParam(0,CPI_ControlState,CS_StartUp);
    for(int i = 0;i < m_emergencyStopFlags.size();i++)
      m_emergencyStopFlags[i] = false;
  }

  //! Request all controllers go into low power mode
  void DogBotAPIC::LowPowerAll()
  {
    m_coms->SendSetParam(0,CPI_ControlState,CS_Standby);
    for(int i = 0;i < m_emergencyStopFlags.size();i++)
      m_emergencyStopFlags[i] = false;
  }

  void DogBotAPIC::RefreshAll()
  {
    size_t deviceCount = 0;
    {
      std::lock_guard<std::mutex> lock(m_mutexDevices);
      deviceCount = m_devices.size();
    }
    for(int i = 0;i < deviceCount;i++) {
      std::shared_ptr<DeviceC> dev;
      {
        std::lock_guard<std::mutex> lock(m_mutexDevices);
        deviceCount = m_devices.size();
        if(i < deviceCount)
          dev = m_devices[i];
      }
      if(!dev || dev->Id() == 0)
        continue;
      dev->QueryRefresh();
    }

  }

  //! Get servo entry by id
  std::shared_ptr<ServoC> DogBotAPIC::GetServoById(int id)
  {
    if(id < 0)
      return std::shared_ptr<ServoC>();
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    if(id >= m_devices.size())
      return std::shared_ptr<ServoC>();
    return std::dynamic_pointer_cast<ServoC>(m_devices[id]);
  }

  //! Get joint entry by name
  std::shared_ptr<JointC> DogBotAPIC::GetJointById(int id)
  {
    return std::shared_ptr<JointC>(GetServoById(id));
  }

  //! Get device entry by name
  std::shared_ptr<DeviceC> DogBotAPIC::GetDeviceByName(const std::string &name)
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    for(auto &a : m_devices) {
      if(a && a->DeviceName() == name)
        return a;
    }
    return std::shared_ptr<DeviceC>();
  }

  //! Get servo entry by name
  std::shared_ptr<JointC> DogBotAPIC::GetJointByName(const std::string &name)
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    for(auto &a : m_devices) {
      if(a && a->DeviceName() == name)
        return std::dynamic_pointer_cast<JointC>(a);
    }
    for(auto &a : m_joints) {
      if(a && a->Name() == name)
        return a;
    }
    return std::shared_ptr<JointC>();
  }

  //! Get kinematics for leg by name
  std::shared_ptr<LegKinematicsC> DogBotAPIC::LegKinematicsByName(const std::string &name)
  {
    return m_dogBotKinematics.LegKinematicsByName(name);
  }


  //! Get list of configured servos
  std::vector<std::shared_ptr<ServoC> > DogBotAPIC::ListServos()
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    std::vector<std::shared_ptr<ServoC> > servos;
    for(auto &a : m_devices) {
      std::shared_ptr<ServoC> ptr = std::dynamic_pointer_cast<ServoC>(a);
      if(ptr)
        servos.push_back(ptr);
    }
    return servos;
  }


  //! Get list of joints
  std::vector<std::shared_ptr<JointC> > DogBotAPIC::ListJoints()
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    return m_joints;
  }


}
