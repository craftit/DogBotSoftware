
#include "dogbot/DogBotAPI.hh"
#include "dogbot/protocol.h"
#include "dogbot/ComsSerial.hh"
#include "dogbot/ComsZMQClient.hh"
#include "dogbot/ComsUSB.hh"
#include "dogbot/Joint4BarLinkage.hh"
#include "dogbot/JointRelative.hh"
#include <fstream>
#include <chrono>
#include <mutex>
#include <cassert>
#include <memory>
#include <exception>
#include <stdexcept>

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
    case CS_LowPower: return "Low Power";
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
    }
    printf("Unexpected packet type %d",(int)packetType);
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

      case CPI_FINAL: return CPIT_Invalid;
    }

    printf("Unexpected parameter index %d for type query",(int)paramIndex);
    return CPIT_Unknown;
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
    std::cerr << "Got prefix '" << prefix << "' " << std::endl;
    if(prefix == "tcp" || prefix == "udp") {
      m_coms = std::make_shared<ComsZMQClientC>(name);
      if(m_deviceManagerMode == DMM_Auto)
        m_deviceManagerMode = DMM_ClientOnly;
    } else  if(name == "local") {
      m_coms = std::make_shared<ComsZMQClientC>("tcp://127.0.0.1:7200");
      if(m_deviceManagerMode == DMM_Auto)
        m_deviceManagerMode = DMM_ClientOnly;
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
    if(!m_coms->Open(name.c_str()))
      return false;
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



  //! Load a configuration file
  bool DogBotAPIC::LoadConfig(const std::string &configFile)
  {
    if(configFile.empty())
      return false;
    try {
      std::ifstream confStrm(configFile,std::ifstream::binary);

      if(!confStrm) {
        m_log->error("Failed to open configuration file '{}' ",configFile);
        return false;
      }

      Json::Value rootConfig;
      confStrm >> rootConfig;
      Json::Value kinematicsList = rootConfig["kinematics"];
      if(!kinematicsList.isNull()) {
        for(int i = 0;i < kinematicsList.size();i++) {
          Json::Value kinConf = kinematicsList[i];
          std::string name = kinConf.get("name","default").asString();
          std::shared_ptr<LegKinematicsC> kin;

          std::lock_guard<std::mutex> lock(m_mutexKinematics);
          for(auto &a : m_legKinematics) {
            if(a->Name() == name) {
              kin = a;
              break;
            }
          }
          if(!kin) {
            kin = std::make_shared<LegKinematicsC>(kinConf);
            m_legKinematics.push_back(kin);
          } else {
            kin->ConfigureFromJSON(kinConf);
          }
        }
      }

      Json::Value deviceList = rootConfig["devices"];
      if(!deviceList.isNull()) {
        ServoUpdateTypeT op = SUT_Updated;
        for(int i = 0;i < deviceList.size();i++) {
          Json::Value deviceConf = deviceList[i];
          std::string deviceType = deviceConf.get("type","").asString();
          if(!deviceType.empty() && deviceType != "servo" ) {
            std::shared_ptr<JointC> joint = MakeJoint(deviceType);
            joint->ConfigureFromJSON(*this,deviceConf);
            m_joints.push_back(joint);
            continue;
          }
          std::shared_ptr<ServoC> device;
          int deviceId = 0;
          uint32_t uid1 = deviceConf.get("uid1",0u).asInt();
          uint32_t uid2 = deviceConf.get("uid2",0u).asInt();
          int reqId = deviceConf.get("deviceId",0u).asInt();
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
                }
              } else {
                while(m_devices.size() < reqId)
                  m_devices.push_back(std::shared_ptr<ServoC>());
                deviceId = reqId;
                device = std::make_shared<ServoC>(m_coms,deviceId);
                m_devices.push_back(device);
                op = SUT_Add;
              }
              if(deviceId > 0) {
                device = std::make_shared<ServoC>(m_coms,deviceId);
                m_devices[deviceId] = device;
              }
            }
            // Still no luck... just append it.
            if(deviceId == 0) {
              deviceId = (int) m_devices.size();
              device = std::make_shared<ServoC>(m_coms,deviceId);
              m_devices.push_back(device);
              op = SUT_Add;
            }
          }
          assert(device);
          if(device) {
            device->ConfigureFromJSON(*this,deviceConf);
            ServoStatusUpdate(device.get(),op);
            if(device->IsExported())
              m_joints.push_back(device);
          }
        }
      }
    } catch(std::exception &ex) {
      m_log->error("Failed load configuration file {}", ex.what());
      return false;
    }

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
        deviceList[index++] = a->ConfigAsJSON();
      }
      for(auto &a : m_joints) {
        if(!a)
          continue;
        if(a->JointType() == "servo")
          continue;
        deviceList[index++] = a->ConfigAsJSON();
      }
      rootConfig["devices"] = deviceList;

      Json::Value kinematicsList;
      index = 0;
      for(auto &a : m_legKinematics) {
        if(!a)
          continue;
        a->ConfigAsJSON();
        kinematicsList[index++] = a->ConfigAsJSON();
      }

      rootConfig["kinematics"] = kinematicsList;
    }



    confStrm << rootConfig;

    return true;
  }


  //! Issue an update notification
  void DogBotAPIC::ServoStatusUpdate(JointC *id,ServoUpdateTypeT op)
  {
    for(auto &a : m_jointStatusCallbacks.Calls()) {
      if(a) a(id,op);
    }
  }

  //! Access device id, create entry if needed
  std::shared_ptr<ServoC> DogBotAPIC::DeviceEntry(int deviceId)
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);

    // The device clearly exists, so make sure there is an entry.
    if(deviceId >= m_devices.size() || deviceId < 0)
      return std::shared_ptr<ServoC>();
    return m_devices[deviceId];
  }

  void DogBotAPIC::HandlePacketAnnounce(const PacketDeviceIdC &pkt)
  {
    std::shared_ptr<ServoC> device;
    int deviceId = pkt.m_deviceId;
    m_log->info("Handling device announcement {} {}   Id:{} ",pkt.m_uid[0],pkt.m_uid[1],(int) pkt.m_deviceId);
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
          device = std::make_shared<ServoC>(m_coms,deviceId,pkt);
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
             device->UId1(),device->UId2(),device->Name()
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
        device = std::make_shared<ServoC>(m_coms,0,pkt);
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
      ServoStatusUpdate(device.get(),updateType);
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
          std::shared_ptr<ServoC> device = DeviceEntry(pkt->m_deviceId);
          if(!device)
            return ;
          if(device->HandlePacketServoReport(*pkt)) {
            ServoStatusUpdate(device.get(),SUT_Updated);
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
                          std::shared_ptr<ServoC> device = DeviceEntry(pkt->m_header.m_deviceId);
                          if(!device)
                            return ;
                          if(device->HandlePacketReportParam(*pkt)) {
                            ServoStatusUpdate(device.get(),SUT_Updated);
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

                          std::shared_ptr<ServoC> device = DeviceEntry(pkt->m_deviceId);
                          if(!device)
                            return ;
#if 0
                          if(device->HandlePacketReportParam(*pkt)) {
                            ServoStatusUpdate(pkt->m_header.m_deviceId,SUT_Updated);
                          }
#endif

                        }
                       );

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
          m_coms->SendQueryDevices();
          m_coms->SendPing(0); // Find out what we're connected to.
        }
        // no break
        case DS_Connected: {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          if(!m_coms->IsReady()) {
            m_driverState = DS_NoConnection;
            break;
          }
          auto now = std::chrono::steady_clock::now();
          bool unassignedDevicesFound = false;
          std::vector<std::shared_ptr<ServoC> > devicesList;
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
              ServoStatusUpdate(a.get(),SUT_Updated);
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
    std::vector<JointC *> updated;
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
        m_log->info("Adding new device '{}' ",deviceId);
        device->SetId(deviceId);
        updated.push_back(device.get());
        m_coms->SendSetDeviceId(deviceId,device->UId1(),device->UId2());
      }
    }
    for(auto a : updated)
      ServoStatusUpdate(a,op);
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
          servo = m_devices[i];
      }
      if(!servo || servo->Id() == 0)
        continue;
      m_coms->SendSetParam(servo->Id(),CPI_PWMMode,CM_Position);
      servo->DemandPosition(servo->Position(),servo->DefaultPositionTorque());
    }
    // Make sure
  }

  void DogBotAPIC::ResetAll()
  {
    m_coms->SendSetParam(0,CPI_ControlState,CS_StartUp);

  }

  //! Request all controllers go into low power mode
  void DogBotAPIC::LowPowerAll()
  {
    m_coms->SendSetParam(0,CPI_ControlState,CS_LowPower);
  }

  void DogBotAPIC::RefreshAll()
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
          servo = m_devices[i];
      }
      if(!servo || servo->Id() == 0)
        continue;
      servo->QueryRefresh();
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
    return m_devices[id];
  }

  //! Get servo entry by name
  std::shared_ptr<JointC> DogBotAPIC::GetJointByName(const std::string &name)
  {
    std::lock_guard<std::mutex> lock(m_mutexKinematics);
    for(auto &a : m_devices) {
      if(a && a->Name() == name)
        return a;
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
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    for(auto &a : m_legKinematics) {
      if(a && a->Name() == name)
        return a;
    }
    return std::shared_ptr<LegKinematicsC>();
  }


  //! Get list of configured servos
  std::vector<std::shared_ptr<ServoC> > DogBotAPIC::ListServos()
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    return m_devices;
  }


  //! Get list of joints
  std::vector<std::shared_ptr<JointC> > DogBotAPIC::ListJoints()
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    return m_joints;
  }

}
