#ifndef DOGBOG_PROTOCOL_HEADER
#define DOGBOG_PROTOCOL_HEADER 1


#define BMCUSB_DATA_OUT_EP          1
#define BMCUSB_DATA_IN_EP           2

#ifdef __cplusplus
extern "C" {
#endif

  // These messages are ordered by decreasing priority.

  enum ComsPacketTypeT
  {
    CPT_NoOp    =  0, // No op.
    CPT_EmergencyStop =  1, // Set parameter (Used to for emergency stop)
    CPT_SyncTime      =  2, // Sync time across controllers.
    CPT_Error         =  3, // Error report
    CPT_SetParam      =  4, // Set parameter
    CPT_Servo         =  5, // Servo control position
    CPT_ServoReport   =  6, // Report servo position
    CPT_ReportParam   =  7, // Report parameter
    CPT_ReadParam     =  8, // Read parameter
    CPT_Pong          =  9, // Ping reply.
    CPT_Ping          = 10, // Ping request
    CPT_AnnounceId    = 11, // Query connected devices
    CPT_QueryDevices  = 12, // Query connected devices
    CPT_SetDeviceId   = 13, // Set device id
    CPT_SaveSetup     = 14, // Save setup to eeprom
    CPT_LoadSetup     = 15, // Load setup from eeprom
    CPT_CalZero       = 16, // Set current position as calibrated zero.
    CPT_Sync          = 17, // Sync data stream
    CPT_PWMState      = 18, // PWM State. Packet holding internal controller data.
    CPT_BridgeMode    = 19 // Enable bridge mode
  };


  enum ComsErrorTypeT {
    CET_UnknownPacketType = 0,
    CET_UnexpectedPacketSize = 1,
    CET_ParameterOutOfRange = 2,
    CET_CANTransmitFailed = 3,
    CET_InternalError = 4,
    CET_MotorNotRunning = 5 // Command requires motor to be running. (Like CalZero)
  };

  enum FaultCodeT {
    FC_Ok = 0,
    FC_Unknown = 1,
    FC_UnderVoltage = 2,
    FC_NoSensor = 3,
    FC_NoMotor = 4,
    FC_CalibrationFailed = 5,
    FC_OverTemprature = 6,
    FC_DriverFault = 7,
    FC_OverVoltage = 8,
    FC_PositionLost = 9,
    FC_InternalTiming = 10,
    FC_InternalStoreFailed = 11,
    FC_Internal5VRailOutOfRange = 12,
    FC_Internal = 13,
    FC_MotorResistanceOutOfRange = 14,
    FC_MotorInducetanceOutOfRange = 15,
    FC_InternalUSB = 14
  };


  enum DeviceTypeT
  {
    DT_Unknown = 0,
    DT_SystemController = 1,
    DT_MotorDriver = 2
  };

  enum ComsParameterIndexT
  {
    CPI_DeviceType      = 0,
    CPI_FirmwareVersion = 1,
    CPI_PWMState        = 2,
    CPI_PWMMode         = 3,
    CPI_PWMFullReport   = 4,
    CPI_CANBridgeMode   = 5,
    CPI_BoardUID        = 6,
    CPI_TIM1_SR         = 7,
    CPI_VSUPPLY         = 8,
    CPI_ControlState    = 9,
    CPI_HomedState     = 10,
    CPI_PositionRef     = 11,
    CPI_CalibrationOffset = 12,
    CPI_FaultCode       = 13,
    CPI_Indicator       = 14,
    CPI_DriveTemp       = 15,
    CPI_MotorTemp       = 16,
    CPI_OtherJoint      = 17,
    CPI_OtherJointGain  = 18,
    CPI_OtherJointOffset= 19,
    CPI_DebugIndex      = 20,
    CPI_MotorResistance = 21,
    CPI_MotorInductance = 22,
    CPI_MotorOffsetVoltage = 23,
    CPI_MotorIGain      = 24,
    CPI_MotorPGain      = 25,
    CPI_PhaseVelocity   = 26,
    CPI_VelocityPGain   = 27,
    CPI_VelocityIGain   = 28,
    CPI_DemandPhaseVelocity = 29,
    CPI_VelocityLimit   = 30,
    CPI_PositionGain   = 31,

    CPI_DRV8305         = 32,
    CPI_DRV8305_01      = 32,
    CPI_DRV8305_02      = 33,
    CPI_DRV8305_03      = 34,
    CPI_DRV8305_04      = 35,
    CPI_DRV8305_05      = 36,

    CPI_5VRail          = 37,
    CPI_AuxPower        = 38,
    CPI_MaxCurrent      = 39,
    CPI_homeIndexPosition = 40,
    CPI_HallSensors      = 41,
    CPI_MinSupplyVoltage = 42,
    CPI_USBPacketDrops   = 43,
    CPI_USBPacketErrors  = 44,
    CPI_FaultState       = 45,
    CPI_IndexSensor      = 46,

    CPI_ANGLE_CAL       = 48,  // 12 Values
    CPI_ANGLE_CAL_0     = 0x30,
    CPI_ANGLE_CAL_1     = 0x31,
    CPI_ANGLE_CAL_2     = 0x32,
    CPI_ANGLE_CAL_3     = 0x33,
    CPI_ANGLE_CAL_4     = 0x34,
    CPI_ANGLE_CAL_5     = 0x35,
    CPI_ANGLE_CAL_6     = 0x36,
    CPI_ANGLE_CAL_7     = 0x37,
    CPI_ANGLE_CAL_8     = 0x38,
    CPI_ANGLE_CAL_9     = 0x39,
    CPI_ANGLE_CAL_10    = 0x3A,
    CPI_ANGLE_CAL_11    = 0x3B,
    CPI_ANGLE_CAL_12    = 0x3C,
    CPI_ANGLE_CAL_13    = 0x3D,
    CPI_ANGLE_CAL_14    = 0x3E,
    CPI_ANGLE_CAL_15    = 0x3F,
    CPI_ANGLE_CAL_16    = 0x40,
    CPI_ANGLE_CAL_17    = 0x41,
    CPI_FINAL           = 0xff
  };

  /* Control state.
   *
   */

  enum ControlStateT {
    CS_StartUp       = 0,
    CS_Standby       = 1,
    CS_LowPower      = 2,
    CS_Ready         = 3,
    CS_EmergencyStop = 4,
    CS_SelfTest      = 5,
    CS_FactoryCalibrate = 6,
    CS_Home          = 7,
    CS_Fault         = 8,
    CS_Teach         = 9,
    CS_Diagnostic    = 10
  };



  enum PWMControlDynamicT {
    CM_Off      = 0,
    CM_Brake    = 1,
    CM_Torque   = 2,
    CM_Velocity = 3,
    CM_Position = 4,
    CM_Fault    = 5,
    CM_Final    = 6
  } ;

  /* Reference coordinates for position reports.
   *
   * Relative  : Is relative to an arbitrary position at power up
   * Absolute  : Once position is calibrated
   * OtherJoint: Position relative to another joint
   * */

  enum PositionReferenceT {
    PR_Relative = 0,
    PR_Absolute = 1,
    PR_OtherJointRelative = 2,
    PR_OtherJointAbsolute = 3,
  };



  /* Motion calibration state
   *
   * Uncalibrated : Absolute position unknown
   *
   */

  enum MotionHomedStateT {
    MHS_Lost = 0,
    MHS_Measuring    = 1,
    MHS_Homed   = 2
  };

  struct PacketErrorC {
    uint8_t m_packetType;
    uint8_t m_deviceId;
    uint8_t m_errorCode;
    uint8_t m_causeType;
    uint8_t m_errorData;
  } __attribute__((packed));

  struct PacketBridgeModeC {
    uint8_t m_packetType;
    uint8_t m_enable;
  };

  struct PacketPingPongC {
    uint8_t m_packetType;
    uint8_t m_deviceId;
  } __attribute__((packed));


  struct PacketReadParamC {
    uint8_t m_packetType;
    uint8_t m_deviceId;   // Target device
    uint16_t m_index;
  } __attribute__((packed));

  struct PacketParamHeaderC {
    uint8_t m_packetType; //  CPT_SetParam or CPT_ReportParam
    uint8_t m_deviceId;   // Target device
    uint16_t m_index;
  } __attribute__((packed));


  union BufferTypeT
  {
    uint8_t  uint8[8];
    uint16_t uint16[4];
    uint32_t uint32[2];
    float float32[2];
  };

  struct PacketParamC {
    struct PacketParamHeaderC m_header;
    uint16_t m_data;
  } __attribute__((packed));

  struct PacketParam8ByteC {
    struct PacketParamHeaderC m_header;
    union BufferTypeT m_data;
  } __attribute__((packed));


  struct PacketPWMStateC {
    uint8_t m_packetType;
    uint16_t m_tick;
    uint16_t m_hall[3];
    uint16_t m_curr[3];
    uint16_t m_angle;
  } __attribute__((packed));

  struct PacketServoC {
    uint8_t m_packetType; // CPT_ServoAbs / CPT_ServoRel
    uint8_t m_deviceId;
    uint8_t m_mode;       //
    int16_t m_position;   // Or velocity.
    uint16_t m_torqueLimit;
  } __attribute__((packed));

  struct PacketServoReportC {
    uint8_t m_packetType; // CPT_ServoAbs / CPT_ServoRel
    uint8_t m_deviceId;
    uint8_t m_mode;
    uint8_t m_timestamp;
    int16_t m_position;
    int16_t m_torque;
  } __attribute__((packed));

  struct PacketDeviceIdC {
    uint8_t m_packetType; // CPT_AnnounceId or CPT_SetDeviceId
    uint8_t m_deviceId;
    uint32_t m_uid[2];
  } __attribute__((packed)) ;

  struct PacketCalZeroC {
    uint8_t m_packetType; // CPT_CalZero
    uint8_t m_deviceId;
  } __attribute__((packed));

  struct PacketStoredConfigC {
    uint8_t m_packetType; // CPT_SaveSetup / CPT_LoadSetup
    uint8_t m_deviceId;
  }  __attribute__((packed));

#ifdef __cplusplus
}
#endif

#endif
