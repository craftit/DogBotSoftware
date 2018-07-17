#ifndef DOGBOG_PROTOCOL_HEADER
#define DOGBOG_PROTOCOL_HEADER 1


#define BMCUSB_DATA_OUT_EP          1
#define BMCUSB_DATA_IN_EP           2

#ifdef __cplusplus
extern "C" {
#endif

#define DOGBOT_FIRMWARE_VERSION 7

  /* Message packet types, see end of file for structs transmitted for each message type.
   * These messages are ordered by decreasing priority.
   */

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
    CPT_Sync          = 17, // Sync data stream, useful for finding packet boundaries when using over a byte serial channel.
    CPT_PWMState      = 18, // PWM State. Packet holding internal controller data.
    CPT_BridgeMode    = 19, // Enable bridge mode
    CPT_FlashCmdReset   = 20, // Status from a flash command
    CPT_FlashCmdResult  = 21, // Status from a flash command
    CPT_FlashChecksumResult = 22, // Generate a checksum
    CPT_FlashEraseSector = 23, // Erase a flash sector
    CPT_FlashChecksum    = 24, // Generate a checksum
    CPT_FlashData        = 25, // Data packet
    CPT_FlashWrite       = 26, // Write buffer
    CPT_FlashRead        = 27, // Read buffer and send it back
    CPT_IMU              = 28, // IMU Data packet
    CPT_Message          = 29, // Debug message packet
    CPT_Range            = 30, // Range sensor reading
    CPT_Final                  // Use to get count of known packet types.
  };


  /* Communication error codes.
   */

  enum ComsErrorTypeT {
    CET_UnknownPacketType = 0,
    CET_UnexpectedPacketSize = 1,
    CET_ParameterOutOfRange = 2,
    CET_CANTransmitFailed = 3,
    CET_InternalError = 4,
    CET_MotorNotRunning = 5, // Command requires motor to be running. (Like CalZero)
    CET_NotImplemented = 6,
    CET_BootLoaderUnexpectedState  = 7,
    CET_BootLoaderLostSequence = 8,
    CET_BootLoaderErase  = 9,
    CET_BootLoaderProtected = 10,
    CET_BootLoaderBusy  = 11,
    CET_BootLoaderWriteFailed = 12,
    CET_BootLoaderUnalignedAddress = 13,
    CET_UnavailableInCurrentMode = 14,
    CET_MotorDriverWarning = 15
  };

  /* Fault codes.
   *
   */

  enum FaultCodeT {
    FC_Ok = 0,
    FC_Unknown = 1,
    FC_UnderVoltage = 2,
    FC_NoSensor = 3,
    FC_NoMotor = 4,
    FC_CalibrationFailed = 5,
    FC_DriverOverTemperature = 6,
    FC_DriverFault = 7,
    FC_OverVoltage = 8,
    FC_PositionLost = 9,
    FC_InternalTiming = 10,
    FC_InternalStoreFailed = 11,
    FC_Internal5VRailOutOfRange = 12,
    FC_Internal = 13,
    FC_MotorResistanceOutOfRange = 14,
    FC_MotorInducetanceOutOfRange = 15,
    FC_InternalUSB = 16,
    FC_InternalCAN = 17,
    FC_FanOverCurrent = 18,
    FC_MotorOverTemperature = 19,
    FC_SensorOverCurrent = 20,
    FC_InvalidCommand = 21 //!< Triggered if we're asked to do something that isn't supported.
  };

  /* An identifier for the role of a device
   *
   */

  enum DeviceTypeT
  {
    DT_Unknown = 0,
    DT_PlatformManager = 1,
    DT_MotorDriver = 2,
    DT_BootLoader = 3,
    DT_IMU = 4
  };


  /* A table of known joint types
   *
   */

  enum JointRoleT {
    JR_Spare = 0,
    JR_FrontLeftKnee,
    JR_FrontLeftPitch,
    JR_FrontLeftRoll,
    JR_FrontRightKnee,
    JR_FrontRightPitch,
    JR_FrontRightRoll,
    JR_BackLeftKnee,
    JR_BackLeftPitch,
    JR_BackLeftRoll,
    JR_BackRightKnee,
    JR_BackRightPitch,
    JR_BackRightRoll
  };

  /* Parameter indices
   *
   */

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
    CPI_OtherJoint      = 17, // Obsolete
    CPI_OtherJointGain  = 18, // Obsolete
    CPI_OtherJointOffset= 19, // Obsolete
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
    // 38 - Free Was fan mode
    CPI_MaxCurrent      = 39,
    CPI_homeIndexPosition = 40,
    CPI_HallSensors      = 41,
    CPI_MinSupplyVoltage = 42,
    CPI_USBPacketDrops   = 43,
    CPI_USBPacketErrors  = 44,
    CPI_FaultState       = 45,
    CPI_IndexSensor      = 46,

    CPI_ANGLE_CAL       = 48,  // 18 Values
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

    CPI_CANPacketDrops   = 0x50,
    CPI_CANPacketErrors  = 0x51,
    CPI_MainLoopTimeout  = 0x52,
    CPI_JointRelative    = 0x53,
    CPI_FanTemperatureThreshold = 0x54,
    CPI_FanMode          = 0x55,
    CPI_FanState         = 0x56,

    CPI_SafetyMode       = 0x57,
    CPI_JointRole        = 0x58,
    CPI_EndStopEnable    = 0x59,
    CPI_EndStopStart     = 0x5A,
    CPI_EndStopStartBounce = 0x5B,
    CPI_EndStopFinal       = 0x5C,
    CPI_EndStopEndBounce = 0x5D,
    CPI_EndStopTargetBreakForce = 0x5E,
    CPI_EndStopLimitBreakForce = 0x5F,
    CPI_JointInertia     = 0x60,  // Obsolete.
    CPI_EndStopPhaseAngles = 0x61,

    CPI_ServoReportFrequency   = 0x62,
    CPI_PWMFrequency       = 0x63,
    CPI_MotionUpdatePeriod = 0x64,
    CPI_SupplyVoltageScale = 0x65,
    CPI_CurrentLimit       = 0x66,
    CPI_PlatformActivity   = 0x67, // Used for platform manager
    CPI_RequestedPlatformActivity = 0x68, // Used for platform manager

    CPI_FINAL           = 0xff
  };


  /* Parameter types.
   *
   */

  enum ComsParameterIndexTypeT
  {
    CPIT_Unknown,
    CPIT_Invalid,
    CPIT_Custom,
    CPIT_bool,
    CPIT_enum8,
    CPIT_uint8,
    CPIT_int8,
    CPIT_uint16,
    CPIT_int16,
    CPIT_uint16_3,
    CPIT_int32,
    CPIT_uint32,
    CPIT_uint32_2,
    CPIT_float32,
    CPIT_float32_2
  };

  /* Control state.
   *
   */

  enum ControlStateT {
    CS_StartUp       = 0, //!< Doing self test, then go to ready if safe to do so.
    CS_SafeStop      = 1, //!< Apply breaks for 20 seconds, then go to low power mode
    CS_Standby      = 2, //!< Reduced power consumption mode.
    CS_Ready         = 3, //!< Motor control loop running, ready for motion commands.
    CS_EmergencyStop = 4, //!< Motor in break state.
    CS_SelfTest      = 5, //!< Doing a self test.
    CS_FactoryCalibrate = 6, //!< Calibrating motor
    CS_Home          = 7, //!< Auto homing motor, NOT IMPLEMENTED
    CS_Fault         = 8, //!< Hardware or configuration fault detected.
    //CS_Teach         = 9, //!< Motor position monitored but no torque. NOT IMPLEMENTED
    CS_Diagnostic    = 10, //!< As CS_Ready, but with extra status reporting.
    CS_BootLoader    = 11, //!< Ready for firmware update
    CS_MotionCalibrate = 12 //!< Calibrating motion parameters of joint
  };

  /* Safety mode, this sets the behaviour if a fault is detected on a controller.
   *
   */

  enum SafetyModeT {
    SM_Unknown = 0,             /*!< Current mode is unknown. */
    SM_GlobalEmergencyStop = 1, /*!< If we enter fault condition send a global emergency stop. (default) */
    SM_MasterEmergencyStop = 2, /*!< As GlobalEmergencyStop, but also monitor the emergency stop switch.  */
    SM_LocalStop = 3            /*!< Only stop the local servo and report problem to control software.   */
  };

  /* Dynamics mode for the control loop.
   *
   */
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
   * */

  enum PositionReferenceT {
    PR_Relative = 0,
    PR_Absolute = 1
  };

  /* Fan management
   * This should be left on Auto unless testing
   */

  enum FanModeT {
    FM_Off = 0,
    FM_On  = 1,
    FM_Auto  = 2
  };

  /* Motion calibration state
   *
   * Lost : Absolute position unknown
   *
   */

  enum MotionHomedStateT {
    MHS_Lost       = 0,
    MHS_Measuring  = 1,
    MHS_Homed      = 2,
    MHS_SoftHomed= 3
  };

  /* Platform activities
   *
   */

  enum PlatformActivityT {
    PA_Passive,
    PA_Idle,
    PA_Bootloader,
    PA_Home,
    PA_Falling,
    PA_Stand,
    PA_Walking,
    PA_Shutdown,
    PA_ROS,
    PA_User
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
  } __attribute__((packed));

  struct PacketPingPongC {
    uint8_t m_packetType;
    uint8_t m_deviceId;
    uint16_t m_payload; // Generally used to id the packet.
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
    uint8_t  int8[8];
    int8_t   uint8[8];
    uint16_t uint16[4];
    int16_t  int16[4];
    uint32_t uint32[2];
    int32_t  int32[2];
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

  // Motion request packet.
  //
  // m_mode uses the following bits:
  //  Bit    0: If set the requested position is absolute relative to homed position, if clear then it is relative to motor power up.
  //  Bit    1: If set, then m_torque is expected required torque, if clear then it is a torque limit.
  //  Bits 2-5: Dynamic mode. See enum PWMControlDynamicT
  //  Bit  6-7: Unused.
  //
#define DOGBOT_PACKETSERVOMODE_ABSOLUTEPOSITION (1u<<0)
#define DOGBOT_PACKETSERVOMODE_DEMANDTORQUE     (1u<<1)
#define DOGBOT_PACKETSERVOMODE_DYNAMIC_BITOFFSET (2)
#define DOGBOT_PACKETSERVOMODE_DYNAMIC(x)       (((x) >> DOGBOT_PACKETSERVOMODE_DYNAMIC_BITOFFSET) & 0x07)
#define DOGBOT_PACKETSERVO_FLOATSCALE           (32767.0f)

  struct PacketServoC {
    uint8_t m_packetType; // Must be CPT_Servo
    uint8_t m_deviceId;   // Destination device for packet.
    uint8_t m_mode;       // See comment above.
    uint8_t m_timestamp;  // Incremented by one for each successive packet, used for detection dropped packets in a trajectory.
    int16_t m_demand;     // Position, Velocity or Torque depending on bits set in dynamic mode, in m_mode field.
    int16_t m_torque;     // Demand torque (for feed forward control) or torque limit, depending on bit 1 in mode.
  } __attribute__((packed));

#define DOGBOT_SERVOREPORTMODE_EMERGENCYSTOP (1u<<7)
#define DOGBOT_SERVOREPORTMODE_LIMITVELOCITY (1u<<6)
#define DOGBOT_SERVOREPORTMODE_LIMITTORQUE   (1u<<5)
#define DOGBOT_SERVOREPORTMODE_LIMITPOSITION (1u<<4)
#define DOGBOT_SERVOREPORTMODE_INDEXSENSOR   (1u<<3)
#define DOGBOT_SERVOREPORTMODE_POSITIONREF   (0x3u)

#define DOGBOT_SERVOREPORT_POSITIONRANGE (M_PI * 4.0)
#define DOGBOT_SERVOREPORT_VELOCITYRANGE (M_PI * 8.0)

  struct PacketServoReportC {
    uint8_t m_packetType; // CPT_ServoReport
    uint8_t m_deviceId;
    uint8_t m_mode;
    uint8_t m_timestamp;
    int16_t m_position;
    int16_t m_torque;
    int16_t m_velocity;
  } __attribute__((packed));

  struct PacketDeviceIdC {
    uint8_t m_packetType; // CPT_AnnounceId or CPT_SetDeviceId
    uint8_t m_deviceId;
    union {
      uint32_t m_uid[2];
      uint8_t m_idBytes[8];
    };
  } __attribute__((packed)) ;

  struct PacketCalZeroC {
    uint8_t m_packetType; // CPT_CalZero
    uint8_t m_deviceId;
  } __attribute__((packed));

  struct PacketStoredConfigC {
    uint8_t m_packetType; // CPT_SaveSetup / CPT_LoadSetup
    uint8_t m_deviceId;
  }  __attribute__((packed));

  struct PacketIMUC {
    uint8_t m_packetType;
    uint8_t m_deviceId;
    int16_t m_accel[3];
    int16_t m_gyro[3];
    int16_t m_rot[4];
  } __attribute__((packed));

  struct PacketRangeC {
    uint8_t m_packetType;
    uint8_t m_deviceId;
    uint8_t m_sensorId;
    int16_t m_range;
  } __attribute__((packed));

  enum StateChangeSourceT {
    SCS_UserRequest = 0,
    SCS_Internal = 1,
    SCS_Unknown = 2,
    SCS_Fault = 3,
    SCS_EStopLostComs = 4,
    SCS_EStopSwitch = 5,
    SCS_External    = 6
  };

  struct PacketEmergencyStopC {
    uint8_t m_packetType;
    uint8_t m_deviceId; //!< Device that issued it.
    uint8_t m_cause;    //!< Reason why it was issued, as StateChangeSourceT
  } __attribute__((packed));

  enum FlashOperationStatusT
  {
    FOS_Ok = 0,
    FOS_WriteComplete = 1,
    FOS_DataAck = 2,
    FOS_SequenceLost = 3,
    FOS_ProgrammingError = 4
  };

  enum BootLoaderStateT {
    BLS_Ready = 0,
    BLS_Disabled = 1,
    BLS_Checksum = 2,
    BLS_Write = 3,
    BLS_Read  = 4,
    BLS_Error = 5
  };

  struct PacketFlashResetC {
    uint8_t m_packetType;
    uint8_t m_deviceId;
    uint8_t m_enable;
  };

  struct PacketFlashResultC {
    uint8_t m_packetType;
    uint8_t m_deviceId;
    uint8_t m_rxSequence;
    uint8_t m_state;
    uint8_t m_result;
  };

  struct PacketFlashEraseC {
    uint8_t m_packetType;
    uint8_t m_deviceId;
    uint8_t m_sequenceNumber;
    uint32_t m_addr;
  };

  struct PacketFlashChecksumResultC {
    uint8_t m_packetType;
    uint8_t m_deviceId;
    uint8_t m_sequenceNumber;
    uint32_t m_sum;
  };

  struct PacketFlashChecksumC {
    uint8_t m_packetType;
    uint8_t m_deviceId;
    uint8_t m_sequenceNumber;
    uint32_t m_addr;
    uint16_t m_len;
  };

  struct PacketFlashDataC {
    uint8_t m_packetType;
    uint8_t m_deviceId;
    uint8_t m_sequenceNumber;
    uint8_t m_data[0];
  };

  /* Data packet with associated buffer.
   *
   */
  struct PacketFlashDataBufferC
  {
    struct PacketFlashDataC m_header;
    uint8_t m_data[7];
  };

  struct PacketFlashWriteC {
    uint8_t m_packetType;
    uint8_t m_deviceId;
    uint8_t m_sequenceNumber;
    uint32_t m_addr;
    uint16_t m_len;
  };

  struct PacketFlashReadC {
    uint8_t m_packetType;
    uint8_t m_deviceId;
    uint8_t m_sequenceNumber;
    uint32_t m_addr;
    uint16_t m_len;
  };


#ifdef __cplusplus
}
#endif

#endif
