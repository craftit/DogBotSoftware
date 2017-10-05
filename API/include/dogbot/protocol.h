#ifndef DOGBOG_PROTOCOL_HEADER
#define DOGBOG_PROTOCOL_HEADER 1

#ifdef __cplusplus
extern "C" {
#endif

  enum ComsPacketTypeT
  {
    CPT_Ping         =  0,  // Ping request
    CPT_Pong         =  1,  // Ping reply.
    CPT_Error        =  2,  // Error report
    CPT_Sync         =  3,  // Sync data stream
    CPT_ReadParam    =  4, // Read parameter
    CPT_SetParam     =  5, // Set parameter
    CPT_ReportParam  =  6, // Report parameter
    CPT_Servo        =  7, // Servo control position
    CPT_ServoReport  =  8, // Report servo position
    // 9 is unused.
    CPT_PWMState     = 10, // PWM State.
    CPT_QueryDevices = 11, // Query connected devices
    CPT_AnnounceId   = 12, // Query connected devices
    CPT_SetDeviceId  = 13  // Set device id
  };


  enum ComsErrorTypeT {
    CET_UnknownPacketType = 0,
    CET_UnexpectedPacketSize = 1,
    CET_ParameterOutOfRange = 2,
    CET_CANTransmitFailed = 3,
    CET_InternalError = 4

  };


  enum ComsParameterIndexT
  {
    CPI_FirmwareVersion = 0,
    CPI_PWMState        = 1,
    CPI_PWMMode         = 2,
    CPI_PWMFullReport   = 3,
    CPI_CANBridgeMode   = 4,
    CPI_BoardUID        = 5,
    CPI_TIM1_SR         = 6,
    CPI_VSUPPLY         = 7,
    CPI_DRV8305         = 0x10,
    CPI_DRV8305_01      = 0x10,
    CPI_DRV8305_02      = 0x11,
    CPI_DRV8305_03      = 0x12,
    CPI_DRV8305_04      = 0x13,
    CPI_DRV8305_05      = 0x14,
    CPI_ANGLE_CAL       = 0x20, // 12 Values
    CPI_FINAL           = 0xff
  };

  enum PWMControlModeT {
    CM_Idle     = 0,
    CM_Break    = 1,
    CM_Torque   = 2,
    CM_Velocity = 3,
    CM_Position = 4,
    CM_Final    = 5
  } ;

  struct PacketErrorC {
    uint8_t m_packetType;
    uint8_t m_deviceId;
    uint8_t m_errorCode;
    uint8_t m_errorData;
  } __attribute__((packed));

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
    uint8_t m_mode;
    uint16_t m_position;
    uint16_t m_torque;
  } __attribute__((packed));

  struct PacketServoReportC {
    uint8_t m_packetType; // CPT_ServoAbs / CPT_ServoRel
    uint8_t m_deviceId;
    uint8_t m_mode;
    uint16_t m_timestamp;
    uint16_t m_position;
    int16_t m_torque;
  } __attribute__((packed));

  struct PacketDeviceIdC {
    uint8_t m_packetType; // CPT_AnnounceId or CPT_SetDeviceId
    uint8_t m_deviceId;
    uint32_t m_uid[2];
  } __attribute__((packed)) ;


#ifdef __cplusplus
}
#endif

#endif
