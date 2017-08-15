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
    CPT_ServoAbs     =  7, // Servo control absolute position
    CPT_ServoRel     =  8, // Servo control relative position
    CPT_CAN          =  9, //  CAN packet
    CPT_PWMState     = 10, // PWM State.
    CPT_QueryDevices = 11, // Query connected devices
    CPT_AnnounceId   = 12, // Query connected devices
    CPT_SetDeviceId  = 13  // Set device id
  };


  enum ComsErrorTypeT {
    CET_UnknownPacketType = 0,
    CET_UnexpectedPacketSize = 1,
    CET_ParameterOutOfRange = 2,
    CET_CANTransmitFailed = 3

  };


  enum ComsParameterIndexT
  {
    CPI_FirmwareVersion = 0,
    CPI_PWMState        = 1,
    CPI_PWMMode         = 2,
    CPI_PWMFullReport   = 3,
    CPI_CANBridgeMode   = 4,
    CPI_BoardUID        = 5,
  };

  enum PWMControlModeT {
    CM_Idle,
    CM_Break,
    CM_Torque,
    CM_Velocity,
    CM_Position,
    CM_Final
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

  struct PacketParamC {
    uint8_t m_packetType; //  CPT_SetParam or CPT_ReportParam
    uint8_t m_deviceId;   // Target device
    uint16_t m_index;
    uint16_t m_data;
  } __attribute__((packed));

  struct PacketParam2Int32C {
    uint8_t m_packetType;
    uint8_t m_deviceId;   // Target device
    uint16_t m_index;
    uint32_t m_data[2];
  } __attribute__((packed));


  struct PacketPWMStateC {
    uint8_t m_packetType;
    uint16_t m_tick;
    uint16_t m_hall[3];
    uint16_t m_curr[3];
    uint16_t m_angle;
  } __attribute__((packed));

  struct PacketServoC {
    uint8_t m_packetType;
    uint8_t m_deviceId;
    uint8_t m_mode;
    uint16_t m_timestamp;
    uint16_t m_position;
    uint16_t m_torque;
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
