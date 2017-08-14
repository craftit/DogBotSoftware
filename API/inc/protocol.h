#ifndef DOGBOG_PROTOCOL_HEADER
#define DOGBOG_PROTOCOL_HEADER 1

#ifdef __cplusplus
extern "C" {
#endif

  enum ComsPacketTypeT
  {
    CPT_Ping  = 0,  // Ping request
    CPT_Pong  = 1,  // Ping reply.
    CPT_Error = 2,  // Error report
    CPT_Sync  = 3,  // Sync data stream
    CPT_ReadParam = 4, // Read parameter
    CPT_SetParam  = 5, // Set parameter
    CPT_Servo     = 6, // Servo control
    CPT_CAN       = 7, //  CAN packet
    CPT_PWMState  = 8, // PWM State.
  };


  enum ComsErrorTypeT {
    CET_UnknownPacketType = 0,
    CET_UnexpectedPacketSize = 1,
    CET_ParameterOutOfRange = 2
  };


  enum ComsParameterIndexT
  {
    CPI_FirmwareVersion = 0,
    CPI_PWMState        = 1,
    CPI_PWMMode         = 2,
    CPI_PWMFullReport   = 3,
    CPI_CANBridgeMode   = 4,
  };

  enum PWMControlModeT {
    CM_Idle,
    CM_Break,
    CM_Torque,
    CM_Velocity,
    CM_Position,
    CM_Final
  } ;

  struct PacketReadParamC {
    uint8_t m_packetType;
    uint16_t m_index;
  };

  struct PacketSetParamC {
    uint8_t m_packetType;
    uint16_t m_index;
    uint16_t m_data;
  };

  struct PacketPWMStateC {
    uint8_t m_packetType;
    uint16_t m_tick;
    uint16_t m_hall[3];
    uint16_t m_curr[3];
    uint16_t m_angle;
  };

  struct PacketServoC {
    uint8_t m_packetType;
    uint16_t m_position;
    uint16_t m_torqueLimit;
  };


#ifdef __cplusplus
}
#endif

#endif
