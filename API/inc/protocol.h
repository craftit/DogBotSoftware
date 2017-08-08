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
  };

#ifdef __cplusplus
}
#endif

#endif
