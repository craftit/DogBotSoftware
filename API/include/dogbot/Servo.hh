#ifndef DOGBOG_SERVO_HEADER
#define DOGBOG_SERVO_HEADER 1

#include "dogbot/Joint.hh"
#include <chrono>
#include "dogbot/Coms.hh"

namespace DogBotN {

  class DogBotAPIC;

  //! Motor calibration data.

  class MotorCalibrationC
  {
  public:
    MotorCalibrationC();

    //! Load configuration from JSON
    bool LoadJSON(const Json::Value &conf);

    //! Save calibration as JSON
    Json::Value AsJSON() const;

    //! Set calibration point
    void SetCal(int place,uint16_t p1,uint16_t p2,uint16_t p3);

    //! Get calibration point
    void GetCal(int place,uint16_t &p1,uint16_t &p2,uint16_t &p3) const;

    //! Send calibration to motor
    bool SendCal(ComsC &coms,int deviceId);

    //! Read calibration from motor
    bool ReadCal(ComsC &coms,int deviceId);

    //! Access motor Kv.
    float MotorKv() const
    { return m_motorKv; }

    //! Access velocity limit
    float VelocityLimit() const
    { return m_velocityLimit; }

    float PositionPGain() const
    { return m_positionPGain; }

    float VelocityPGain() const
    { return m_velocityPGain; }

    float VelocityIGain() const
    { return m_velocityIGain; }

    float CurrentLimit() const
    { return m_currentLimit; }

    float MotorInductance() const
    { return m_motorInductance; }

    float MotorResistance() const
    { return m_motorResistance; }

  protected:
    //! Get json value if set.
    bool GetJSONValue(const Json::Value &conf,const char *name,float &value);

    static const int m_hallCalPoints = 18;
    //std::vector<>
    uint16_t m_hall[m_hallCalPoints][3];

    // Servo parameters
    float m_motorKv = 260;
    float m_velocityLimit = 0;
    float m_currentLimit = 0;
    float m_positionPGain = 0;
    float m_velocityPGain = 0;
    float m_velocityIGain = 0;
    float m_motorInductance = 0;
    float m_motorResistance = 0;
  };

  //! Information about a single servo.

  class ServoC
   : public JointC
  {
  public:

    // Construct from coms link and deviceId
    ServoC(const std::shared_ptr<ComsC> &coms,int deviceId);

    // Construct with announce packet.
    ServoC(const std::shared_ptr<ComsC> &coms,int deviceId,const PacketDeviceIdC &pktAnnounce);

    //! Type of joint
    virtual std::string JointType() const override;

    //! Set servo enabled flag.
    void SetEnabled(bool enabled)
    { m_enabled = enabled; }

    //! Is servo enabled ?
    bool IsEnabled() const
    { return m_enabled; }

    //! Access the device id.
    int Id() const
    { return m_id; }

    //! Set device id.
    void SetId(int id)
    { m_id = id; }

    //! Access part 1 of unique id
    int UId1() const
    { return m_uid1; }

    //! Access part 2 of unique id
    int UId2() const
    { return m_uid2; }

    //! Set device unique id
    void SetUID(uint32_t uid1,uint32_t uid2);

    //! Does this servo have a matching id ?
    bool HasUID(uint32_t uid1,uint32_t uid2) const
    { return m_uid1 == uid1 && m_uid2 == uid2; }

    //! Configure from JSON
    bool ConfigureFromJSON(DogBotAPIC &api,const Json::Value &value) override;

    //! Get the servo configuration as JSON
    Json::Value ConfigAsJSON() const override;

    //! Get last reported state of the servo and the time it was taken.
    //! Position in radians.
    //! Velocity in radians/second
    //! torque in N.m
    bool GetState(TimePointT &tick,double &position,double &velocity,double &torque) const override;

    //! Estimate state at the given time.
    //! Position in radians.
    //! Velocity in radians/second
    //! torque in N.m
    //! This will linearly extrapolate position, and assume velocity and torque are
    //! the same as the last reading.
    //! If the data is more than 5 ticks away from the
    bool GetStateAt(TimePointT theTime,double &position,double &velocity,double &torque) const override;

    //! Update torque for the servo. In Newton-meters.
    bool DemandTorque(float torque) override;

    //! Demand a position for the servo, torque limit is in Newton-meters
    bool DemandPosition(float position,float torqueLimit) override;

    //! Last fault code received
    FaultCodeT FaultCode() const
    { return m_faultCode; }

    //! Get the current calibration state.
    MotionHomedStateT HomedState() const
    { return m_homedState; }

    //! Access the control state.
    ControlStateT ControlState() const
    { return m_controlState; }

    //! Last reported temperature
    float DriveTemperature() const
    { return m_driveTemperature; }

    //! Last reported temperature
    float MotorTemperature() const
    { return m_motorTemperature; }

    //! Access supply voltage
    float SupplyVoltage() const
    { return m_supplyVoltage; }

    //! Access default torque to use.
    float DefaultPositionTorque() const
    { return m_defaultPositionTorque; }

    //! Access control dynamic
    PWMControlDynamicT ControlDynamic() const
    { return m_controlDynamic; }

    //! Query setup information from the controller again.
    void QueryRefresh();

    //! Access index state
    int IndexState() const
    { return m_reportedMode; }

    //! Update coms device
    virtual void UpdateComs(const std::shared_ptr<ComsC> &coms) override;

  protected:
    //! Initialise timeouts and setup
    void Init();

    //! Do constant setup
    void SetupConstants();

    //! Process update
    //! Returns true if state changed
    bool HandlePacketPong(const PacketPingPongC &);

    //! Process update
    //! Returns true if state changed
    bool HandlePacketServoReport(const PacketServoReportC &);

    //! Handle an incoming announce message.
    //! Returns true if state changed.
    bool HandlePacketAnnounce(const PacketDeviceIdC &pkt,bool isManager);

    //! Handle parameter update.
    bool HandlePacketReportParam(const PacketParam8ByteC &pkt);

    //! Tick from main loop
    //! Used to check for communication timeouts.
    //! Returns true if state changed.
    bool UpdateTick(TimePointT timeNow);


    mutable std::mutex m_mutexAdmin;

    uint32_t m_uid1 = 0;
    uint32_t m_uid2 = 0;
    int m_id = -1; // Device id.

    bool m_enabled = true;

    std::shared_ptr<MotorCalibrationC> m_motorCal;

    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");
    std::shared_ptr<ComsC> m_coms;

    bool m_online = false;

    int m_queryCycle = 0;

    FaultCodeT m_faultCode = FC_Unknown;
    MotionHomedStateT m_homedState = MHS_Lost;
    ControlStateT m_controlState = CS_StartUp;
    PWMControlDynamicT m_controlDynamic = CM_Off;
    mutable std::mutex m_mutexState;

    TimePointT m_timeEpoch;
    TimePointT m_timeOfLastReport;
    TimePointT m_timeOfLastComs;

    uint8_t m_lastTimestamp = 0;

    std::chrono::duration<double> m_tickDuration; // Default is 10ms
    std::chrono::duration<double> m_comsTimeout; // Default is 200ms
    unsigned m_tick = 0;

    float m_defaultPositionTorque = 4.0;
    float m_supplyVoltage = 0;
    enum PositionReferenceT m_positionRef = PR_Relative;
    float m_driveTemperature = 0;
    float m_motorTemperature = 0;

    int m_toQuery = 0;
    int m_bootloaderQueryCount;
    std::vector<ComsParameterIndexT> m_updateQuery;

    unsigned m_reportedMode = 0;

    // Current parameters
    float m_motorKv = 260; //! < Motor speed constant
    float m_gearRatio = 21.0; //!< Gearbox ratio
    float m_servoKt = 0;   //! < Servo torque constant
    float m_velocityLimit = 0;
    float m_currentLimit = 0;
    float m_positionPGain = 0;
    float m_velocityPGain = 0;
    float m_velocityIGain = 0;
    float m_motorInductance = 0;
    float m_motorResistance = 0;

    friend class DogBotAPIC;
  };

}

#endif
