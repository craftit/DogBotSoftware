#ifndef DOGBOG_SERVO_HEADER
#define DOGBOG_SERVO_HEADER 1

#include "dogbot/Joint.hh"
#include "dogbot/Device.hh"
#include <chrono>
#include "dogbot/Coms.hh"

namespace DogBotN {

  class DogBotAPIC;
  enum HomeDirectionHintT { HDH_None, HDH_Anticlockwise, HDH_Clockwise };

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
   : public JointC,
     public DeviceC
  {
  public:
    typedef std::function<void (TimePointT theTime,double position,double velocity,double torque,PositionReferenceT posRef)> PositionRefUpdateFuncT;

    // Construct from coms link and deviceId
    ServoC(const std::shared_ptr<ComsC> &coms,int deviceId);

    // Construct with announce packet.
    ServoC(const std::shared_ptr<ComsC> &coms,int deviceId,const PacketDeviceIdC &pktAnnounce);

    //! Type of joint
    virtual std::string JointType() const override;

    //! Access the device type
    virtual const char *DeviceType() const override;

    //! Set the name of the joint/device
    virtual void SetName(const std::string &name) override;

    //! Configure from JSON
    virtual bool ConfigureFromJSON(DogBotAPIC &api,const Json::Value &value) override;

    //! Get the servo configuration as JSON
    virtual void ConfigAsJSON(Json::Value &value) const override;

    //! Get last reported state of the servo and the time it was taken.
    //! Position in radians.
    //! Velocity in radians/second
    //! torque in Newton-meters
    bool GetState(TimePointT &tick,double &position,double &velocity,double &torque) const override;

    //! Get raw state information
    bool GetRawState(TimePointT &tick,double &position,double &velocity,double &torque,enum PositionReferenceT &posRef);

    //! Estimate state at the given time.
    //! Position in radians.
    //! Velocity in radians/second
    //! torque in Newton-meters
    //! This will linearly extrapolate position, and assume velocity and torque are
    //! the same as the last reading.
    //! If the data is more than 5 ticks away from the
    bool GetStateAt(TimePointT theTime,double &position,double &velocity,double &torque) const override;

    //! Update torque for the servo.
    // torque is in Newton-meters.
    bool DemandTorque(float torque) override;

    //! Demand a position for the servo
    //! position in radians
    //! torqueLimit is in Newton-meters
    bool DemandPosition(float position,float torqueLimit) override;

    //! Set the trajectory.
    // period in seconds,
    // torqueLimit is in Newton-meters
    bool SetupTrajectory(float period,float torqueLimit) override;

    //! Demand next position for the servo
    //! position in radians
    //! Expected torque in Newton-meters
    bool DemandTrajectory(float position,float torque = 0) override;

    //! Access the type of last position received.
    enum PositionReferenceT PositionReference() const
    { return m_positionRef; }

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
    void QueryRefresh() override;

    //! Access index state
    int IndexState() const
    { return m_reportedMode; }

    //! Update coms device
    virtual void UpdateComs(const std::shared_ptr<ComsC> &coms) override;

    //! Home a point position. This will block until homing is complete.
    //! Returns true if homing succeeded.
    bool HomeJoint(bool restorePosition,HomeDirectionHintT directionHint = HDH_None);

    //! Add a update callback for motor position
    CallbackHandleC AddPositionRefUpdateCallback(const PositionRefUpdateFuncT &callback)
    { return m_positionRefCallbacks.Add(callback); }

    //! Test if joint home index sensor is active
    bool IsAtHomeIndex() const
    { return !m_homeIndexState; }

    //! Restore configuration from stored settings.
    virtual bool RestoreConfig() override;

    //! Access supply voltage scaling.
    float SupplyVoltageScale() const
    { return m_supplyVoltageScale; }

    //! Test if we've had communication with the device.
    // This checks if we've ever received a firmware version
    bool IsPresent() const
    { return m_firmwareVersion >= 0; }

    //! Check if the motor firmware version matches API.
    bool IsFirmwareVersionOk() const;

    //! Access update tick duration
    double TickDuration() const override
    { return m_tickDuration.count();  }

  protected:
    //! Convert a report value to a torque
    float TorqueReport2Current(int16_t val)
    { return ((float) val * m_maxCurrent)/ 32767.0; }

    //! Move joint until we see an index state change.
    //! This always works in relative coordinates.
    JointMoveStatusT MoveUntilIndexChange(
        float targetPosition,
        float torqueLimit,
        bool currentIndexState,
        float &changedAt,
        bool &indexChanged,
        double timeOut = 3.0
        );

    //! Demand a position for the servo, torque limit is in Newton-meters
    bool DemandPosition(float position,float torqueLimit,enum PositionReferenceT positionRef);

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

    //! Handle parameter update.
    //! Returns true if a value has changed.
    bool HandlePacketReportParam(const PacketParam8ByteC &pkt,int size) override;

    //! Tick from main loop
    //! Used to check for communication timeouts.
    //! Returns true if state changed.
    virtual bool UpdateTick(TimePointT timeNow) override;


    mutable std::mutex m_mutexAdmin;

    CallbackArrayC<PositionRefUpdateFuncT> m_positionRefCallbacks;

    std::shared_ptr<MotorCalibrationC> m_motorCal;

    int m_firmwareVersion = -1;
    int m_queryCycle = 0;

    FaultCodeT m_faultCode = FC_Unknown;
    MotionHomedStateT m_homedState = MHS_Lost;
    PWMControlDynamicT m_controlDynamic = CM_Off;
    bool m_homeIndexState = false;

    TimePointT m_timeEpoch;
    TimePointT m_timeOfLastReport;

    uint8_t m_lastTimestamp = 0;
    uint8_t m_trajectoryTimestamp = 0;

    TimePointT m_lastVersionQuery;
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
    float m_servoReportFrequency = 100.0;
    float m_motorKv = 260;    //! < Motor speed constant
    float m_gearRatio = 21.0; //!< Gearbox ratio
    float m_servoKt = 0;      //!< Servo torque constant
    float m_maxCurrent = 20.0;
    float m_velocityLimit = 0;
    float m_currentLimit = 0;
    float m_positionPGain = 0;
    float m_velocityPGain = 0;
    float m_velocityIGain = 0;
    float m_motorInductance = 0;
    float m_motorResistance = 0;

    float m_homeOffset = 0;
    float m_defaultHomeOffset = 0; // Value as loaded from configuration.
    float m_endStopStart = 0;
    float m_endStopFinal = 0;
    float m_supplyVoltageScale = 1.0;
    float m_defaultSupplyVoltageScale = 1.0; // Value as loaded from configuration.
    bool m_endStopEnable = false;
    enum SafetyModeT m_safetyMode = SM_GlobalEmergencyStop;
    enum SafetyModeT m_defaultSafetyMode =  SM_GlobalEmergencyStop;
    friend class DogBotAPIC;
  };

}

#endif
