#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <chrono>

#include "dogbot/Coms.hh"
#include "dogbot/DogBotAPI.hh"
#include "dogbot/LegKinematics.hh"
#include "dogbot/SplineGaitController.hh"
#include "dogbot/LegController.hh"
#include "dogbot/DevicePlatformManager.hh"

#include "ServoTable.hh"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

protected:
  void timerEvent(QTimerEvent *event) override;

private slots:
  void LocalProcessParam(PacketParam8ByteC psp);

  void PlatformProcessParam(PacketParam8ByteC psp);

  void on_pushButtonConnect_clicked();

  void on_pushButtonPWMReport_clicked();

  void on_pushButtonPing_clicked();

  void on_comboBoxMotorControlMode_activated(const QString &arg1);

  void on_sliderPosition_sliderMoved(int position);

  void on_sliderTorque_sliderMoved(int torque);

  void on_pushButtonQueryId_clicked();

  void on_pushButtonGetVersion_clicked();

  void on_pushButtonState_clicked();

  void on_pushButtonQueryDevices_clicked();

  void on_pushButtonPing1_clicked();

  void on_pushButtonOpenLog_clicked();

  void on_pushButtonDrv8305_2_clicked();

  void on_pushButtonDrv8305_3_clicked();

  void on_pushButtonDrv8305_5_clicked();

  void on_pushButtonDrv8305_4_clicked();

  void on_pushButtonTim1_clicked();

  void on_spinDeviceId_valueChanged(int arg1);

  void on_comboBoxCalibration_activated(const QString &arg1);

  void on_comboBoxControlState_activated(const QString &arg1);

  void on_checkBoxIndicator_toggled(bool checked);

  void on_comboBox_activated(const QString &arg1);

  void on_comboBoxPositionRef_activated(const QString &arg1);

  void on_pushButton_2_clicked();

  void on_pushButtonCalZero_clicked();

  void on_doubleSpinBoxDemandPosition_editingFinished();

  void on_doubleSpinBoxTorqueLimit_editingFinished();

  void on_doubleSpinBoxDemandPosition_valueChanged(double arg1);

  void on_doubleSpinBoxCalibrationOffset_editingFinished();

  void on_pushButton_3_clicked();

  void on_pushButton_4_clicked();

  void on_doubleSpinBoxIGain_valueChanged(double arg1);

  void updateIGain(double arg1);
  void on_sliderPosition_valueChanged(int value);

  void on_doubleSpinBoxVelocityGain_valueChanged(double arg1);

  void on_doubleSpinBoxVelocityIGain_valueChanged(double arg1);

  void on_doubleSpinBoxVelocityLimit_valueChanged(double arg1);

  void on_doubleSpinBoxPositionGain_valueChanged(double arg1);

  void on_pushButton_5_clicked();

  void on_pushButtonEmergencyStop_clicked();

  void on_doubleSpinBoxHomeIndexOffset_valueChanged(double arg1);

  void on_pushButtonQueryHomed_clicked();

  void on_pushButtonOff_clicked();

  void on_pushButtonHold_clicked();

  void on_comboBoxServoName_activated(const QString &arg1);

  void on_actionSaveConfig_triggered();

  void on_actionLoadConfig_triggered();

  void on_actionSave_Config_As_triggered();

  void on_actionExit_triggered();

  void on_comboBoxFanState_activated(int index);

  void on_lineEditFanTempThreshold_editingFinished();

  void on_comboBoxFanState_currentIndexChanged(const QString &arg1);

  void on_updatePosition(double angle);

  void on_doubleSpinBoxEndStopStart_valueChanged(double arg1);

  void on_doubleSpinBoxEndStopEnd_valueChanged(double arg1);

  void on_checkBoxEndStopEnable_toggled(bool checked);

  void on_pushButtonSetEndStopStart_clicked();

  void on_pushButtonSetEndStopEnd_clicked();

  void on_pushButtonSaveConfig_clicked();

  void on_comboBoxSafetyMode_activated(const QString &arg1);

  void on_pushButtonLowPower_clicked();

  void on_comboBoxGotoJoint_activated(const QString &arg1);

  void on_pushButtonHomeJoint_clicked();

  void on_pushButtonHomeAll_clicked();

  void on_pushButtonPhaseEndStops_clicked();

  void on_doubleSpinBoxEndStopForce_valueChanged(double arg1);

  void on_lineEditServoReportFrequency_textEdited(const QString &arg1);

  void on_checkBoxRunAnimation_stateChanged(int arg1);

  void on_dialAnimationOmega_valueChanged(int value);

  void on_doubleSpinBoxAnimationTorque_valueChanged(double arg1);

  void on_doubleSpinBoxAnimationSpeedLimit_valueChanged(double arg1);

  void on_comboBoxAmimationStyle_activated(const QString &arg1);

  void on_horizontalSliderHeight_valueChanged(int value);

  void on_checkBoxStand_stateChanged(int arg1);

  void on_pushButtonUpdateSupplyCal_clicked();

  void on_pushButtonRestoreConfig_clicked();

  void on_pushButtonResetSupplyVoltageCal_clicked();

  void on_doubleSpinBoxSupplyVoltageScale_valueChanged(double arg1);

  void on_doubleSpinBoxMotionUpdate_valueChanged(double arg1);

  void on_doubleSpinBoxCurrentLimit_valueChanged(double arg1);

  void on_pushButtonActivityHome_clicked();

  void on_pushButtonActivityWalk_clicked();

  void on_pushButtonActivityPassive_clicked();

  void on_pushButtonActivityIdle_clicked();

  void on_pushButtonActivityAbort_clicked();

  void on_pushButtonPowerOnAll_clicked();

  void on_checkBoxDiagnosticMode_stateChanged(int arg1);

  void on_lineEditFanTempThreshold_textChanged(const QString &arg1);

  void on_lineEditMinimumSupply_editingFinished();

  void on_pushButtonRestoreDefaults_clicked();

  void on_checkBoxEnableAngleStats_stateChanged(int arg1);

  void on_pushButtonDumpAngleStats_clicked();

  void on_lineEditDebugValue_editingFinished();

  void on_lineEditMaxCurrent_editingFinished();

  void on_pushButtonRestoreFactorySetup_clicked();

  void on_pushButtonSetMinSupplyVoltage_clicked();

  void on_comboBoxDeviceType_activated(const QString &arg1);

  void on_comboBox_HSSourceCurrent_activated(const QString &arg1);

  void on_comboBox_HSSinkCurrent_activated(const QString &arg1);

  void on_comboBox_LSSourceCurrent_activated(const QString &arg1);

  void on_comboBox_LSSinkCurrent_activated(const QString &arg1);

  void on_pushButtonDrv8305_6_clicked();

  void on_pushButton_Drv8305_A_clicked();

  void on_comboBox_DeadTime_activated(int index);

  void on_pushButtonDrv8305_7_clicked();

  void on_comboBox_HSDriveTime_activated(int index);

  void on_comboBox_LSDriveTime_activated(int index);

  void on_pushButtonDrv8305_0_clicked();

  void on_pushButtonDrv8305_1_clicked();

signals:
  void setLogText(const QString &str);
  void setControlState(const QString &str);
  void setControlMode(const QString &str);
  void setFault(const QString &str);
  void setCalibrationState(int index);
  void setCalibrationAngle(double value);
  void setPositionRef(const QString &str);
  void setIndicator(bool state);
  void setSupplyVoltage(QString str);
  void setDriveTemperature(QString str);
  void setMotorTemperature(QString str);
  void setFanTemperatureThreshold(QString str);
  void setMotorIGain(double offset);
  void setMotorVelocity(double offset);
  void setVelocityPGain(double offset);
  void setVelocityIGain(double offset);
  void setDemandPhaseVelocity(double offset);
  void setVelocityLimit(double offset);
  void setPositionGain(double offset);
  void setHomeIndexOffset(double offset);
  void setHallSensors(const QString &str);
  void setUSBErrors(const QString &str);
  void setUSBDrops(const QString &str);
  void setCANErrors(const QString &str);
  void setCANDrops(const QString &str);
  void setMainLoopTimeout(const QString &str);
  void setFaultMap(const QString &str);
  void setIndexSensor(bool state);
  void setJointRelativeEnabled(bool state);
  void setFanMode(const QString &str);
  void updatePosition(double angle);

  void callLocalProcessParam(struct PacketParam8ByteC psp);

private:
  void SetupComs();

  void QueryAll();

  bool ProcessParam(struct PacketParam8ByteC *psp, std::string &displayStr);

  //! Close current connection
  void CloseConnection();

  //! Get the current position and set the given parameter to it.
  void SetValueToCurrentPosition(ComsParameterIndexT param);

  //! Run the current animation
  void RunAnimation();

  //! Stop current animation
  void StopAnimation();

  bool SetupPlatformManager();

  void SendHSCurrentSetup();
  void SendLSCurrentSetup();

  int m_toQuery = 0;
  std::vector<ComsParameterIndexT> m_displayQuery;

  bool m_updatePositionFromController = false;

  Ui::MainWindow *ui;
  std::shared_ptr<DogBotN::ComsC> m_coms;
  std::shared_ptr<DogBotN::DogBotAPIC> m_dogBotAPI;
  bool m_PWMReportRequested = false;

  std::vector<PacketDeviceIdC> m_devices;
  float m_position = 0;
  float m_torque = 3.0;
  std::shared_ptr<std::ostream> m_logStrm;
  int m_targetDeviceId = 0;
  enum ControlStateT m_targetControlState = CS_StartUp;
  enum PositionReferenceT g_positionReference = PR_Relative;

  float m_servoAngle = 0;
  float m_servoTorque = 0;
  float m_maxCurrent = 20.0;
  enum PositionReferenceT m_servoRef = PR_Relative;
  enum PWMControlDynamicT m_controlMode = CM_Off;
  ServoTable *m_servoTable = 0;
  std::string m_configFilename;

  DogBotN::SplineGaitControllerC m_gaitController;
  bool m_runAnimation = false;
  std::thread m_animationThread;
  float m_animationOmega = 1.0;
  float m_animationTorque = 6.0;
  float g_animationVelocityLimit = 500.0;
  float m_minHeight = 0.3;
  float m_maxHeight = 0.6;
  bool m_runStand = false;
  std::mutex m_accessGait;
  float m_lastSpeedLimit = 0.0;
  std::shared_ptr<DogBotN::LegControllerC> m_legs[4];
  std::shared_ptr<DogBotN::DevicePlatformManagerC> m_platformManager;
};

#endif // MAINWINDOW_H
