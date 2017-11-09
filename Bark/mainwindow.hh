#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <chrono>

#include "../API/include/dogbot/SerialComs.hh"
#include "../API/include/dogbot/DogBotAPI.hh"
#include "../API/include/dogbot/LegKinematics.hh"

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

  void on_pushButtonSetDeviceId_clicked();

  void on_pushButtonOpenLog_clicked();

  void on_pushButtonQueryState_clicked();

  void on_pushButtonDrv8305_2_clicked();

  void on_pushButtonDrv8305_3_clicked();


  void on_pushButtonDrv8305_5_clicked();

  void on_pushButtonDrv8305_4_clicked();

  void on_pushButtonTim1_clicked();

  void on_spinDeviceId_valueChanged(int arg1);

  void on_pushButton_clicked();

  void on_comboBoxCalibration_activated(const QString &arg1);

  void on_comboBoxControlState_activated(const QString &arg1);

  void on_checkBoxIndicator_toggled(bool checked);

  void on_pushButtonDriveTemp_clicked();

  void on_spinOtherJointId_valueChanged(int arg1);

  void on_comboBox_activated(const QString &arg1);

  void on_comboBoxPositionRef_activated(const QString &arg1);

  void on_pushButton_2_clicked();

  void on_checkBoxBounce_clicked(bool checked);

  void on_verticalSliderBounceOffset_valueChanged(int value);

  void on_verticalSliderBounceRange_valueChanged(int value);

  void on_doubleSpinBoxOmega_valueChanged(double arg1);

  void on_doubleSpinBoxBounceTorque_valueChanged(double arg1);

  void on_pushButtonCalZero_clicked();

  void on_doubleSpinBoxJointRelGain_valueChanged(double arg1);

  void on_doubleSpinBoxJointRelOffset_valueChanged(double arg1);

  void on_doubleSpinBoxDemandPosition_editingFinished();

  void on_doubleSpinBoxTorqueLimit_editingFinished();

  void on_doubleSpinBoxDemandPosition_valueChanged(double arg1);

  void on_doubleSpinBoxCalibrationOffset_editingFinished();

  void on_spinBoxHipJointId_valueChanged(int arg1);

  void on_spinBoxKneeJointId_valueChanged(int arg1);

  void on_checkBoxReverseHip_clicked(bool checked);

  void on_pushButton_3_clicked();

  void on_pushButton_4_clicked();

  void on_doubleSpinBoxIGain_valueChanged(double arg1);

  void updateIGain(double arg1);
  void on_sliderPosition_valueChanged(int value);

  void on_doubleSpinBoxVelocityGain_valueChanged(double arg1);

  void on_doubleSpinBoxVelocityIGain_valueChanged(double arg1);

  void on_doubleSpinBoxVelocityLimit_valueChanged(double arg1);

  void on_doubleSpinBoxPositionGain_valueChanged(double arg1);

  void on_checkBoxFan_toggled(bool checked);

signals:
  void setLogText(const QString &str);
  void setControlState(const QString &str);
  void setControlMode(const QString &str);
  void setFault(const QString &str);
  void setCalibrationState(int index);
  void setCalibrationAngle(double value);
  void setOtherJoint(int jointId);
  void setPositionRef(const QString &str);
  void setIndicator(bool state);
  void setOtherJointGain(double gain);
  void setOtherJointOffset(double offset);
  void setSupplyVoltage(QString str);
  void setDriveTemperature(QString str);
  void setMotorIGain(double offset);
  void setMotorVelocity(double offset);
  void setVelocityPGain(double offset);
  void setVelocityIGain(double offset);
  void setDemandPhaseVelocity(double offset);
  void setVelocityLimit(double offset);
  void setPositionGain(double offset);

private:
  void SetupComs();

  void QueryAll();

  bool ProcessParam(struct PacketParam8ByteC *psp, std::string &displayStr);

  void EnableBounce(bool state);

  Ui::MainWindow *ui;
  std::shared_ptr<DogBotN::SerialComsC> m_coms;
  DogBotN::DogBotAPIC m_dogbotAPI;
  bool m_PWMReportRequested = false;

  std::vector<PacketDeviceIdC> m_devices;
  float m_position = 0;
  float m_torque = 0;
  std::shared_ptr<std::ostream> m_logStrm;
  int m_targetDeviceId = 0;
  enum PositionReferenceT g_positionReference = PR_Relative;

  bool m_bounceRunning = false;
  std::chrono::time_point<std::chrono::steady_clock> m_startBounce;
  float m_omega = M_PI ;
  float m_bounceOffset = 0.4;
  float m_bounceRange = 0.3;
  float m_bounceTorque = 2.0;
  int m_hipJointId = 2;
  int m_kneeJointId = 1;
  bool m_reverseHip = false;
  DogBotN::LegKinematicsC m_legKinematics;

  float m_servoAngle = 0;
  float m_servoTorque = 0;
  enum PositionReferenceT m_servoRef = PR_Relative;
  enum PWMControlModeT m_controlMode = CM_Idle;
};

#endif // MAINWINDOW_H
