#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "../API/include/dogbot/SerialComs.hh"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

private slots:
  void on_pushButtonConnect_clicked();

  void on_pushButtonPWM_clicked();

  void on_pushButtonPWMReport_clicked();

  void on_pushButtonPing_clicked();

  void on_comboBoxMotorControlMode_activated(const QString &arg1);

  void on_sliderPosition_sliderMoved(int position);

  void on_sliderTorque_sliderMoved(int position);

  void on_pushButtonStopPWM_clicked();

  void on_pushButtonQueryId_clicked();

  void on_pushButtonGetVersion_clicked();

  void on_pushButtonState_clicked();

  void on_pushButtonSetBridgeMode_clicked();

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

signals:
  void setLogText(const QString &str);

private:
  void SetupComs();

  Ui::MainWindow *ui;
  DogBotN::SerialComsC m_coms;
  bool m_PWMReportRequested = false;

  std::vector<PacketDeviceIdC> m_devices;
  float m_position = 0;
  float m_torque = 0;
  std::shared_ptr<std::ostream> m_logStrm;
  int m_targetDeviceId = 0;
};

#endif // MAINWINDOW_H
