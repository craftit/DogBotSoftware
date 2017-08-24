#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <QFileDialog>
#include <fstream>

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  ui->lineEditDevice->setText("/dev/ttyACM1");
  SetupComs();
  connect(this,SIGNAL(setLogText(const QString &)),ui->textEditLog,SLOT(setText(const QString &)));
}

void MainWindow::SetupComs()
{
  m_coms.SetHandler(CPT_PWMState,[this](uint8_t *data,int size) mutable
  {
    char buff[1024];
    PacketPWMStateC *msg = (PacketPWMStateC *) data;
    sprintf(buff,"%5d,%4d,%4d,%4d,%4d,%4d,%4d,%6d   \n",
           msg->m_tick,
           msg->m_hall[0],msg->m_hall[1],msg->m_hall[2],
           msg->m_curr[0],msg->m_curr[1],msg->m_curr[2],
           msg->m_angle);

    if(m_logStrm) {
      *m_logStrm << buff;
    } else {
      std::cout << buff;
    }
    //emit setLogText(buff);
  });

  m_coms.SetHandler(CPT_Pong,[this](uint8_t *data,int size) mutable
  {
    std::cout << "Got pong. " << std::endl;
    emit setLogText("Got pong");
  });

  m_coms.SetHandler(CPT_ReportParam,[this](uint8_t *data,int size) mutable
  {
    printf("Got SetParam.  Size:%d \n",size);
    std::string displayStr;
    struct PacketParam8ByteC *psp = (struct PacketParam8ByteC *) data;
    displayStr += "Index:";
    displayStr += std::to_string((int) psp->m_header.m_index);
    displayStr += " Device:";
    displayStr += std::to_string((int) psp->m_header.m_deviceId);
    displayStr += " Data:";
    char buff[64];
    for(unsigned i = 0;i < (size - sizeof(psp->m_header));i++) {
      sprintf(buff,"%02x ",(unsigned) psp->m_data.uint8[i]);
      displayStr += buff;
    }

    emit setLogText(displayStr.c_str());
  });

  m_coms.SetHandler(CPT_AnnounceId,[this](uint8_t *data,int size) mutable
  {
    if(size != sizeof(struct PacketDeviceIdC)) {
      emit setLogText("Unexpected packet length.");
      return;
    }
    const PacketDeviceIdC *pkt = (const PacketDeviceIdC *) data;

    int atIndex = -1;
    for(unsigned i = 0;i < m_devices.size();i++) {
      if(m_devices[i].m_uid[0] == pkt->m_uid[0] &&
         m_devices[i].m_uid[1] == pkt->m_uid[1]) {
        atIndex = i;
        m_devices[i].m_deviceId = pkt->m_deviceId;
        break;
      }
    }
    if(atIndex < 0) {
      m_devices.push_back(*pkt);
    }
    std::string displayStr;
    for(unsigned i = 0;i < m_devices.size();i++) {
      displayStr += std::to_string(m_devices[i].m_uid[0]) + " " + std::to_string(m_devices[i].m_uid[1]) + " -> " + std::to_string(m_devices[i].m_deviceId) + "\n";
    }
    emit setLogText(displayStr.c_str());
  });

}


MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::on_pushButtonConnect_clicked()
{
  if(m_coms.Open(ui->lineEditDevice->text().toStdString().c_str())) {
    ui->labelConnectionState->setText("Ok");
    emit setLogText("Connect ok");
  } else {
    ui->labelConnectionState->setText("Failed");
    emit setLogText("Connect failed");
  }
}

void MainWindow::on_pushButtonPWM_clicked()
{
  m_coms.SendSetParam(0,CPI_PWMState,1);
  emit setLogText("PWM clicked");

}

void MainWindow::on_pushButtonPWMReport_clicked()
{
  if(!m_PWMReportRequested) {
    ui->pushButtonPWMReport->setText("Stop PWM Report");
    m_PWMReportRequested = true;
    m_coms.SendSetParam(0,CPI_PWMFullReport,1);
  } else {
    ui->pushButtonPWMReport->setText("Start PWM Report");
    m_PWMReportRequested = false;
    m_coms.SendSetParam(0,CPI_PWMFullReport,0);
  }
}

void MainWindow::on_pushButtonPing_clicked()
{
  emit setLogText("Ping");
  m_coms.SendPing(0);
}

void MainWindow::on_comboBoxMotorControlMode_activated(const QString &arg1)
{
  enum PWMControlModeT controlMode = CM_Final;
  if(arg1 == "Idle") {
    controlMode = CM_Idle;
  }
  if(arg1 == "Break") {
    controlMode = CM_Break;
  }
  if(arg1 == "Torque") {
    controlMode = CM_Torque;
  }
  if(arg1 == "Velocity") {
    controlMode = CM_Velocity;
  }
  if(arg1 == "Position") {
    controlMode = CM_Position;
  }
  if(controlMode == CM_Final) {
    printf("Unhandled control mode %s ",arg1.toStdString().c_str());
    return ;
  }
  m_coms.SendSetParam(0,CPI_PWMMode,controlMode);
}

void MainWindow::on_sliderPosition_sliderMoved(int position)
{
  m_position = position * 3.14159265359/ 360.0;
  m_coms.SendMoveWithEffort(0,m_position,m_torque);
}

void MainWindow::on_sliderTorque_sliderMoved(int torque)
{
  m_torque = torque / 10.0;
  m_coms.SendMoveWithEffort(0,m_position,m_torque);
}

void MainWindow::on_pushButtonStopPWM_clicked()
{
  m_coms.SendSetParam(0,CPI_PWMState,0);
  emit setLogText("PWM clicked");
}

void MainWindow::on_pushButtonQueryId_clicked()
{
  m_coms.SendQueryParam(0,CPI_BoardUID);
}

void MainWindow::on_pushButtonGetVersion_clicked()
{
  m_coms.SendQueryParam(0,CPI_FirmwareVersion);
}

void MainWindow::on_pushButtonState_clicked()
{
  m_coms.SendQueryParam(0,CPI_PWMState);
}

void MainWindow::on_pushButtonSetBridgeMode_clicked()
{
  m_coms.SendSetParam(0,CPI_CANBridgeMode,1);
  m_coms.SendQueryParam(0,CPI_CANBridgeMode);
}

void MainWindow::on_pushButtonQueryDevices_clicked()
{
  m_coms.SendQueryDevices();
}

void MainWindow::on_pushButtonPing1_clicked()
{
  emit setLogText("Ping");
  m_coms.SendPing(1);
}

void MainWindow::on_pushButtonSetDeviceId_clicked()
{
  for(int i = 0;i < m_devices.size();i++) {
    m_coms.SendSetDeviceId(i+1,m_devices[i].m_uid[0],m_devices[i].m_uid[1]);
  }
}

void MainWindow::on_pushButtonOpenLog_clicked()
{
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save Log File"),
                             "motor.csv",
                             tr("Log (*.csv);;All Files (*)"));
  m_logStrm = std::shared_ptr<std::ostream>(new std::ofstream(fileName.toLocal8Bit().data()));
}

void MainWindow::on_pushButtonQueryState_clicked()
{
  m_coms.SendQueryParam(0,CPI_DRV8305_01);
}

void MainWindow::on_pushButtonDrv8305_2_clicked()
{
  m_coms.SendQueryParam(0,CPI_DRV8305_02);
}

void MainWindow::on_pushButtonDrv8305_3_clicked()
{
  m_coms.SendQueryParam(0,CPI_DRV8305_03);
}

void MainWindow::on_pushButtonDrv8305_4_clicked()
{
  m_coms.SendQueryParam(0,CPI_DRV8305_04);
}

void MainWindow::on_pushButtonDrv8305_5_clicked()
{
  m_coms.SendQueryParam(0,CPI_DRV8305_05);
}


void MainWindow::on_pushButtonTim1_clicked()
{
  m_coms.SendQueryParam(0,CPI_TIM1_SR);
}
