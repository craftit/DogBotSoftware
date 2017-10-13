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
  connect(this,SIGNAL(setCalibrationState(int)),ui->comboBoxCalibration,SLOT(setCurrentIndex(int)));
  connect(this,SIGNAL(setControlState(const QString &)),ui->lineEditContrlolState,SLOT(setText(const QString &)));
  connect(this,SIGNAL(setFault(const QString &)),ui->lineEditFault,SLOT(setText(const QString &)));
  connect(this,SIGNAL(setCalibrationAngle(const QString &)),ui->lineEditCalibration,SLOT(setText(const QString &)));

}

void MainWindow::SetupComs()
{
  m_coms = std::make_shared<DogBotN::SerialComsC>();

  //m_dogbotAPI.Connect(m_coms);

  m_coms->SetHandler(CPT_PWMState,[this](uint8_t *data,int size) mutable
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

  m_coms->SetHandler(CPT_Pong,[this](uint8_t *data,int size) mutable
  {
    struct PacketPingPongC *psp = (struct PacketPingPongC *) data;
    std::string displayStr = "Got pong ";

    displayStr += std::to_string((int) psp->m_deviceId);
    std::cout << displayStr << std::endl;
    emit setLogText(displayStr.c_str());
  });

  m_coms->SetHandler(CPT_ReportParam,[this](uint8_t *data,int size) mutable
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
    switch ((enum ComsParameterIndexT) psp->m_header.m_index) {
    case CPI_DriveTemp: {
      sprintf(buff,"\n Drive temp:%f ",((float) psp->m_data.float32[0]));
      displayStr += buff;
    } break;
    case CPI_VSUPPLY: {
      sprintf(buff,"\n Supply Voltage:%f ",((float) psp->m_data.uint16[0] / 1000.0f));
      displayStr += buff;
    } break;
    case CPI_PositionCal: {
      enum MotionCalibrationT calMode = (enum MotionCalibrationT) psp->m_data.uint8[0];
      switch(calMode) {
        case MC_Uncalibrated: emit setCalibrationState(0); break;
        case MC_Measuring: emit setCalibrationState(1);  break;
        case MC_Calibrated: emit setCalibrationState(2); break;
      default:
        sprintf(buff,"\n Unexpected calibration mode: %02x ",(unsigned) psp->m_data.uint8[0]);
        displayStr += buff;
      }
    } break;
    case CPI_CalibrationOffset: {
      sprintf(buff,"%f",(psp->m_data.float32[0] * 360.0f / (M_PI * 2.0)));
      emit setCalibrationAngle(buff);
    } break;
    case CPI_ControlState: {
      enum ControlStateT controlState = (enum ControlStateT) psp->m_data.uint8[0];
      switch(controlState) {
      case CS_EmergencyStop: emit setControlState("Emergency Stop"); break;
      case CS_FactoryCalibrate: emit setControlState("Factory Calibrate"); break;
      case CS_Standby: emit setControlState("Standby"); break;
      case CS_LowPower: emit setControlState("LowPower"); break;
      case CS_Manual: emit setControlState("Manual"); break;
      case CS_PositionCalibration: emit setControlState("Position Calibration"); break;
      case CS_SelfTest: emit setControlState("Self Test"); break;
      case CS_Teach: emit setControlState("Teach"); break;
      case CS_Fault: emit setControlState("Fault"); break;
      case CS_StartUp: emit setControlState("Startup"); break;
      default: {
          sprintf(buff,"Unknown state %d ", (int) psp->m_data.uint8[0]);
          emit setControlState(buff);
        } break;
      }
    } break;
    case CPI_FaultCode: {
      enum FaultCodeT faultCode = (enum FaultCodeT) psp->m_data.uint8[0];
      switch(faultCode) {
      case FC_Ok: emit setFault("Ok"); break;
      case FC_CalibrationFailed: emit setFault("Calibration Failed"); break;
      case FC_DriverFault: emit setFault("Driver Fault"); break;
      case FC_Internal: emit setFault("Internal error"); break;
      case FC_Internal5VRailOutOfRange: emit setFault("5V Rail Out Of Range"); break;
      case FC_InternalStoreFailed: emit setFault("Store failed"); break;
      case FC_InternalTiming: emit setFault("Timing error"); break;
      case FC_OverTemprature: emit setFault("Over Temprature"); break;
      case FC_OverVoltage: emit setFault("Over Voltage"); break;
      case FC_UnderVoltage: emit setFault("Under Voltage"); break;
      case FC_NoSensor: emit setFault("No sensor"); break;
      case FC_NoMotor: emit setFault("No motor"); break;
      case FC_PositionLost: emit setFault("Position Lost"); break;
      default: {
        sprintf(buff,"Unknown fault %d ", (int) psp->m_data.uint8[0]);
        emit setFault(buff);
        } break;
      }
    } break;
    default:
      break;
    }
    std::cout << "ReportParam: " << displayStr << std::endl;
    emit setLogText(displayStr.c_str());
  });

  m_coms->SetHandler(CPT_AnnounceId,[this](uint8_t *data,int size) mutable
  {
    if(size != sizeof(struct PacketDeviceIdC)) {
      std::cerr << "Packet length " << size << " expected " << sizeof(struct PacketDeviceIdC) << std::endl;
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

  m_coms->SetHandler(CPT_Servo,[this](uint8_t *data,int size) mutable
  {
    if(size != sizeof(struct PacketServoC)) {
      emit setLogText("Unexpected packet length.");
      return;
    }
    const PacketServoC *pkt = (const PacketServoC *) data;
    std::cout << "Servo " << (int) pkt->m_deviceId << " Position:" << pkt->m_position << " Torque: " << pkt->m_torque << " State:" << pkt->m_mode << std::endl;
  });

  m_coms->SetHandler(CPT_ServoReport,[this](uint8_t *data,int size) mutable
  {
    if(size != sizeof(struct PacketServoReportC)) {
      emit setLogText("Unexpected packet length.");
      return;
    }
    const PacketServoReportC *pkt = (const PacketServoReportC *) data;
    std::cout << "ServoReport " << (int) pkt->m_deviceId << ((pkt->m_mode & 1) ? " Abs " : " Rel ") << " Position:" << pkt->m_position << " Torque: " << pkt->m_torque  << " State:" << (int) pkt->m_mode << std::endl;
  });

  m_coms->SetHandler(CPT_Error,[this](uint8_t *data,int size) mutable
  {
    if(size != sizeof(struct PacketErrorC)) {
      emit setLogText("Unexpected packet length.");
      return;
    }
    const PacketErrorC *pkt = (const PacketErrorC *) data;
    std::cout << "Device: " << (int) pkt->m_deviceId << " Error:" << (int) pkt->m_errorCode << "  Data:" << (int)  pkt->m_errorData << " \n";
  });

}


MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::on_pushButtonConnect_clicked()
{
  if(m_coms->Open(ui->lineEditDevice->text().toStdString().c_str())) {
    ui->labelConnectionState->setText("Ok");
    emit setLogText("Connect ok");

    m_coms->SendQueryParam(m_targetDeviceId,CPI_ControlState);
    m_coms->SendQueryParam(m_targetDeviceId,CPI_FaultCode);
    m_coms->SendQueryParam(m_targetDeviceId,CPI_PositionCal);

  } else {
    ui->labelConnectionState->setText("Failed");
    emit setLogText("Connect failed");
  }
}

void MainWindow::on_pushButtonPWM_clicked()
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_PWMState,1);
  emit setLogText("Start PWM clicked");
  m_coms->SendQueryParam(m_targetDeviceId,CPI_PWMState);
}

void MainWindow::on_pushButtonStopPWM_clicked()
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_PWMState,0);
  emit setLogText("Stop PWM clicked");
  m_coms->SendQueryParam(m_targetDeviceId,CPI_PWMState);
}


void MainWindow::on_pushButtonPWMReport_clicked()
{
  if(!m_PWMReportRequested) {
    ui->pushButtonPWMReport->setText("Stop PWM Report");
    m_PWMReportRequested = true;
    m_coms->SendSetParam(m_targetDeviceId,CPI_PWMFullReport,1);
  } else {
    ui->pushButtonPWMReport->setText("Start PWM Report");
    m_PWMReportRequested = false;
    m_coms->SendSetParam(m_targetDeviceId,CPI_PWMFullReport,0);
  }
}

void MainWindow::on_pushButtonPing_clicked()
{
  emit setLogText("Ping");
  m_coms->SendPing(0);
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
  m_coms->SendSetParam(m_targetDeviceId,CPI_PWMMode,controlMode);
  m_coms->SendQueryParam(m_targetDeviceId,CPI_PWMMode);
}

void MainWindow::on_sliderPosition_sliderMoved(int position)
{
  m_position = position * 2.0 * 3.14159265359/ 360.0;
  m_coms->SendMoveWithEffort(m_targetDeviceId,m_position,m_torque);
}

void MainWindow::on_sliderTorque_sliderMoved(int torque)
{
  m_torque = torque / 10.0;
  m_coms->SendMoveWithEffort(m_targetDeviceId,m_position,m_torque);
}

void MainWindow::on_pushButtonQueryId_clicked()
{
  m_coms->SendQueryParam(m_targetDeviceId,CPI_BoardUID);
}

void MainWindow::on_pushButtonGetVersion_clicked()
{
  m_coms->SendQueryParam(m_targetDeviceId,CPI_FirmwareVersion);
}

void MainWindow::on_pushButtonState_clicked()
{
  m_coms->SendQueryParam(m_targetDeviceId,CPI_PWMState);
}

void MainWindow::on_pushButtonSetBridgeMode_clicked()
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_CANBridgeMode,1);
  m_coms->SendQueryParam(m_targetDeviceId,CPI_CANBridgeMode);
}

void MainWindow::on_pushButtonQueryDevices_clicked()
{
  m_coms->SendQueryDevices();
}

void MainWindow::on_pushButtonPing1_clicked()
{
  emit setLogText("Ping");
  m_coms->SendPing(m_targetDeviceId);
}

void MainWindow::on_pushButtonSetDeviceId_clicked()
{
  for(int i = 0;i < m_devices.size();i++) {
    m_coms->SendSetDeviceId(i+1,m_devices[i].m_uid[0],m_devices[i].m_uid[1]);
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
  m_coms->SendQueryParam(m_targetDeviceId,CPI_DRV8305_01);
}

void MainWindow::on_pushButtonDrv8305_2_clicked()
{
  m_coms->SendQueryParam(m_targetDeviceId,CPI_DRV8305_02);
}

void MainWindow::on_pushButtonDrv8305_3_clicked()
{
  m_coms->SendQueryParam(m_targetDeviceId,CPI_DRV8305_03);
}

void MainWindow::on_pushButtonDrv8305_4_clicked()
{
  m_coms->SendQueryParam(m_targetDeviceId,CPI_DRV8305_04);
}

void MainWindow::on_pushButtonDrv8305_5_clicked()
{
  m_coms->SendQueryParam(m_targetDeviceId,CPI_DRV8305_05);
}


void MainWindow::on_pushButtonTim1_clicked()
{
  m_coms->SendQueryParam(m_targetDeviceId,CPI_TIM1_SR);
}

void MainWindow::on_spinDeviceId_valueChanged(int arg1)
{
  m_targetDeviceId = arg1;
}

void MainWindow::on_pushButton_clicked()
{
  m_coms->SendQueryParam(m_targetDeviceId,CPI_VSUPPLY);
}

void MainWindow::on_comboBoxCalibration_activated(const QString &arg1)
{
  enum MotionCalibrationT calMode = MC_Uncalibrated;
  if(arg1 == "Uncalibrated") {
    calMode = MC_Uncalibrated;
  }
  if(arg1 == "Measuring") {
    calMode = MC_Measuring;
  }
  if(arg1 == "Calibrated") {
    calMode = MC_Calibrated;
  }
  m_coms->SendSetParam(m_targetDeviceId,CPI_PositionCal,calMode);
  m_coms->SendQueryParam(m_targetDeviceId,CPI_PositionCal);
}

void MainWindow::on_comboBoxControlState_activated(const QString &arg1)
{
  enum ControlStateT controlState = CS_Fault;
  if(arg1 == "Emergency Stop") {
    controlState = CS_EmergencyStop;
  }
  if(arg1 == "Factory Calibrate") {
    controlState = CS_FactoryCalibrate;
  }
  if(arg1 == "Low Power") {
    controlState = CS_LowPower;
  }
  if(arg1 == "Manual") {
    controlState = CS_Manual;
  }
  if(arg1 == "Position Calibration") {
    controlState = CS_PositionCalibration;
  }
  if(arg1 == "Teach") {
    controlState = CS_Teach;
  }
  if(arg1 == "Reset") {
    controlState = CS_StartUp;
  }
  if(controlState != CS_Fault)  {
    m_coms->SendSetParam(m_targetDeviceId,CPI_ControlState,controlState);
  }
}

void MainWindow::on_checkBoxIndicator_toggled(bool checked)
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_Indicator,checked);
}

void MainWindow::on_pushButtonDriveTemp_clicked()
{
  m_coms->SendQueryParam(m_targetDeviceId,CPI_DriveTemp);
}
