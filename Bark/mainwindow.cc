#include "mainwindow.hh"
#include "ui_mainwindow.h"
#include "dogbot/DogBotAPI.hh"
#include <iostream>
#include <QFileDialog>
#include <fstream>

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  ui->lineEditDevice->setText("/dev/ttyACM1");
//  ui->lineEditDevice->setText("/dev/tty.usbmodem401");


  SetupComs();

  m_servoTable = new ServoTable(m_dogbotAPI);
  ui->tableViewServoList->setModel(m_servoTable);

  connect(this,SIGNAL(setLogText(const QString &)),ui->textEditLog,SLOT(setText(const QString &)));
  connect(this,SIGNAL(setCalibrationState(int)),ui->comboBoxCalibration,SLOT(setCurrentIndex(int)));
  connect(this,SIGNAL(setControlState(const QString &)),ui->lineEditContrlolState,SLOT(setText(const QString &)));
  connect(this,SIGNAL(setControlMode(const QString &)),ui->comboBoxMotorControlMode,SLOT(setCurrentText(const QString &)));
  connect(this,SIGNAL(setFault(const QString &)),ui->lineEditFault,SLOT(setText(const QString &)));
  connect(this,SIGNAL(setCalibrationAngle(double)),ui->doubleSpinBoxCalibrationOffset,SLOT(setValue(double)));
  connect(this,SIGNAL(setOtherJoint(int)),ui->spinOtherJointId,SLOT(setValue(int)));
  connect(this,SIGNAL(setPositionRef(const QString &)),ui->comboBoxPositionRef,SLOT(setCurrentText(const QString &)));
  connect(this,SIGNAL(setIndicator(bool)),ui->checkBoxIndicator,SLOT(setChecked(bool)));


  connect(this,SIGNAL(setOtherJointGain(double)),ui->doubleSpinBoxJointRelGain,SLOT(setValue(double)));
  connect(this,SIGNAL(setOtherJointOffset(double)),ui->doubleSpinBoxJointRelOffset,SLOT(setValue(double)));

  connect(this,SIGNAL(setSupplyVoltage(QString)),ui->label_SupplyVoltage,SLOT(setText(QString)));
  connect(this,SIGNAL(setDriveTemperature(QString)),ui->label_DriverTemperature,SLOT(setText(QString)));
  connect(this,SIGNAL(setMotorIGain(double)),this,SLOT(updateIGain(double)));
  connect(this,SIGNAL(setMotorVelocity(double)),ui->doubleSpinBoxVelocity,SLOT(setValue(double)));
  connect(this,SIGNAL(setVelocityPGain(double)),ui->doubleSpinBoxVelocityGain,SLOT(setValue(double)));
  connect(this,SIGNAL(setVelocityIGain(double)),ui->doubleSpinBoxVelocityIGain,SLOT(setValue(double)));
  connect(this,SIGNAL(setDemandPhaseVelocity(double)),ui->doubleSpinBoxDemandVelocity,SLOT(setValue(double)));
  connect(this,SIGNAL(setVelocityLimit(double)),ui->doubleSpinBoxVelocityLimit,SLOT(setValue(double)));
  connect(this,SIGNAL(setPositionGain(double)),ui->doubleSpinBoxPositionGain,SLOT(setValue(double)));

  startTimer(10);
}

void MainWindow::on_checkBoxBounce_clicked(bool checked)
{
    EnableBounce(checked);
}

void MainWindow::EnableBounce(bool state)
{
  m_startBounce = std::chrono::steady_clock::now();
  m_bounceRunning = state;
}

void MainWindow::timerEvent(QTimerEvent *)
{
  //std::cout << "Setting " << m_servoAngle * 360.0 / (2.0* M_PI) << std::endl;
  ui->doubleSpinBoxPostion->setValue(m_servoAngle * 360.0 / (2.0* M_PI));
  ui->doubleSpinBoxTorque_2->setValue(m_servoTorque);

  std::string posRefStr = "unknown";
  switch(m_servoRef)
  {
    case PR_Relative: posRefStr = "Relative"; break;
    case PR_Absolute: posRefStr = "Absolute"; break;
    case PR_OtherJointRelative: posRefStr = "Relative Other"; break;
    case PR_OtherJointAbsolute: posRefStr = "Absolute Other"; break;
  }
  ui->labelCalState->setText(posRefStr.c_str());

  if(!m_bounceRunning)
    return ;

  auto now = std::chrono::steady_clock::now();

  std::chrono::duration<double> diff = now-m_startBounce;

  float phase = diff.count() * m_omega;
  float height = sin(phase) * m_bounceRange + m_bounceOffset;

  float position[3] = { 0,0,0 };
  float angles[3] = { 0,0,0 };
  position[2] = height;
  if(!m_legKinematics.Inverse(position,angles)) {
    std::cout << "Kinematics failed. \n";
    return ;
  }

  std::cout << " Pelvis: " << angles[0] <<  " Hip:" << angles[1] << " Knee:" <<  angles[2] << std::endl;

  m_coms->SendMoveWithEffort(m_hipJointId,m_reverseHip ? -angles[1] : angles[1],m_bounceTorque,PR_Absolute);
  m_coms->SendMoveWithEffort(m_kneeJointId,-angles[2],m_bounceTorque,PR_Absolute);
  //  int m_hipJointId = 1;
  //int m_kneeJointId = 2;
}


void MainWindow::on_doubleSpinBoxBounceTorque_valueChanged(double arg1)
{
  m_bounceTorque = arg1;
}

void MainWindow::on_doubleSpinBoxOmega_valueChanged(double arg1)
{
  m_omega = arg1;
}

void MainWindow::on_verticalSliderBounceOffset_valueChanged(int value)
{
  m_bounceOffset = 0.3 + (float) value * 0.01;
}


void MainWindow::on_verticalSliderBounceRange_valueChanged(int value)
{
  m_bounceRange = (float) value * 0.01;
}

bool MainWindow::ProcessParam(struct PacketParam8ByteC *psp,std::string &displayStr)
{
  char buff[64];
  bool ret = true;

  switch ((enum ComsParameterIndexT) psp->m_header.m_index) {
  case CPI_DriveTemp: {
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      sprintf(buff,"%3.1f",((float) psp->m_data.float32[0]));
      emit setDriveTemperature(buff);
    }
    ret = false;
    sprintf(buff,"\n Drive temp:%3.1f ",((float) psp->m_data.float32[0]));
    displayStr += buff;
  } break;
  case CPI_VSUPPLY: {
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      sprintf(buff,"%2.1f",((float) psp->m_data.uint16[0] / 1000.0f));
      emit setSupplyVoltage(buff);
    }
    ret = false;
    sprintf(buff,"\n Supply Voltage:%2.1f ",((float) psp->m_data.uint16[0] / 1000.0f));
    displayStr += buff;
  } break;
  case CPI_PositionCal: {
    enum MotionCalibrationT calMode = (enum MotionCalibrationT) psp->m_data.uint8[0];
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      switch(calMode) {
        case MC_Uncalibrated: emit setCalibrationState(0); break;
        case MC_Measuring: emit setCalibrationState(1);  break;
        case MC_Calibrated: emit setCalibrationState(2); break;
      default:
        sprintf(buff,"\n Unexpected calibration mode: %02x ",(unsigned) psp->m_data.uint8[0]);
        displayStr += buff;
      }
    }
  } break;
  case CPI_CalibrationOffset: {
    float calAngleDeg =  (psp->m_data.float32[0] * 360.0f / (M_PI * 2.0));
    sprintf(buff,"%f",calAngleDeg);
    displayStr += buff;
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      emit setCalibrationAngle(calAngleDeg);
    }
  } break;
  case CPI_ControlState: {
    enum ControlStateT controlState = (enum ControlStateT) psp->m_data.uint8[0];
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      emit setControlState(DogBotN::ControlStateToString(controlState));
    }
  } break;
  case CPI_FaultCode: {
    enum FaultCodeT faultCode = (enum FaultCodeT) psp->m_data.uint8[0];
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      emit setFault(DogBotN::FaultCodeToString(faultCode));
    }
  } break;
  case CPI_PWMMode: {
    enum PWMControlModeT controlMode =  (enum PWMControlModeT) psp->m_data.uint8[0];
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      printf("Setting control mode %d ",(int) psp->m_data.uint8[0]);
      m_controlMode = controlMode;
      switch(controlMode) {
      case CM_Idle:     emit setControlMode("Idle"); break;
      case CM_Break:    emit setControlMode("Break"); break;
      case CM_Torque:   emit setControlMode("Torque"); break;
      case CM_Velocity: emit setControlMode("Velocity"); break;
      case CM_Position: emit setControlMode("Position"); break;
      default:
        printf("Unhandled control mode %d ",(int) psp->m_data.uint8[0]);
      }
    }
  } break;
  case CPI_OtherJoint: {
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      uint8_t otherJointId = psp->m_data.uint8[0];
      setOtherJoint(otherJointId);
    }
  } break;
  case CPI_PositionRef: {
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      enum PositionReferenceT posRef = (enum PositionReferenceT) psp->m_data.uint8[0];
      switch(posRef)
      {
        case PR_Relative: emit setPositionRef("Relative"); break;
        case PR_Absolute: emit setPositionRef("Absolute"); break;
        case PR_OtherJointRelative: emit setPositionRef("RelativeOther"); break;
        case PR_OtherJointAbsolute: emit setPositionRef("AbsoluteOther"); break;
      }
    }
  } break;
  case CPI_Indicator: {
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      emit setIndicator(psp->m_data.uint8[0] > 0);
    }
  } break;
  case CPI_OtherJointGain:
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      emit setOtherJointGain(psp->m_data.float32[0]);
    }
    break;
  case CPI_OtherJointOffset:
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      emit setOtherJointOffset(psp->m_data.float32[0] * 360.0 / (2.0 * M_PI));
    }
    break;
  case CPI_MotorInductance:
    sprintf(buff,"\n Inductance: %f ", psp->m_data.float32[0]);
    displayStr += buff;
    break;
  case CPI_MotorResistance:
    sprintf(buff,"\n Resistance: %f ",psp->m_data.float32[0]);
    displayStr += buff;
    break;
  case CPI_MotorIGain:
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      emit setMotorIGain(psp->m_data.float32[0]);
    }
    sprintf(buff,"\n IGain: %f ",psp->m_data.float32[0]);
    displayStr += buff;
    break;
  case CPI_MotorPGain:
    sprintf(buff,"\n PGain: %f ",psp->m_data.float32[0]);
    displayStr += buff;
    break;
  case CPI_PhaseVelocity:
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      emit setMotorVelocity(psp->m_data.float32[0]);
    }
    sprintf(buff,"\n Velocity: %f ",psp->m_data.float32[0]);
    displayStr += buff;
    ret = false;
    break;
  case CPI_VelocityPGain:
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      emit setVelocityPGain(psp->m_data.float32[0]);
    }
    sprintf(buff,"\n VelocityPGain: %f ",psp->m_data.float32[0]);
    displayStr += buff;
    //ret = false;
    break;
  case CPI_VelocityIGain:
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      emit setVelocityIGain(psp->m_data.float32[0]);
    }
    sprintf(buff,"\n VelocityIGain: %f ",psp->m_data.float32[0]);
    displayStr += buff;
    //ret = false;
    break;
  case CPI_VelocityLimit:
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      emit setVelocityLimit(psp->m_data.float32[0]);
    }
    sprintf(buff,"\n VelocityLimit: %f ",psp->m_data.float32[0]);
    displayStr += buff;
    //ret = false;
    break;
  case CPI_DemandPhaseVelocity:
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      emit setDemandPhaseVelocity(psp->m_data.float32[0]);
    }
    sprintf(buff,"\n DemandVelocity: %f ",psp->m_data.float32[0]);
    displayStr += buff;
    ret = false;
    break;
  case CPI_PositionGain:
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      emit setPositionGain(psp->m_data.float32[0]);
    }
    sprintf(buff,"\n PositionGain: %f ",psp->m_data.float32[0]);
    displayStr += buff;
    ret = false;
    break;
  case CPI_DRV8305_01:
  case CPI_DRV8305_02:
  case CPI_DRV8305_03:
  case CPI_DRV8305_04:
  case CPI_DRV8305_05: {
    int reg = (psp->m_header.m_index - (int) CPI_DRV8305_01)+1;
    sprintf(buff,"\n Reg %d contents: %04X ",reg,(int) psp->m_data.uint16[0]);
    displayStr += buff;

   } break;
  default:
    break;
  }

  return ret;
}

void MainWindow::SetupComs()
{
  m_coms = std::make_shared<DogBotN::SerialComsC>();

  m_dogbotAPI = std::make_shared<DogBotN::DogBotAPIC>(m_coms);

  m_dogbotAPI->Init();

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

  m_coms->SetHandler(CPT_SetParam,[this](uint8_t *data,int size) mutable
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
    ProcessParam(psp,displayStr);
    std::cout << "SetParam: " << displayStr << std::endl;
    emit setLogText(displayStr.c_str());
  });

  m_coms->SetHandler(CPT_ReportParam,[this](uint8_t *data,int size) mutable
  {
//    printf("Got ReportParam.  Size:%d \n",size);
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
    if(ProcessParam(psp,displayStr)) {
      std::cout << "ReportParam: " << displayStr << std::endl;
      emit setLogText(displayStr.c_str());
    }
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
    std::cout << "Servo " << (int) pkt->m_deviceId << " Position:" << pkt->m_position << " Torque: " << pkt->m_torqueLimit << " State:" << pkt->m_mode << std::endl;
  });


  m_coms->SetHandler(CPT_ServoReport,[this](uint8_t *data,int size) mutable
  {
    if(size != sizeof(struct PacketServoReportC)) {
      emit setLogText("Unexpected packet length.");
      return;
    }
    const PacketServoReportC *pkt = (const PacketServoReportC *) data;
    if(pkt->m_deviceId == m_targetDeviceId) {
      m_servoAngle = m_coms->PositionReport2Angle(pkt->m_position);
      m_servoTorque = m_coms->TorqueReport2Value(pkt->m_torque);
      m_servoRef = (enum PositionReferenceT) (pkt->m_mode & 0x3);
    }

    //std::cout << "ServoReport " << (int) pkt->m_deviceId << ((pkt->m_mode & 1) ? " Abs " : " Rel ") << " Position:" << pkt->m_position << " Torque: " << pkt->m_torque  << " State:" << (int) pkt->m_mode << std::endl;
  });

  m_coms->SetHandler(CPT_Error,[this](uint8_t *data,int size) mutable
  {
    if(size != sizeof(struct PacketErrorC)) {
      emit setLogText("Unexpected packet length.");
      return;
    }
    const PacketErrorC *pkt = (const PacketErrorC *) data;
    std::cout << "Device: " << (int) pkt->m_deviceId << " Error:" << (int) pkt->m_errorCode << "  Data:" << (int)  pkt->m_causeType << " " << (int)  pkt->m_errorData << " " << std::endl;
  });

}


MainWindow::~MainWindow()
{
  delete ui;
  delete m_servoTable;
}

void MainWindow::QueryAll()
{
  m_coms->SendQueryParam(m_targetDeviceId,CPI_ControlState);
  m_coms->SendQueryParam(m_targetDeviceId,CPI_FaultCode);
  m_coms->SendQueryParam(m_targetDeviceId,CPI_PositionCal);
  m_coms->SendQueryParam(m_targetDeviceId,CPI_PositionRef);
  m_coms->SendQueryParam(m_targetDeviceId,CPI_PWMMode);
  m_coms->SendQueryParam(m_targetDeviceId,CPI_CalibrationOffset);
  m_coms->SendQueryParam(m_targetDeviceId,CPI_OtherJoint);
  m_coms->SendQueryParam(m_targetDeviceId,CPI_Indicator);
  m_coms->SendQueryParam(m_targetDeviceId,CPI_OtherJointOffset);
  m_coms->SendQueryParam(m_targetDeviceId,CPI_OtherJointGain);
  m_coms->SendQueryParam(m_targetDeviceId,CPI_MotorIGain);
  m_coms->SendQueryParam(m_targetDeviceId,CPI_VelocityPGain);
  m_coms->SendQueryParam(m_targetDeviceId,CPI_VelocityIGain);
  m_coms->SendQueryParam(m_targetDeviceId,CPI_VelocityLimit);
  m_coms->SendQueryParam(m_targetDeviceId,CPI_PositionGain);
}

void MainWindow::on_pushButtonConnect_clicked()
{
  if(m_coms->Open(ui->lineEditDevice->text().toStdString().c_str())) {
    ui->labelConnectionState->setText("Ok");
    emit setLogText("Connect ok");

    // Put connected device into bridge mode.
    m_coms->SendSetParam(0,CPI_CANBridgeMode,1);

    QueryAll();
  } else {
    ui->labelConnectionState->setText("Failed");
    emit setLogText("Connect failed");
  }
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
  m_controlMode = controlMode;
  m_coms->SendSetParam(m_targetDeviceId,CPI_PWMMode,controlMode);
}

void MainWindow::on_sliderPosition_sliderMoved(int position)
{
  m_position = position * 2.0 * 3.14159265359/ 360.0;
  std::cerr << "Mode: " << (int) m_controlMode << std::endl;
  // Convert position to radians
  switch(m_controlMode)
  {
  case CM_Position:
  {
    std::cerr << "Sending pos: " << std::endl;
    m_position = position * 2.0 * 3.14159265359/ 360.0;
    std::cout << "Sending move. Pos: " << m_position << " Torque:" << m_torque << " Ref:" << (int) g_positionReference << std::endl << std::flush;
    m_coms->SendMoveWithEffort(m_targetDeviceId,m_position,m_torque,g_positionReference);
    ui->doubleSpinBoxDemandPosition->setValue(position);
  } break;
  case CM_Velocity:
  {
    float velocity = position * 2.0 * 3.14159265359/ 360.0;
    std::cout << "Sending velocity. Pos: " << velocity << " Torque:" << m_torque << " Ref:" << (int) g_positionReference << std::endl << std::flush;
    m_coms->SendVelocityWithEffort(m_targetDeviceId,velocity,m_torque);
  } break;
  default:
    break;
  }

}

void MainWindow::on_sliderTorque_sliderMoved(int torque)
{
  m_torque = torque / 10.0;
  std::cout << "Sending move. Pos: " << m_position << " Torque:" << m_torque << " Ref:" << (int) g_positionReference << std::endl << std::flush;
  m_coms->SendMoveWithEffort(m_targetDeviceId,m_position,m_torque,g_positionReference);
  ui->doubleSpinBoxTorqueLimit->setValue(m_torque);
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
  QueryAll();
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

void MainWindow::on_spinOtherJointId_valueChanged(int arg1)
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_OtherJoint,arg1);
}

void MainWindow::on_comboBox_activated(const QString &arg1)
{
  if(arg1 == "Relative") {
    g_positionReference = PR_Relative;
  }
  if(arg1 == "Absolute") {
    g_positionReference = PR_Absolute;
  }
  if(arg1 == "RelativeOther") {
    g_positionReference = PR_OtherJointRelative;
  }
  if(arg1 == "AbsoluteOther") {
    g_positionReference = PR_OtherJointAbsolute;
  }
  std::cout << "Changing send positions to " << (int) g_positionReference << std::endl << std::flush;
}

void MainWindow::on_comboBoxPositionRef_activated(const QString &arg1)
{
  enum PositionReferenceT positionReference  = PR_Relative;
  if(arg1 == "Relative") {
    positionReference = PR_Relative;
  }
  if(arg1 == "Absolute") {
    positionReference = PR_Absolute;
  }
  if(arg1 == "RelativeOther") {
    positionReference = PR_OtherJointRelative;
  }
  if(arg1 == "AbsoluteOther") {
    positionReference = PR_OtherJointAbsolute;
  }
  std::cout << "Changing requested ref to " << (int) positionReference << std::endl  << std::flush;
  m_coms->SendSetParam(m_targetDeviceId,CPI_PositionRef,(uint8_t) positionReference);
}

void MainWindow::on_pushButton_2_clicked()
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_DebugIndex,(uint8_t) 7);
}

void MainWindow::on_pushButtonCalZero_clicked()
{
  std::cout << "Sending cal zero. " << std::endl;
  m_coms->SendCalZero(m_targetDeviceId);
}

void MainWindow::on_doubleSpinBoxJointRelGain_valueChanged(double arg1)
{
   m_coms->SendSetParam(m_targetDeviceId,CPI_OtherJointGain,(float) arg1);
}

void MainWindow::on_doubleSpinBoxJointRelOffset_valueChanged(double arg1)
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_OtherJointOffset,(float) (arg1 * M_PI * 2.0 / 360.0f));
}

void MainWindow::on_doubleSpinBoxDemandPosition_editingFinished()
{
  double demandAngleDeg = ui->doubleSpinBoxDemandPosition->value();
  double newPosition = (demandAngleDeg * 2.0 * M_PI) / 360.0;
  m_position = newPosition;
  std::cout << "Sending move. Pos: " << m_position << " Torque:" << m_torque << " Ref:" << (int) g_positionReference << std::endl << std::flush;
  m_coms->SendMoveWithEffort(m_targetDeviceId,m_position,m_torque,g_positionReference);
  ui->sliderPosition->setValue(demandAngleDeg);
}

void MainWindow::on_doubleSpinBoxTorqueLimit_editingFinished()
{
  double newTorqueLimit = ui->doubleSpinBoxTorqueLimit->value();
  m_torque = newTorqueLimit;
  std::cout << "Sending move. Pos: " << m_position << " Torque:" << m_torque << " Ref:" << (int) g_positionReference << std::endl  << std::flush;
  m_coms->SendMoveWithEffort(m_targetDeviceId,m_position,m_torque,g_positionReference);
  ui->sliderTorque->setValue(newTorqueLimit * 10.0);
}

void MainWindow::on_doubleSpinBoxDemandPosition_valueChanged(double arg1)
{
  std::cout << "DemandPosition: " << arg1 << std::endl;
}

void MainWindow::on_doubleSpinBoxCalibrationOffset_editingFinished()
{
  float calAngleRad =(ui->doubleSpinBoxCalibrationOffset->value() * (M_PI * 2.0) / 360.0f);
  m_coms->SendSetParam(m_targetDeviceId,CPI_CalibrationOffset,calAngleRad);

}

void MainWindow::on_spinBoxHipJointId_valueChanged(int arg1)
{
   m_hipJointId = arg1;
}

void MainWindow::on_spinBoxKneeJointId_valueChanged(int arg1)
{
   m_kneeJointId = arg1;
}

void MainWindow::on_checkBoxReverseHip_clicked(bool checked)
{
  m_reverseHip = checked;
}

void MainWindow::on_pushButton_3_clicked()
{
  m_coms->SendQueryParam(m_targetDeviceId,CPI_MotorResistance);
}

void MainWindow::on_pushButton_4_clicked()
{
  m_coms->SendQueryParam(m_targetDeviceId,CPI_MotorInductance);
}

void MainWindow::on_doubleSpinBoxIGain_valueChanged(double arg1)
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_MotorIGain,(float) arg1);
}

void MainWindow::updateIGain(double arg1)
{
  ui->doubleSpinBoxIGain->blockSignals(true);
  ui->doubleSpinBoxIGain->setValue(arg1);
  ui->doubleSpinBoxIGain->blockSignals(false);
}

void MainWindow::on_sliderPosition_valueChanged(int value)
{

}

void MainWindow::on_doubleSpinBoxVelocityGain_valueChanged(double arg1)
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_VelocityPGain,(float) arg1);
}

void MainWindow::on_doubleSpinBoxVelocityIGain_valueChanged(double arg1)
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_VelocityIGain,(float) arg1);
}

void MainWindow::on_doubleSpinBoxVelocityLimit_valueChanged(double arg1)
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_VelocityLimit,(float) arg1);
}

void MainWindow::on_doubleSpinBoxPositionGain_valueChanged(double arg1)
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_PositionGain,(float) arg1);
}

void MainWindow::on_checkBoxFan_toggled(bool checked)
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_AuxPower,checked ? 1 : 0);
}
