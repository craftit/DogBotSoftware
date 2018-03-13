#include "mainwindow.hh"
#include "ui_mainwindow.h"
#include "dogbot/DogBotAPI.hh"
#include "dogbot/ComsSerial.hh"
#include "dogbot/ComsProxy.hh"
#include <iostream>
#include <QFileDialog>
#include <QMessageBox>
#include <fstream>

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  ui->comboBoxConnect->setCurrentText("local");
  ui->tabWidget->setCurrentIndex(0);
  ui->tabWidget->tabBar()->hide();

  //ui->lineEditDevice->setText("usb");
  //ui->lineEditDevice->setText("/dev/ttyACM1");
  //ui->lineEditDevice->setText("/dev/tty.usbmodem401");

  SetupComs();

  m_servoTable = new ServoTable(m_dogBotAPI);
  ui->tableViewServoList->setModel(m_servoTable);
  ui->tableViewServoList->horizontalHeader()->setStretchLastSection(true);
  ui->tableViewServoList->setColumnWidth(ServoTable::ColumnDeviceId,25);
  ui->tableViewServoList->setColumnWidth(ServoTable::ColumnName,200);
  ui->tableViewServoList->setColumnWidth(ServoTable::ColumnDynamic,70);
  ui->tableViewServoList->setColumnWidth(ServoTable::ColumnAngle,70);
  ui->tableViewServoList->setColumnWidth(ServoTable::ColumnSpeed,70);
  ui->tableViewServoList->setColumnWidth(ServoTable::ColumnTorque,70);
  ui->tableViewServoList->setColumnWidth(ServoTable::ColumnSupplyVoltage,70);

  connect(this,SIGNAL(setLogText(const QString &)),ui->textEditLog,SLOT(setText(const QString &)));
  connect(this,SIGNAL(setCalibrationState(int)),ui->comboBoxCalibration,SLOT(setCurrentIndex(int)));
  connect(this,SIGNAL(setControlState(const QString &)),ui->comboBoxControlState,SLOT(setCurrentText(QString)));
  connect(this,SIGNAL(setControlMode(const QString &)),ui->comboBoxMotorControlMode,SLOT(setCurrentText(const QString &)));
  connect(this,SIGNAL(setFault(const QString &)),ui->lineEditFault,SLOT(setText(const QString &)));
  connect(this,SIGNAL(setCalibrationAngle(double)),ui->doubleSpinBoxCalibrationOffset,SLOT(setValue(double)));
  connect(this,SIGNAL(setOtherJoint(int)),ui->spinOtherJointId,SLOT(setValue(int)));
  connect(this,SIGNAL(setPositionRef(const QString &)),ui->comboBoxPositionRef,SLOT(setCurrentText(const QString &)));
  connect(this,SIGNAL(setIndicator(bool)),ui->checkBoxIndicator,SLOT(setChecked(bool)));

  connect(this,SIGNAL(setOtherJointGain(double)),ui->doubleSpinBoxJointRelGain,SLOT(setValue(double)));
  connect(this,SIGNAL(setOtherJointOffset(double)),ui->doubleSpinBoxJointRelOffset,SLOT(setValue(double)));

  connect(this,SIGNAL(setSupplyVoltage(QString)),ui->lineEdit_SupplyVoltage,SLOT(setText(QString)));
  connect(this,SIGNAL(setDriveTemperature(QString)),ui->lineEdit_DriveTemperature,SLOT(setText(QString)));
  connect(this,SIGNAL(setMotorTemperature(QString)),ui->lineEdit_MotorTemperature,SLOT(setText(QString)));
  connect(this,SIGNAL(setFanTemperatureThreshold(QString)),ui->lineEditFanTempThreshold,SLOT(setText(QString)));

  connect(this,SIGNAL(setMotorIGain(double)),this,SLOT(updateIGain(double)));
  connect(this,SIGNAL(setMotorVelocity(double)),ui->doubleSpinBoxVelocity,SLOT(setValue(double)));
  connect(this,SIGNAL(setVelocityPGain(double)),ui->doubleSpinBoxVelocityGain,SLOT(setValue(double)));
  connect(this,SIGNAL(setVelocityIGain(double)),ui->doubleSpinBoxVelocityIGain,SLOT(setValue(double)));
  connect(this,SIGNAL(setDemandPhaseVelocity(double)),ui->doubleSpinBoxDemandVelocity,SLOT(setValue(double)));
  connect(this,SIGNAL(setVelocityLimit(double)),ui->doubleSpinBoxVelocityLimit,SLOT(setValue(double)));
  connect(this,SIGNAL(setPositionGain(double)),ui->doubleSpinBoxPositionGain,SLOT(setValue(double)));
  connect(this,SIGNAL(setHomeIndexOffset(double)),ui->doubleSpinBoxHomeIndexOffset,SLOT(setValue(double)));
  connect(this,SIGNAL(setHallSensors(QString)),ui->lineEditHallReadings,SLOT(setText(QString)));
  connect(this,SIGNAL(setUSBDrops(QString)),ui->lineEditUSBDrop,SLOT(setText(QString)));
  connect(this,SIGNAL(setUSBErrors(QString)),ui->lineEditUSBError,SLOT(setText(QString)));
  connect(this,SIGNAL(setCANDrops(QString)),ui->lineEditCANDrop,SLOT(setText(QString)));
  connect(this,SIGNAL(setCANErrors(QString)),ui->lineEditCANError,SLOT(setText(QString)));
  connect(this,SIGNAL(setMainLoopTimeout(QString)),ui->lineEditMainLoopTimeout,SLOT(setText(QString)));
  connect(this,SIGNAL(setFaultMap(QString)),ui->lineEditFaultMap,SLOT(setText(QString)));
  connect(this,SIGNAL(setIndexSensor(bool)),ui->checkBoxIndexSensor,SLOT(setChecked(bool)));
  connect(this,SIGNAL(setJointRelativeEnabled(bool)),ui->checkBoxJointRelative,SLOT(setChecked(bool)));
  connect(this,SIGNAL(setFanMode(const QString &)),ui->comboBoxFanState,SLOT(setCurrentText(QString)));
  connect(this,SIGNAL(updatePosition(double)),this,SLOT(on_updatePosition(double)));

  connect(this,SIGNAL(callLocalProcessParam(PacketParam8ByteC)),this,SLOT(LocalProcessParam(PacketParam8ByteC)));


  startTimer(10);

  m_displayQuery.push_back(CPI_ControlState);
  m_displayQuery.push_back(CPI_FaultCode);
  m_displayQuery.push_back(CPI_HomedState);
  m_displayQuery.push_back(CPI_PositionRef);
  m_displayQuery.push_back(CPI_PWMMode);
  m_displayQuery.push_back(CPI_CalibrationOffset);
  m_displayQuery.push_back(CPI_OtherJoint);
  m_displayQuery.push_back(CPI_Indicator);
  m_displayQuery.push_back(CPI_OtherJointOffset);
  m_displayQuery.push_back(CPI_OtherJointGain);
  m_displayQuery.push_back(CPI_MotorIGain);
  m_displayQuery.push_back(CPI_VelocityPGain);
  m_displayQuery.push_back(CPI_VelocityIGain);
  m_displayQuery.push_back(CPI_VelocityLimit);
  m_displayQuery.push_back(CPI_PositionGain);
  m_displayQuery.push_back(CPI_homeIndexPosition);
  m_displayQuery.push_back(CPI_MaxCurrent);
  m_displayQuery.push_back(CPI_USBPacketDrops);
  m_displayQuery.push_back(CPI_USBPacketErrors);
  m_displayQuery.push_back(CPI_CANPacketDrops);
  m_displayQuery.push_back(CPI_CANPacketErrors);
  m_displayQuery.push_back(CPI_MainLoopTimeout);
  m_displayQuery.push_back(CPI_FaultState);
  m_displayQuery.push_back(CPI_FanMode);
  m_displayQuery.push_back(CPI_FanTemperatureThreshold);
  m_displayQuery.push_back(CPI_EndStopEnable);
  m_displayQuery.push_back(CPI_EndStopStart);
  m_displayQuery.push_back(CPI_EndStopEnd);
  m_displayQuery.push_back(CPI_EndStopTargetBreakForce);
  m_displayQuery.push_back(CPI_JointInertia);

  // Update default position
  ui->sliderTorque->setSliderPosition(m_torque*10.0);
  ui->doubleSpinBoxTorqueLimit->setValue(m_torque);
}

void MainWindow::timerEvent(QTimerEvent *)
{
  if(m_toQuery < (int) m_displayQuery.size() && m_coms->IsReady()) {
    m_coms->SendQueryParam(m_targetDeviceId,m_displayQuery[m_toQuery]);
    m_toQuery++;
  }

  //std::cout << "Setting " << m_servoAngle * 360.0 / (2.0* M_PI) << std::endl;
  ui->doubleSpinBoxPostion->setValue(m_servoAngle * 360.0 / (2.0* M_PI));
  ui->doubleSpinBoxTorque_2->setValue(m_servoTorque);

  std::string posRefStr = "unknown";
  switch(m_servoRef)
  {
    case PR_Relative: posRefStr = "Relative"; break;
    case PR_Absolute: posRefStr = "Absolute"; break;
  }
  ui->labelCalState->setText(posRefStr.c_str());
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
  case CPI_MotorTemp: {
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      float temp = ((float) psp->m_data.float32[0]);
      if(temp < -60 && false) {
        sprintf(buff,"Fault");
      } else {
        sprintf(buff,"%3.1f",temp);
      }
      emit setMotorTemperature(buff);
    }
    ret = false;
    sprintf(buff,"\n Motor temp:%3.1f ",((float) psp->m_data.float32[0]));
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
  case CPI_FanTemperatureThreshold: {
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      sprintf(buff,"%2.1f",((float) psp->m_data.float32[0]));
      emit setFanTemperatureThreshold(buff);
    }
    ret = false;
    sprintf(buff,"\n Temperature threshold:%2.1f ",((float) psp->m_data.float32[0]));
    displayStr += buff;
  } break;
  case CPI_HomedState: {
    enum MotionHomedStateT calMode = (enum MotionHomedStateT) psp->m_data.uint8[0];
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      switch(calMode) {
        case MHS_Lost: emit setCalibrationState(0); break;
        case MHS_Measuring: emit setCalibrationState(1);  break;
        case MHS_Homed: emit setCalibrationState(2); break;
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
    std::cerr << "Fault " << ((int) psp->m_header.m_deviceId) << " : " << DogBotN::FaultCodeToString(faultCode) << std::endl;
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      emit setFault(DogBotN::FaultCodeToString(faultCode));
    }
  } break;
  case CPI_PWMMode: {
    enum PWMControlDynamicT controlMode =  (enum PWMControlDynamicT) psp->m_data.uint8[0];
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      //printf("Setting control mode %d ",(int) psp->m_data.uint8[0]);
      m_controlMode = controlMode;
      emit setControlMode(DogBotN::ControlDynamicToString(controlMode));
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
      }
    }
  } break;
  case CPI_JointRelative: {
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      enum JointRelativeT jointRelativeMode = (enum JointRelativeT) psp->m_data.uint8[0];
      emit setJointRelativeEnabled(jointRelativeMode != JR_Isolated);
    }
  } break;
  case CPI_Indicator: {
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
  case CPI_FanMode:
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      switch((FanModeT) psp->m_data.uint8[0])
      {
      case FM_On:
        emit setFanMode("On");
        break;
      case FM_Off:
        emit setFanMode("Off");
        break;
      case FM_Auto:
        emit setFanMode("Auto");
        break;
      }
    }
    sprintf(buff,"\n Fan mode: %d ",psp->m_data.uint8[0]);
    displayStr += buff;
    ret = false;
    break;
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
  case CPI_5VRail: {
    sprintf(buff,"\n 5VRail: %f ",psp->m_data.float32[0]);
    displayStr += buff;
    break;
  }
  case CPI_homeIndexPosition: {
    double homePos = psp->m_data.float32[0] * 360.0 / (M_PI * 2.0);
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      emit setHomeIndexOffset(homePos);
    }
    sprintf(buff,"\n %d Home: %f ",psp->m_header.m_deviceId,homePos);
    displayStr += buff;
  } break;
  case CPI_EndStopEnd: {
    double homePos = psp->m_data.float32[0] * 360.0 / (M_PI * 2.0);
    sprintf(buff,"\n %d EndStopEnd: %f ",psp->m_header.m_deviceId,homePos);
    displayStr += buff;
  } break;
  case CPI_EndStopStart: {
    double homePos = psp->m_data.float32[0] * 360.0 / (M_PI * 2.0);
    sprintf(buff,"\n %d EndStopStart: %f ",psp->m_header.m_deviceId,homePos);
    displayStr += buff;
  } break;
  case CPI_HallSensors: {
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      sprintf(buff,"%5d %5d %5d ",
              (int) psp->m_data.uint16[0],(int) psp->m_data.uint16[1],(int) psp->m_data.uint16[2]);
      emit setHallSensors(buff);
    }

    sprintf(buff,"\n %d Hall: %d %d %d ",
            psp->m_header.m_deviceId,(int) psp->m_data.uint16[0],(int) psp->m_data.uint16[1],(int) psp->m_data.uint16[2]);
    displayStr += buff;
    ret = false;
  } break;
  case CPI_USBPacketDrops: {
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      sprintf(buff,"%d ", (int) psp->m_data.uint32[0]);
      emit setUSBDrops(buff);
    }
    sprintf(buff,"\n %d USB Packet drop: %d ",psp->m_header.m_deviceId,psp->m_data.uint32[0]);
    displayStr += buff;
    ret = false;
  } break;
  case CPI_USBPacketErrors: {
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      sprintf(buff,"%d ", (int) psp->m_data.uint32[0]);
      emit setUSBErrors(buff);
    }
    sprintf(buff,"\n %d USB Packet error: %d ",psp->m_header.m_deviceId,psp->m_data.uint32[0]);
    displayStr += buff;
  } break;
  case CPI_CANPacketDrops: {
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      sprintf(buff,"%d ", (int) psp->m_data.uint32[0]);
      emit setCANDrops(buff);
    }
    sprintf(buff,"\n %d CAN Packet drop: %d ",psp->m_header.m_deviceId,psp->m_data.uint32[0]);
    displayStr += buff;
    ret = false;
  } break;
  case CPI_CANPacketErrors: {
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      sprintf(buff,"%d ", (int) psp->m_data.uint32[0]);
      emit setCANErrors(buff);
    }
    sprintf(buff,"\n %d CAN Packet error: %d ",psp->m_header.m_deviceId,psp->m_data.uint32[0]);
    displayStr += buff;
  } break;
  case CPI_FaultState: {
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      sprintf(buff,"%X ", (int) psp->m_data.uint32[0]);
      emit setFaultMap(buff);
    }
    sprintf(buff,"\n %d Fault state: %X ",psp->m_header.m_deviceId,psp->m_data.uint32[0]);
    displayStr += buff;
  } break;
  case CPI_IndexSensor: {
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      emit setIndexSensor(psp->m_data.uint8[0] != 0);
    }
    sprintf(buff,"\n %d Index sensor: %X ",psp->m_header.m_deviceId,(int) psp->m_data.uint8[0]);
    displayStr += buff;
  } break;
  case CPI_MainLoopTimeout: {
    if(psp->m_header.m_deviceId == m_targetDeviceId) {
      sprintf(buff,"%d ", (int) psp->m_data.uint32[0]);
      emit setMainLoopTimeout(buff);
    }
    sprintf(buff,"\n %d Main loop timeouts: %d ",psp->m_header.m_deviceId,psp->m_data.uint32[0]);
    displayStr += buff;
  } break;
  default:
    break;
  }

  if(psp->m_header.m_deviceId == m_targetDeviceId) {
    emit LocalProcessParam(*psp);
  }

  return ret;
}

void MainWindow::LocalProcessParam(PacketParam8ByteC psp)
{
  switch ((enum ComsParameterIndexT) psp.m_header.m_index)
  {
  case CPI_Indicator:
    ui->checkBoxIndicator->setChecked(psp.m_data.uint8[0] > 0);
    break;
  case CPI_EndStopEnable:
    ui->checkBoxEndStopEnable->setChecked(psp.m_data.uint8[0] > 0);
    break;
  case CPI_EndStopStart: {
    float angleDeg = (psp.m_data.float32[0] * 360.0f / (M_PI * 2.0));
    ui->doubleSpinBoxEndStopStart->setValue(angleDeg);
  } break;
  case CPI_EndStopEnd: {
    float angleDeg = (psp.m_data.float32[0] * 360.0f / (M_PI * 2.0));
    ui->doubleSpinBoxEndStopEnd->setValue(angleDeg);
  } break;
  case CPI_EndStopLimitBreakForce:
    ui->doubleSpinBoxEndStopForce->setValue(psp.m_data.float32[0]);
    break;
  case CPI_JointInertia:
    ui->doubleSpinBoxJointInertia->setValue(psp.m_data.float32[0]);
    break;

  default:
    break;
  }
}

void MainWindow::SetupComs()
{
  auto logger = spdlog::stdout_logger_mt("console");

  logger->info("Starting bark");
  m_coms = std::make_shared<DogBotN::ComsProxyC>();
  m_coms->SetLogger(logger);
  m_dogBotAPI = std::make_shared<DogBotN::DogBotAPIC>(m_coms,logger,false,DogBotN::DogBotAPIC::DMM_ClientOnly);

  m_coms->SetHandler(CPT_PWMState,[this](const uint8_t *data,int size) mutable
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

  m_coms->SetHandler(CPT_Pong,[this](const uint8_t *data,int size) mutable
  {
    const struct PacketPingPongC *psp = (const struct PacketPingPongC *) data;
    if(size != sizeof(struct PacketPingPongC)) {
      std::cerr << "Packet PingPong length was " << size << " expected " << sizeof(struct PacketPingPongC) << std::endl;
      emit setLogText("Unexpected pong packet size.");
      return ;
    }
    std::string displayStr = "Got pong ";
    displayStr += std::to_string((int) psp->m_deviceId);
    std::cout << displayStr << std::endl;
    emit setLogText(displayStr.c_str());
  });

  m_coms->SetHandler(CPT_SetParam,[this](const uint8_t *data,int size) mutable
  {
    printf("Got SetParam.  Size:%d \n",size);
    if(size < (int) sizeof(struct PacketParamHeaderC)) {
      std::cerr << "Packet ReportParam length too small " << size << " expected at least" << sizeof(struct PacketParamHeaderC) << std::endl;
      emit setLogText("Unexpected ReportParam packet size.");
      return ;
    }
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

  m_coms->SetHandler(CPT_ReportParam,[this](const uint8_t *data,int size) mutable
  {
//    printf("Got ReportParam.  Size:%d \n",size);
    if(size < (int) sizeof(struct PacketParamHeaderC)) {
      std::cerr << "Packet ReportParam length too small " << size << " expected at least" << sizeof(struct PacketParamHeaderC) << std::endl;
      emit setLogText("Unexpected ReportParam packet size.");
      return ;
    }
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

  m_coms->SetHandler(CPT_AnnounceId,[this](const uint8_t *data,int size) mutable
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

  m_coms->SetHandler(CPT_Servo,[this](const uint8_t *data,int size) mutable
  {
    if(size != sizeof(struct PacketServoC)) {
      emit setLogText("Unexpected packet length.");
      return;
    }
    const PacketServoC *pkt = (const PacketServoC *) data;
    std::cout << "Servo " << (int) pkt->m_deviceId << " Position:" << pkt->m_position << " Torque: " << pkt->m_torqueLimit << " State:" << pkt->m_mode << std::endl;
  });


  m_coms->SetHandler(CPT_ServoReport,[this](const uint8_t *data,int size) mutable
  {
    if(size != sizeof(struct PacketServoReportC)) {
      emit setLogText("Unexpected packet length.");
      return;
    }
    const PacketServoReportC *pkt = (const PacketServoReportC *) data;
    if(pkt->m_deviceId == m_targetDeviceId) {
      m_servoAngle = m_coms->PositionReport2Angle(pkt->m_position);
      m_servoTorque = m_coms->TorqueReport2Current(pkt->m_torque);
      m_servoRef = (enum PositionReferenceT) (pkt->m_mode & 0x3);

      if(m_inDeviceChange) {
        m_inDeviceChange = false;
        emit updatePosition(m_servoAngle);
      }
    }

    //std::cout << "ServoReport " << (int) pkt->m_deviceId << ((pkt->m_mode & 1) ? " Abs " : " Rel ") << " Position:" << pkt->m_position << " Torque: " << pkt->m_torque  << " State:" << (int) pkt->m_mode << std::endl;
  });

  m_coms->SetHandler(CPT_Error,[this](const uint8_t *data,int size) mutable
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
  m_toQuery = 0;

  // Update name
  std::shared_ptr<DogBotN::ServoC> servo = m_dogBotAPI->GetServoById(m_targetDeviceId);
  if(servo)
    ui->comboBoxServoName->setCurrentText(servo->Name().c_str());
}

//! Close current connection
void MainWindow::CloseConnection()
{
  m_coms->Close();
  ui->pushButtonConnect->setText("Connect");
  ui->tabWidget->setTabEnabled(1,false);
  ui->tabWidget->setTabEnabled(2,false);
  ui->tabWidget->setTabEnabled(3,false);
  ui->labelConnectionState->setText("Disconnected");
  ui->tabWidget->setCurrentIndex(0);
  ui->tabWidget->tabBar()->hide();
  return ;
}

void MainWindow::on_pushButtonConnect_clicked()
{
  if(m_coms->IsReady()) {
    std::cerr << "Closing. " << std::endl;
    CloseConnection();
    return ;
  }

  if(m_coms->Open(ui->comboBoxConnect->currentText().toStdString().c_str())) {
    std::cerr << "Connect ok. " << std::endl;
    ui->labelConnectionState->setText("Connection Ok");
    ui->pushButtonConnect->setText("Disconnect");
    emit setLogText("Connect ok");
    ui->tabWidget->setTabEnabled(1,true);
    ui->tabWidget->setTabEnabled(2,true);
    ui->tabWidget->setTabEnabled(3,true);
    ui->tabWidget->tabBar()->show();
    ui->tabWidget->setCurrentIndex(1);
    QueryAll();
  } else {
    std::cerr << "Failed to connect. " << std::endl;
    ui->labelConnectionState->setText("Failed");
    emit setLogText("Connect failed");
    ui->pushButtonConnect->setText("Connect");
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
  enum PWMControlDynamicT controlMode = CM_Final;
  if(arg1 == "Off") {
    controlMode = CM_Off;
  }
  if(arg1 == "Brake") {
    controlMode = CM_Brake;
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

void MainWindow::on_updatePosition(double angle)
{
  int index = angle * 360 / (2.0 * 3.14159265359);
  ui->sliderPosition->setSliderPosition(index);
  ui->doubleSpinBoxDemandPosition->setValue(index);
}

void MainWindow::on_sliderPosition_sliderMoved(int position)
{
  m_position = position * 2.0 * 3.14159265359/ 360.0;
  //std::cerr << "Mode: " << (int) m_controlMode << std::endl;
  // Convert position to radians
  switch(m_controlMode)
  {
  case CM_Position:
  {
    //std::cerr << "Sending pos: " << std::endl;
    m_position = position * 2.0 * 3.14159265359/ 360.0;
    //std::cout << "Sending move. Pos: " << m_position << " Torque:" << m_torque << " Ref:" << (int) g_positionReference << std::endl << std::flush;
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
  switch(m_controlMode)
  {
  case CM_Position:
    m_coms->SendMoveWithEffort(m_targetDeviceId,m_position,m_torque,g_positionReference);
  default:
    break;
  }
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
  m_inDeviceChange = true;
  QueryAll();
}

void MainWindow::on_comboBoxCalibration_activated(const QString &arg1)
{
  enum MotionHomedStateT calMode = MHS_Lost;
  if(arg1 == "Lost") {
    calMode = MHS_Lost;
  }
  if(arg1 == "Measuring") {
    calMode = MHS_Measuring;
  }
  if(arg1 == "Homed") {
    calMode = MHS_Homed;
  }
  m_coms->SendSetParam(m_targetDeviceId,CPI_HomedState,calMode);
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
  if(arg1 == "Motion Calibrate") {
    controlState = CS_MotionCalibrate;
  }
  if(arg1 == "Low Power") {
    controlState = CS_LowPower;
  }
  if(arg1 == "Ready") {
    controlState = CS_Ready;
  }
  if(arg1 == "Auto Home") {
    controlState = CS_Home;
  }
  if(arg1 == "Teach") {
    controlState = CS_Teach;
  }
  if(arg1 == "Reset") {
    controlState = CS_StartUp;
  }
  if(arg1 == "Diagnostic") {
    controlState = CS_Diagnostic;
  }
  if(controlState != CS_Fault)  {
    m_coms->SendSetParam(m_targetDeviceId,CPI_ControlState,controlState);
  }
}

void MainWindow::on_checkBoxIndicator_toggled(bool checked)
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_Indicator,checked);
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
  std::cout << "Changing requested ref to " << (int) positionReference << std::endl  << std::flush;
  m_coms->SendSetParam(m_targetDeviceId,CPI_PositionRef,(uint8_t) positionReference);
}

void MainWindow::on_pushButton_2_clicked()
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_DebugIndex,(uint8_t) 7);
  //m_coms->SendSetParam(m_targetDeviceId,CPI_CANBridgeMode,1);
  //m_coms->SendQueryParam(m_targetDeviceId,CPI_CANBridgeMode);
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
  //std::cout << "DemandPosition: " << arg1 << std::endl;
}

void MainWindow::on_doubleSpinBoxCalibrationOffset_editingFinished()
{
  float calAngleRad =(ui->doubleSpinBoxCalibrationOffset->value() * (M_PI * 2.0) / 360.0f);
  m_coms->SendSetParam(m_targetDeviceId,CPI_CalibrationOffset,calAngleRad);

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

void MainWindow::on_pushButton_5_clicked()
{
  m_coms->SendQueryParam(m_targetDeviceId,CPI_5VRail);
}

void MainWindow::on_pushButtonEmergencyStop_clicked()
{
  m_coms->SendEmergencyStop();
}

void MainWindow::on_doubleSpinBoxHomeIndexOffset_valueChanged(double arg1)
{
  float homeAngleRad = (arg1 * (M_PI * 2.0) / 360.0f);
  m_coms->SendSetParam(m_targetDeviceId,CPI_homeIndexPosition,homeAngleRad);
}

void MainWindow::on_pushButtonQueryHomed_clicked()
{
  m_coms->SendQueryParam(m_targetDeviceId,CPI_homeIndexPosition);
}

void MainWindow::on_pushButtonBrake_clicked()
{
  m_coms->SendSetParam(0,CPI_PWMMode,CM_Brake);
}

void MainWindow::on_pushButtonOff_clicked()
{
  m_coms->SendSetParam(0,CPI_PWMMode,CM_Off);
}

void MainWindow::on_pushButtonHold_clicked()
{
  // Go through and set demand to current position.
  m_dogBotAPI->DemandHoldPosition();
}

void MainWindow::on_comboBoxServoName_activated(const QString &arg1)
{
  std::shared_ptr<DogBotN::ServoC> servo = m_dogBotAPI->GetServoById(m_targetDeviceId);
  if(servo)
    servo->SetName(arg1.toLatin1().data());
}

void MainWindow::on_actionSaveConfig_triggered()
{
  if(m_configFilename.empty()) {
    QString fileName = QFileDialog::getSaveFileName(this,
            tr("Save Config File"), "",
            tr("Setup (*.json);;All Files (*)"));
    if(fileName.isEmpty())
      return ;
    m_configFilename = fileName.toStdString();
  }

  if(!m_dogBotAPI->SaveConfig(m_configFilename)) {
    QMessageBox::information(this, "Save Config", "Failed to save file " + QString(m_configFilename.c_str()));
  }
}

void MainWindow::on_actionLoadConfig_triggered()
{
  QString fileName = QFileDialog::getOpenFileName(this,
          tr("Load Config File"), "",
          tr("Setup (*.json);;All Files (*)"));

  if(fileName.isEmpty())
    return ;
  m_configFilename = fileName.toStdString();
  if(!m_dogBotAPI->LoadConfig(m_configFilename)) {
    QMessageBox::information(this, "Load Config", "Failed to load file " + fileName);
    return ;
  }
}

void MainWindow::on_actionSave_Config_As_triggered()
{
  QString fileName = QFileDialog::getSaveFileName(this,
          tr("Save Config File"), "",
          tr("Setup (*.json);;All Files (*)"));
  if(fileName.isEmpty())
    return;
  m_configFilename = fileName.toStdString();
  if(!m_dogBotAPI->SaveConfig(m_configFilename)) {
    QMessageBox::information(this, "Save Config", "Failed to save file " + QString(m_configFilename.c_str()));
  }

}

void MainWindow::on_pushButtonResetAll_clicked()
{
  m_dogBotAPI->ResetAll();
}

void MainWindow::on_pushButtonRefresh_clicked()
{
  m_dogBotAPI->RefreshAll();
}

void MainWindow::on_actionExit_triggered()
{
  QApplication::quit();
}


void MainWindow::on_comboBoxFanState_currentIndexChanged(const QString &arg1)
{

}

void MainWindow::on_comboBoxFanState_activated(int index)
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_FanMode,index);
}

void MainWindow::on_checkBoxJointRelative_stateChanged(int arg1)
{
  enum JointRelativeT jointRelativeMode;
  if(arg1 == 0) {
    jointRelativeMode = JR_Isolated;
  } else {
    jointRelativeMode = JR_Offset;
  }
  m_coms->SendSetParam(m_targetDeviceId,CPI_JointRelative,jointRelativeMode);
}

void MainWindow::on_lineEditFanTempThreshold_editingFinished()
{
  float value = atof(ui->lineEditFanTempThreshold->text().toLatin1().data());
  m_coms->SendSetParam(m_targetDeviceId,CPI_FanTemperatureThreshold,value);
}

void MainWindow::on_doubleSpinBoxEndStopStart_valueChanged(double arg1)
{
  float endStopAngleRad = (arg1 * (M_PI * 2.0) / 360.0f);
  m_coms->SendSetParam(m_targetDeviceId,CPI_EndStopStart,endStopAngleRad);
}

void MainWindow::on_doubleSpinBoxEndStopEnd_valueChanged(double arg1)
{
  float endStopAngleRad = (arg1 * (M_PI * 2.0) / 360.0f);
  m_coms->SendSetParam(m_targetDeviceId,CPI_EndStopEnd,endStopAngleRad);
}

void MainWindow::on_checkBoxEndStopEnable_toggled(bool checked)
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_EndStopEnable,checked);
}

void MainWindow::on_doubleSpinBoxJointInertia_valueChanged(double arg1)
{
  m_coms->SendSetParam(m_targetDeviceId,CPI_JointInertia,arg1);
}

//! Get the current position and set the given parameter to it.
void MainWindow::SetValueToCurrentPosition(ComsParameterIndexT param)
{
  if(m_targetDeviceId == 0) {
    std::cerr << "No device id\n";
    return ;
  }
  std::shared_ptr<DogBotN::ServoC> servo =  m_dogBotAPI->GetServoById(m_targetDeviceId);
  if(!servo) {
    std::cerr << "No servo\n";
    return ;
  }
  DogBotN::JointC::TimePointT tick;
  double position = 0;
  double velocity = 0;
  double torque = 0;
  if(!servo->GetState(tick,position,velocity,torque)) {
    std::cerr << "No position\n";
    return ;
  }
  m_coms->SendSetParam(m_targetDeviceId,param,position);

}

void MainWindow::on_pushButtonSetEndStopStart_clicked()
{
  SetValueToCurrentPosition(CPI_EndStopStart);
}

void MainWindow::on_pushButtonSetEndStopEnd_clicked()
{
  SetValueToCurrentPosition(CPI_EndStopEnd);
}

void MainWindow::on_pushButtonSaveConfig_clicked()
{
  if(m_targetDeviceId == 0)
    return ;
  m_coms->SendStoreConfig(m_targetDeviceId);
}
