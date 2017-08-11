#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>

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

    sprintf(buff,"%5d  %4d %4d %4d  %4d %4d %4d  %6d     \r",
           0,//msg->m_tick,
           msg->m_hall[0],msg->m_hall[1],msg->m_hall[2],
           msg->m_curr[0],msg->m_curr[1],msg->m_curr[2],
           msg->m_angle);
    std::cout << buff;
    //emit setLogText(buff);
  });

  m_coms.SetHandler(CPT_Pong,[this](uint8_t *data,int size) mutable
  {
    printf("Got pong. \n");
    emit setLogText("Got pong");
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
  m_coms.SendSetParam(CPI_PWMState,1);
  emit setLogText("PWM clicked");

}

void MainWindow::on_pushButtonPWMReport_clicked()
{
  if(!m_PWMReportRequested) {
    ui->pushButtonPWMReport->setText("Stop PWM Report");
    m_PWMReportRequested = true;
    m_coms.SendSetParam(CPI_PWMFullReport,1);
  } else {
    ui->pushButtonPWMReport->setText("Start PWM Report");
    m_PWMReportRequested = false;
    m_coms.SendSetParam(CPI_PWMFullReport,0);
  }
}

void MainWindow::on_pushButtonPing_clicked()
{
  emit setLogText("Ping");
  m_coms.SendPing();
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
  m_coms.SendSetParam(CPI_PWMMode,controlMode);
}

void MainWindow::on_sliderPosition_sliderMoved(int position)
{
  m_position = position * 3.14159265359/ 360.0;
  m_coms.SendMoveWithEffort(m_position,m_torque);
}

void MainWindow::on_sliderTorque_sliderMoved(int torque)
{
  m_torque = torque / 10.0;
  m_coms.SendMoveWithEffort(m_position,m_torque);
}

void MainWindow::on_pushButtonStopPWM_clicked()
{
  m_coms.SendSetParam(CPI_PWMState,0);
  emit setLogText("PWM clicked");
}
