
#include <iostream>
#include <unistd.h>

#include "dogbot/ComsSerial.hh"
#include "dogbot/ComsUSB.hh"
#include "dogbot/DogBotAPI.hh"

int main(int nargs,char **argv)
{
  auto logger = spdlog::stdout_logger_mt("console");

  logger->info("Starting dtalk");


  std::shared_ptr<DogBotN::ComsC> coms = std::make_shared<DogBotN::ComsUSBC>();


  //std::string devFilename = "/dev/tty.usbmodem401";
#if 0
  std::string devFilename = "/dev/ttyACM1";
  if(nargs > 1)
    devFilename = argv[1];

  std::shared_ptr<DogBotN::ComsSerialC> coms = std::make_shared<DogBotN::ComsSerialC>();

  coms->SetLogger(logger);

  if(!coms->Open(devFilename.c_str())) {
    logger->error("Failed to open serial port.");
    // Reason will already be logged.
    return 1;
  }

  DogBotN::DogBotAPIC dogBotAPI(coms,logger);
  DogBotN::MotorCalibrationC motorCal;
  if(!dogBotAPI.ReadCalibration(0,motorCal)) {
    logger->error("Failed to read calibration.");
    return 1;
  }
  Json::Value calInfo = motorCal.AsJSON();
  std::cout << calInfo << std::endl;
#endif

#if 0
  coms.SetHandler(CPT_PWMState,[logger](uint8_t *data,int size) mutable
  {
    PacketPWMStateC *msg = (PacketPWMStateC *) data;
    logger->info("%5d  %4d %4d %4d  %4d %4d %4d  %6d     \n",
           msg->m_tick,
           msg->m_hall[0],msg->m_hall[1],msg->m_hall[2],
           msg->m_curr[0],msg->m_curr[1],msg->m_curr[2],
           msg->m_angle);


#if 0
    int16_t pos = (unsigned) data[1] + ((unsigned) data[2] << 8);
    int16_t effort = (unsigned) data[3] + ((unsigned) data[4] << 8);
    int32_t velocity = (unsigned) data[5] + ((unsigned) data[6] << 8) + ((unsigned) data[7] << 16) + ((unsigned) data[8] << 24);
    bool atGoal = data[9];
    bool stalled = data[10];

    ROS_INFO("Pos:%d Effort:%d Velocity:%d  Goal:%d Stalled:%d ",pos,effort,(int) velocity,atGoal,stalled);

    //control_msgs::GripperCommandFeedback msg;

    msg.position = 1.0 - ((pos * 3.14159265359)/ (2 * 1024.0));
    if(msg.position < 0) msg.position = 0;
    if(msg.position > 1.0) msg.position = 1.0;
    msg.effort = (float) (effort / 1024.0) - 0.5;
    msg.stalled = stalled;
    msg.reached_goal = atGoal;

    as->publishFeedback(msg);

    // Are we finished with action?
    if(stalled || atGoal)
      done.unlock();
#endif
  });

#endif
  logger->info("Setup and ready. ");
  while(1) {
    sleep(1);
    logger->info("Sending ping. ");
    coms->SendPing(0);
    coms->SendSetParam(0,CPI_Indicator,1);
    sleep(1);
    coms->SendSetParam(0,CPI_Indicator,0);
  }

  return 0;
}
