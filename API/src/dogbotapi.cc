
#include "../include/dogbot/DogBotAPI.hh"
#include <fstream>
#include <chrono>

namespace DogBotN {

  //! Constructor
  DogBotAPIC::DogBotAPIC(const std::string &configFile)
  {
    if(!configFile.empty())
      Init(configFile);
  }

  //! Set the logger to use
  void DogBotAPIC::SetLogger(std::shared_ptr<spdlog::logger> &log)
  {
    m_log = log;
    m_coms.SetLogger(log);
  }


  //! Shutdown controller.
  bool DogBotAPIC::Shutdown()
  {
    m_terminate = true;
    return false;
  }

  //! Load configuration for the robot.

  bool DogBotAPIC::Init(const std::string &configFile)
  {
    if(m_started) {
      m_log->error("Init already called. ");
      return false;
    }

    std::ifstream confStrm(configFile,std::ifstream::binary);

    if(!confStrm) {
      m_log->error("Failed to open configuration file '{}' ",configFile);
      return false;
    }

    confStrm >> m_configRoot;

    //! Initalise which serial device to use.
    m_deviceName = m_configRoot.get("device", "/dev/ttyACM0" ).asString();

    m_started = true;

    m_threadMonitor = std::move(std::thread { [this]{ RunMonitor(); } });


    return true;
  }

  //! Monitor thread
  void DogBotAPIC::RunMonitor()
  {
    m_log->debug("Running monitor. ");

    while(!m_terminate)
    {
      switch(m_driverState)
      {
        case DS_Init: {
          std::this_thread::sleep_for(std::chrono::seconds(1));
        } break;
        case DS_NoConnection: {
          if(!m_coms.Open(m_deviceName.c_str())) {
            m_log->warn("Failed to open coms channel. ");
            std::this_thread::sleep_for(std::chrono::seconds(5));
          } else {
            m_driverState = DS_Connected;
          }
        } break;
        case DS_Connected:
        case DS_Calibrated:
        case DS_Error:
          std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }

    m_log->debug("Exiting monitor. ");
  }




}
