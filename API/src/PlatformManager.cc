
#include "dogbot/PlatformManager.hh"
#include "dogbot/DogBotController.hh"
#include "dogbot/Strings.hh"
#include <chrono>
#include <thread>

#define DODEBUG 0
#if DODEBUG
#define ONDEBUG(x) x
#else
#define ONDEBUG(x)
#endif

namespace DogBotN {

  PlatformManagerC::PlatformManagerC(std::shared_ptr<DogBotAPIC> &api)
   : m_dogBotAPI(api)
  {
    SetDeviceType(DT_PlatformManager);
  }

  //! Start platform running.
  void PlatformManagerC::Init()
  {
    if(m_isRunning)
      return;
    std::lock_guard<std::mutex> lock(m_accessParam);
    if(m_isRunning)
      return;
    m_isRunning = true;
    m_threadRun = std::move(std::thread { [this]{ Run(); } });
  }

  //! Handle incoming packet to device
  void PlatformManagerC::VirtProcessPacket(const uint8_t *data,int len)
  {
    enum ComsPacketTypeT pktType = (enum ComsPacketTypeT) data[0];
    switch(pktType)
    {
      //case CPT_:
      default:
        ComsVirtualDeviceC::VirtProcessPacket(data,len);
        break;
    }
  }


  //! Set a parameter
  bool PlatformManagerC::SetParam(enum ComsParameterIndexT index,union BufferTypeT *data,int len)
  {
    switch(index)
    {
      case CPI_RequestedPlatformActivity:
      case CPI_PlatformActivity: {
        //m_log->info("Got SetPlatformActivity.");
        if(len != 5)
          return false;
        int32_t newActivityKey = data->uint32[0];
        enum PlatformActivityT newActivity = (enum PlatformActivityT) data->uint8[4];

        std::lock_guard<std::mutex> lock(m_accessParam);
        if(newActivityKey == 0 && newActivity == PA_Idle) {
          m_requestedActivityKey = 0;
          m_requestedActivity = PA_Idle;
          m_activityPending = true;
        } else if(m_activityKey == 0 || newActivityKey == m_activityKey) {
          m_log->info("Requesting activity {} .",DogBotN::ComsPlatformActivityToString(newActivity));
          m_requestedActivityKey = newActivityKey;
          m_requestedActivity = newActivity;
          m_activityPending = true;
        }
      } break;
      default:
        return ComsVirtualDeviceC::SetParam(index,data,len);
    }
    return true;
  }


  //! Read a parameter
  bool PlatformManagerC::ReadParam(enum ComsParameterIndexT index,int *len,union BufferTypeT *data)
  {
    switch(index)
    {
      case CPI_PlatformActivity: {
        *len = 5;
        std::lock_guard<std::mutex> lock(m_accessParam);
        data->uint32[0] = m_activityKey;
        data->uint8[4] = m_currentActivity;
      } break;
      case CPI_RequestedPlatformActivity: {
        *len = 5;
        std::lock_guard<std::mutex> lock(m_accessParam);
        data->uint32[0] = m_requestedActivityKey;
        data->uint8[4] = m_requestedActivity;
      } break;
      default:
        return ComsVirtualDeviceC::ReadParam(index,len,data);
    }
    return true;
  }

  void PlatformManagerC::FinishedActivity()
  {
    {
      std::lock_guard<std::mutex> lock(m_accessParam);
      m_log->info("Finished activity {}.",DogBotN::ComsPlatformActivityToString(m_currentActivity));
      m_activityKey = 0;
      m_currentActivity = PA_Idle;
    }
    ReadParamAndReply(CPI_PlatformActivity);
  }

  //! Run walk animation
  void PlatformManagerC::RunWalk()
  {
    m_speedLimit = 1000;
    m_dogBotAPI->Connection()->SendSetParam(0,CPI_VelocityLimit,m_speedLimit);

    DogBotN::DogBotControllerC controller(m_dogBotAPI);
    DogBotN::DogBotKinematicsC &kinematics = m_dogBotAPI->DogBotKinematics();

    float updatePeriod = 0.01;
    float torque = 4;

    controller.SetupTrajectory(updatePeriod,torque);

    enum PlatformActivityT currentActivity = PA_Idle;
    int32_t activityKey = 0;

    {
      std::lock_guard<std::mutex> lock(m_accessParam);
      activityKey = m_activityKey;
      currentActivity = m_currentActivity;
    }


    auto nextTime = std::chrono::steady_clock::now();
    DogBotN::SimpleQuadrupedPoseC pose;
    DogBotN::PoseAnglesC poseAngles;

    // We don't bother lock here, as we're only interested if the values have
    // been changed.
    while(activityKey == m_activityKey && m_currentActivity == m_requestedActivity)
    {

      kinematics.Pose2Angles(pose,poseAngles);

      nextTime += std::chrono::microseconds((int)(updatePeriod * 1000000.0f));
      std::this_thread::sleep_until(nextTime);
      controller.NextTrajectory(poseAngles);
    }
  }

  //! Run activity
  void PlatformManagerC::Run()
  {
    ONDEBUG(m_log->info("Starting platform manager thread."));
    while(m_isRunning) {
      // Are we being asked to abort the current activity ?
      // We probably need to be a bit more careful than this.
      {
        std::unique_lock<std::mutex> lock(m_accessParam);
        if(m_requestedActivity != m_currentActivity && m_activityPending) {
          if((m_requestedActivity == PA_Idle && m_requestedActivityKey == 0) ||
              m_activityKey == m_requestedActivityKey)
          {
            m_log->info("Aborting current activity.");
            m_activityKey = 0;
            m_currentActivity = PA_Idle;
            m_activityPending = false;
            lock.unlock();
            ReadParamAndReply(CPI_PlatformActivity);
          }
        }
      }
      if(m_currentActivity != PA_Idle)
        m_log->info("Current activity {} .",DogBotN::ComsPlatformActivityToString(m_currentActivity));
      switch(m_currentActivity)
      {
        case PA_Home:
          m_dogBotAPI->HomeAll();
          FinishedActivity();
          break;
        case PA_Idle: {
          std::unique_lock<std::mutex> lock(m_accessParam);
          if(m_requestedActivity != m_currentActivity && m_activityPending) {
            m_log->info("Starting activity {}.",DogBotN::ComsPlatformActivityToString(m_requestedActivity));
            m_activityKey = m_requestedActivityKey;
            m_currentActivity = m_requestedActivity;
            m_requestedActivityKey = 0;
            m_activityPending = false;
            lock.unlock();
            ReadParamAndReply(CPI_PlatformActivity); // Let everyone know what we're doing.
            break;
          }
          lock.unlock();
          std::this_thread::sleep_for(std::chrono::microseconds(100));
        } break;
        case PA_Walking:
          RunWalk();
          FinishedActivity();
          break;
        case PA_Passive:
          std::this_thread::sleep_for(std::chrono::seconds(1));
          break;
        case PA_Bootloader:
        case PA_Falling:
        case PA_Stand:
        case PA_Shutdown:
        case PA_User:
        default:
          std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
    ONDEBUG(m_log->info("Exiting platform manager thread."));
  }


}
