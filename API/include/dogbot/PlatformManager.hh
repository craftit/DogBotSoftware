#ifndef DOGBOG_PLATFORMMANAGER_HEADER
#define DOGBOG_PLATFORMMANAGER_HEADER 1

#include "dogbot/ComsVirtualDevice.hh"
#include "dogbot/DogBotAPI.hh"

namespace DogBotN {

  //! Platform manager

  class PlatformManagerC
   : public ComsVirtualDeviceC
  {
  public:
    //! Construct a manger.
    PlatformManagerC(std::shared_ptr<DogBotAPIC> &api);

    //! Start platform running.
    void Init();

    //! Handle incoming packet to device
    virtual void VirtProcessPacket(const uint8_t *data,int len) override;

    //! Set a parameter
    virtual bool SetParam(enum ComsParameterIndexT index,union BufferTypeT *data,int len) override;

    //! Read a parameter
    virtual bool ReadParam(enum ComsParameterIndexT index,int *len,union BufferTypeT *data) override;

  protected:
    //! Run activities
    void Run();

    //! Flag current activity as finished.
    void FinishedActivity();

    //! Run walk animation
    void RunWalk();

    std::shared_ptr<DogBotAPIC> m_dogBotAPI;
    enum PlatformActivityT m_currentActivity = PA_Idle;
    enum PlatformActivityT m_requestedActivity = PA_Idle;
    bool m_activityPending = false;
    int32_t m_activityKey = 0;
    int32_t m_requestedActivityKey = 0;
    std::mutex m_accessParam;
    float m_speedLimit = 500;
    bool m_isRunning = false;
    std::thread m_threadRun;
  };
}
#endif

