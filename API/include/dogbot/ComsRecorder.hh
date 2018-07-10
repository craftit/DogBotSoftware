#ifndef DOGBOG_ComsRecorder_HEADER
#define DOGBOG_ComsRecorder_HEADER 1

#include "dogbot/Coms.hh"

namespace DogBotN {

  //! Utility class for recording a log of commands sent and data received.

  class ComsRecorderC
  {
  public:
    ComsRecorderC(
        const std::shared_ptr<ComsC> &coms,
        std::shared_ptr<spdlog::logger> &log,
        const std::string &filename
        );

    //! Make sure everything is disconnected.
    ~ComsRecorderC();

    //! Start recording.
    //! This will truncate the log file and start a new recording.
    bool Start();

    //! Stop recording.
    bool Stop();

    //! Set verbose mode for dumping messages
    void SetVerbose(bool verbose)
    { m_verbose = verbose; }

    //! Run recorder
    void Run(const std::string &addr);

  protected:

    void RecordPacket(bool isCommand,const uint8_t *data,int len);

    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");

    std::shared_ptr<ComsC> m_coms;

    std::string m_filename;
    int m_fd = -1;

    bool m_verbose = false;
    bool m_terminate = false;
    CallbackHandleC m_genericHandlerId;
    CallbackHandleC m_sendHandler;
  };


  //! Class for accessing coms log files.

  class ComsPlaybackC
  {

  };

}

#endif
