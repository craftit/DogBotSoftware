
#include "../include/dogbot/ComsRecorder.hh"

#include "dogbot/DogBotAPI.hh"
#include "dogbot/ZMQContext.hh"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>

namespace DogBotN {


  ComsRecorderC::ComsRecorderC(
      const std::shared_ptr<ComsC> &coms,
      std::shared_ptr<spdlog::logger> &log,
      const std::string &filename
      )
   : m_coms(coms),
     m_log(log),
     m_filename(filename)
  {

  }


  //! Make sure everything is disconnected.
  ComsRecorderC::~ComsRecorderC()
  {
    Stop();
  }

  void ComsRecorderC::RecordPacket(bool isCommand,const uint8_t *data,int len)
  {
    uint8_t buffer[256];
    int at = 0;
    clock_gettime(CLOCK_REALTIME,(struct timespec*)&buffer[at]);
    at += sizeof(timespec);
    buffer[at++] = isCommand ? 1 : 0; // Flag field, indicates received packet.
    buffer[at++] = len;
    memcpy(&buffer[at],data,len);
    at += len;
    int done = 0;
    do {
      int n = write(m_fd,&buffer[done],at-done);
      if(n <= 0) {
        if(errno == EINTR || errno == EAGAIN)
          continue;
        m_log->error("Error occurred writing log data. Error:{} ",strerror(errno));
        return ;
      }
      if(n != at && n != 0) {
        m_log->warn("Non-atomic recorder write.");
      }
      done += n;
    } while(done < at) ;
  }


  //! Start recording
  bool ComsRecorderC::Start()
  {
    assert(m_fd < 0);
    m_fd = open(m_filename.c_str(),O_WRONLY | O_CREAT | O_TRUNC,0777);
    if(m_fd < 0) {
      m_log->error("Failed to open log file. ");
      return false;
    }
    m_genericHandlerId = m_coms->SetGenericHandler([this](const uint8_t *data,int len) mutable
                            {
                              RecordPacket(false,data,len);
                            }
    );


    m_sendHandler = m_coms->SetCommandHandler([this](const uint8_t *data,int len) mutable
                            {
                              RecordPacket(true,data,len);
                            }
    );
    return true;
  }


  //! Stop recording.
  bool ComsRecorderC::Stop()
  {
    m_genericHandlerId.Remove();
    m_sendHandler.Remove();
    if(m_fd >= 0) {
      close(m_fd);
      m_fd = -1;
    } else
      return false;
    return true;
  }


}
