#ifndef DOGBOG_ZMQCLIENTCOMS_HEADER
#define DOGBOG_ZMQCLIENTCOMS_HEADER 1

#include "dogbot/Coms.hh"
#include <zmq.hpp>

namespace DogBotN {


  //! Low level network communications

  class ComsZMQClientC
   : public ComsC
  {
  public:
    ComsZMQClientC(const char *portAddr);

    //! default
    ComsZMQClientC();

    //! Destructor
    // Disconnects and closes file descriptors
    virtual ~ComsZMQClientC();

    //! Open a port.
    virtual bool Open(const char *portAddr) override;

    //! Close connection
    virtual void Close() override;

    //! Is connection ready ?
    virtual bool IsReady() const override;

    //! Send packet
    virtual void SendPacket(const uint8_t *data,int len) override;


  protected:

    bool RunRecieve();

    std::shared_ptr<zmq::socket_t> m_client;
    std::shared_ptr<zmq::socket_t> m_sub;

    std::thread m_threadRecieve;

    std::mutex m_accessTx;
    std::timed_mutex m_mutexExitOk;

  };
}
#endif
