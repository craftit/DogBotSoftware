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
    ComsZMQClientC(const std::string &portAddr);

    //! default
    ComsZMQClientC();

    //! Destructor
    // Disconnects and closes file descriptors
    virtual ~ComsZMQClientC();

    //! Open a port.
    virtual bool Open(const std::string &portAddr) override;

    //! Close connection
    virtual void Close() override;

    //! Is connection ready ?
    virtual bool IsReady() const override;

    //! Send packet
    virtual void SendPacketWire(const uint8_t *data,int len) override;


  protected:

    bool RunRecieve();

    std::string m_rootAddress;

    std::shared_ptr<zmq::socket_t> m_client;

    std::thread m_threadRecieve;

    std::mutex m_accessTx;
    std::timed_mutex m_mutexExitOk;

  };
}
#endif
