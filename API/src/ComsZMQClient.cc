
#include "../include/dogbot/ComsZMQClient.hh"
#include "dogbot/ZMQContext.hh"

namespace DogBotN
{

  ComsZMQClientC::ComsZMQClientC(const std::string &portAddr)
  {
    Open(portAddr);
  }

  //! default
  ComsZMQClientC::ComsZMQClientC()
  {}

  // Disconnects and closes file descriptors
  ComsZMQClientC::~ComsZMQClientC()
  {
    Close();
  }

  //! Close connection
  void ComsZMQClientC::Close()
  {
    m_log->debug("Close called.");

    ComsC::Close();

    if(!m_mutexExitOk.try_lock_for(std::chrono::milliseconds(500))) {
      m_log->error("Failed to shutdown receiver thread.");
    }
    m_threadRecieve.join();
  }

  //! Is connection ready ?
  bool ComsZMQClientC::IsReady() const
  {
    return true;
  }


  bool ComsZMQClientC::Open(const std::string &portAddr)
  {
    m_terminate = false;


    if(m_terminate)
      return false;

    if(!m_mutexExitOk.try_lock()) {
      m_log->error("Exit lock already locked, multiple threads attempting to open coms ?");
      return false;
    }

    m_client = std::make_shared<zmq::socket_t>(g_zmqContext,ZMQ_PUSH);
    m_client->connect ("tcp://127.0.0.1:7200");

    m_threadRecieve = std::move(std::thread { [this]{ RunRecieve(); } });
    return true;
  }


  //! Process received packet.

  bool ComsZMQClientC::RunRecieve()
  {
    m_log->debug("Running receiver. ");

    m_sub = std::make_shared<zmq::socket_t>(g_zmqContext,ZMQ_SUB);
    m_sub->connect ("tcp://127.0.0.1:7201");
    m_sub->setsockopt(ZMQ_SUBSCRIBE,0,0);

    while(!m_terminate) {
      //bool socket_t::recv(message_t *msg, int flags = 0);
      zmq::message_t msg;
      if(!m_sub->recv(&msg,0))
        continue;
      m_log->info("Client got msg.");
      ProcessPacket((uint8_t*)msg.data(),msg.size());
    }

    m_mutexExitOk.unlock();
    m_log->debug("Exiting receiver. ");

    return true;
  }

  //! Send packet
  void ComsZMQClientC::SendPacket(const uint8_t *buff,int len)
  {
    std::lock_guard<std::mutex> lock(m_accessTx);
    zmq::message_t msg (len);
    memcpy (msg.data (), buff, len);
    m_client->send (msg);
  }

}
