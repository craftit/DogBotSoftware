
#include "dogbot/ComsZMQServer.hh"
#include "dogbot/DogBotAPI.hh"
#include "dogbot/ZMQContext.hh"

namespace DogBotN {


  ComsZMQServerC::ComsZMQServerC(const std::shared_ptr<ComsC> &coms,std::shared_ptr<spdlog::logger> &log)
   : m_coms(coms),
     m_log(log)
  {
  }


  //! Run server
  void ComsZMQServerC::Run(const std::string &addr)
  {

    m_server = std::make_shared<zmq::socket_t>(g_zmqContext,ZMQ_PULL);
    m_server->bind ("tcp://*:7200");

    // Publish state messages
    m_pub = std::make_shared<zmq::socket_t>(g_zmqContext,ZMQ_PUB);
    m_pub->bind ("tcp://*:7201");

    m_coms->SetGenericHandler([this](uint8_t *data,int len) mutable
                              {
                                m_log->info("Server got dev msg. {} {} ",ComsPacketTypeToString((ComsPacketTypeT)data[0]),len);
                                zmq::message_t msg (len);
                                memcpy (msg.data (), data, len);
                                m_pub->send(msg);
                              }
    );

    m_log->info("Server run loop started.");
    while(!m_terminate) {
      zmq::message_t msg;
      if(!m_server->recv(&msg,0)) {
        m_log->info("Server no msg.");
        continue;
      }
      m_log->info("Server got msg.");
      m_coms->SendPacket((uint8_t *) msg.data(),msg.size());
    }

    m_log->info("Server run loop exiting.");

    // Remove handler with reference to this instance.
    m_coms->SetGenericHandler([](uint8_t *data,int len){});

  }

}
