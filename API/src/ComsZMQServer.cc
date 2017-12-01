
#include "dogbot/ComsZMQServer.hh"
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
    m_server->connect ("tcp://127.0.0.1:5555");

    // Publish state messages
    m_pub = std::make_shared<zmq::socket_t>(g_zmqContext,ZMQ_PUB);
    m_pub->connect ("tcp://127.0.0.1:5556");

    m_coms->SetGenericHandler([this](uint8_t *data,int len) mutable
                              {
                                zmq::message_t msg (len);
                                memcpy (msg.data (), data, len);
                                m_pub->send(msg);
                              }
    );

    while(!m_terminate) {
      zmq::message_t msg;
      if(!m_server->recv(&msg,0))
        continue;
      m_coms->SendPacket((uint8_t *) msg.data(),msg.size());
    }

    // Remove handler with reference to this instance.
    m_coms->SetGenericHandler([](uint8_t *data,int len){});

  }

}
