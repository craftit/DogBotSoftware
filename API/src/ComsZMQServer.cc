
#include "dogbot/ComsZMQServer.hh"
#include "dogbot/DogBotAPI.hh"
#include "dogbot/ZMQContext.hh"

namespace DogBotN {


  ComsZMQServerC::ComsZMQServerC(const std::shared_ptr<ComsC> &coms,std::shared_ptr<spdlog::logger> &log)
   : m_coms(coms),
     m_log(log)
  {
  }


  //! Make sure everything is disconnected.
  ComsZMQServerC::~ComsZMQServerC()
  {
    if(m_coms && m_genericHandlerId >= 0) {
      m_coms->RemoveGenericHandler(m_genericHandlerId);
      m_genericHandlerId = -1;
    }

  }

  //! Run server
  void ComsZMQServerC::Run(const std::string &addr)
  {

    m_server = std::make_shared<zmq::socket_t>(g_zmqContext,ZMQ_PULL);
    m_server->bind ("tcp://*:7200");

    // Publish state messages
    m_pub = std::make_shared<zmq::socket_t>(g_zmqContext,ZMQ_PUB);
    m_pub->bind ("tcp://*:7201");

    m_genericHandlerId = m_coms->SetGenericHandler([this](uint8_t *data,int len) mutable
                              {
                                ComsPacketTypeT cpt = (ComsPacketTypeT)data[0];
                                switch(cpt)
                                {
                                  case CPT_ReportParam: {
                                    PacketParam8ByteC *pkt = (PacketParam8ByteC *) data;
                                    std::string contents;
                                    for(int i = 0;i < len - sizeof(pkt->m_header);i++) {
                                      contents += " ";
                                      contents += std::to_string((int) pkt->m_data.uint8[i]);
                                    }
                                    m_log->info("Server got dev {} ReportParam {} : {} ",(int)pkt->m_header.m_deviceId,(int) pkt->m_header.m_index,contents);
                                  } break;
                                  default:
                                    m_log->info("Server got dev msg. {} {} ",ComsPacketTypeToString(cpt),len);
                                  break;
                                }
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

    // Remove handler with reference to this instance.
    m_coms->RemoveGenericHandler(m_genericHandlerId);
    m_genericHandlerId = -1;

    m_log->info("Server run loop exiting.");
  }

}
