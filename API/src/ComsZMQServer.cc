
#include "dogbot/ComsZMQServer.hh"
#include "dogbot/DogBotAPI.hh"
#include "dogbot/ZMQContext.hh"
#include "dogbot/Strings.hh"

#define DODEBUG 0
#if DODEBUG
#define ONDEBUG(x) x
#else
#define ONDEBUG(x)
#endif

namespace DogBotN {


  ComsZMQServerC::ComsZMQServerC(const std::shared_ptr<ComsC> &coms,std::shared_ptr<spdlog::logger> &log)
   : m_coms(coms),
     m_log(log)
  {
  }


  //! Make sure everything is disconnected.
  ComsZMQServerC::~ComsZMQServerC()
  {
    m_genericHandlerId.Remove();
  }

  //! Run server
  void ComsZMQServerC::Run(const std::string &addr)
  {

    std::shared_ptr<zmq::socket_t>  zserver = std::make_shared<zmq::socket_t>(g_zmqContext,ZMQ_PULL);
    std::string addrServer = addr + ":7200";
    zserver->bind (addrServer.c_str());
    zserver->setsockopt(ZMQ_RCVTIMEO,500);

    // Publish state messages
    std::shared_ptr<zmq::socket_t> zpub = std::make_shared<zmq::socket_t>(g_zmqContext,ZMQ_PUB);
    std::string addrPub = addr + ":7201";
    zpub->bind (addrPub.c_str());
    zpub->setsockopt(ZMQ_SNDTIMEO,500);

    m_genericHandlerId = m_coms->SetGenericHandler([this,zpub](const uint8_t *data,int len) mutable
                              {
                                ComsPacketTypeT cpt = (ComsPacketTypeT)data[0];
                                switch(cpt)
                                {
                                  case CPT_ReportParam: {
                                    // FIXME:- Pick out interesting things we should log.
                                    if(m_verbose) {
                                      PacketParam8ByteC *pkt = (PacketParam8ByteC *) data;
                                      std::string contents;
                                      for(int i = 0;i < len - sizeof(pkt->m_header);i++) {
                                        contents += " ";
                                        contents += std::to_string((int) pkt->m_data.uint8[i]);
                                      }
                                      m_log->info("Server got dev {} ReportParam {} : {} ",(int)pkt->m_header.m_deviceId,(int) pkt->m_header.m_index,contents);

                                    }
                                  } break;
                                  default:
                                    if(m_verbose) {
                                      m_log->info("Server got dev msg. {} {} ",ComsPacketTypeToString(cpt),len);
                                    }
                                  break;
                                }
                                try {
                                  zmq::message_t msg (len);
                                  memcpy (msg.data (), data, len);
                                  zpub->send(msg);
                                } catch(zmq::error_t &err) {
                                  m_log->error("Caught exception forwarding message %d '%s' ",err.num(),err.what());
                                }
                              }
    );

    ONDEBUG(m_log->info("Server run loop started."));
    try {
      while(!m_terminate) {
        zmq::message_t msg;
        if(!zserver->recv(&msg,0)) {
          if(m_verbose) {
            m_log->info("Server no msg.");
          }
          continue;
        }
        if(m_verbose) {
          m_log->info("Server got msg.");
        }
        m_coms->SendPacket((uint8_t *) msg.data(),msg.size());
      }
    } catch(zmq::error_t &err) {
      m_log->error("Caught exception run thread %d '%s' ",err.num(),err.what());
    }
    ONDEBUG(m_log->info("Server run loop exiting."));

    // Remove handler with reference to this instance.
    m_genericHandlerId.Remove();

    ONDEBUG(m_log->info("Server finished."));
  }

}
