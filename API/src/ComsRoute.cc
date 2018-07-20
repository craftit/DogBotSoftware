
#include "dogbot/ComsRoute.hh"
#include "dogbot/ComsZMQClient.hh"
#include "dogbot/ComsSerial.hh"
#include "dogbot/ComsUSB.hh"
#include <iostream>

namespace DogBotN
{

  ComsRouteC::ComsRouteC(const std::shared_ptr<ComsC> &coms)
  {
    m_name = "route";
    AddComs(coms);
  }

  //! default
  ComsRouteC::ComsRouteC()
  {}

  // Disconnects and closes file descriptors
  ComsRouteC::~ComsRouteC()
  {
    Close();
  }

  //! Set
  void ComsRouteC::AddComs(const std::shared_ptr<ComsC> &coms)
  {
    assert(coms.get() != this); // On the off chance something tried to make a loop.
    if(!coms)
      return ;

    {
      auto callBack = coms->SetGenericHandler([this,coms](const uint8_t *data,int len) mutable
                                      {
                                        if(len > 2 && ((enum ComsPacketTypeT) data[0]) == CPT_AnnounceId) {
                                          int destId = data[1];
                                          if(destId != 0) {
                                            std::lock_guard<std::mutex> lock(m_accessTx);
                                            if(m_route.size() <= destId) {
                                              m_route.reserve(destId+1);
                                              while(m_route.size() <= destId)
                                                m_route.push_back(std::shared_ptr<ComsC>());
                                            }
                                            if(m_verbose) m_log->info("Adding route for device {} ",destId);
                                            m_route[destId] = coms;
                                          }
                                        }
                                        ProcessPacket(data,len);
                                      });

      std::lock_guard<std::mutex> lock(m_accessTx);
      coms->SetLogger(m_log);
      m_coms.push_back(ComsChannelC(callBack,coms));
    }

    // Send a query devices packet.
    {
      uint8_t data[2];
      data[0] = CPT_QueryDevices;
      coms->SendPacket(data,1);
    }
  }

  //! Remove coms channel
  void ComsRouteC::RemoveComs(const std::shared_ptr<ComsC> &coms)
  {
    if(!coms)
      return ;

    {
      std::lock_guard<std::mutex> lock(m_accessTx);
      for(auto it = m_coms.begin();it != m_coms.end();it++) {
        if(it->m_coms == coms) {
          m_coms.erase(it);
          break;
        }
      }
    }

  }


  //! Close connection
  void ComsRouteC::Close()
  {
    std::unique_lock<std::mutex> lock(m_accessTx);
    while(m_coms.size() > 0) {
      m_coms.back().m_coms->Close();
      m_coms.pop_back();
    }
  }

  //! Is connection ready ?
  bool ComsRouteC::IsReady() const
  {
    std::lock_guard<std::mutex> lock(m_accessTx);
    if(m_coms.empty())
      return false;
    return true;
  }

  //! Set the logger to use
  void ComsRouteC::SetLogger(const std::shared_ptr<spdlog::logger> &log)
  {
    ComsC::SetLogger(log);
    std::lock_guard<std::mutex> lock(m_accessTx);
    for(auto &a : m_coms)
      a.m_coms->SetLogger(log);
  }


  bool ComsRouteC::Open(const std::string &portAddr)
  {
    std::shared_ptr<ComsC> coms;
    auto colonAt = portAddr.find(':');
    std::string prefix;
    if(colonAt != std::string::npos) {
      prefix = portAddr.substr(0,colonAt);
    }
    if(std::string("usb") == portAddr) {
      coms = std::make_shared<ComsUSBC>();
    } else if(std::string("local") == portAddr || prefix == "tcp" || prefix == "udp") {
      coms = std::make_shared<ComsZMQClientC>();
    } else {
      coms = std::make_shared<ComsSerialC>();
    }
    coms->SetLogger(m_log);
    if(!coms->Open(portAddr))
      return false;
    AddComs(coms);
    return true;
  }

  //! Send packet
  void ComsRouteC::SendPacketWire(const uint8_t *buff,int len)
  {
    if(len > 1 && buff[0] != CPT_SetDeviceId) {
      int dest = buff[1];
      std::unique_lock<std::mutex> lock(m_accessTx);
      if(dest != 0 && m_route.size() > dest && m_route[dest]) {
        m_route[dest]->SendPacket(buff,len);
        return ;
      }
    }
    // Send it everywhere
    for(auto &a : m_coms)
      a.m_coms->SendPacket(buff,len);
  }


}
