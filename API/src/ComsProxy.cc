
#include "dogbot/ComsProxy.hh"
#include "dogbot/ComsZMQClient.hh"
#include "dogbot/ComsSerial.hh"
#include "dogbot/ComsUSB.hh"
#include <iostream>

namespace DogBotN
{

  ComsProxyC::ComsProxyC(const std::shared_ptr<ComsC> &coms)
   : m_coms(coms)
  {}

  //! default
  ComsProxyC::ComsProxyC()
  {}

  // Disconnects and closes file descriptors
  ComsProxyC::~ComsProxyC()
  {
    m_genericHandlerId.Remove();
  }

  //! Set
  void ComsProxyC::SetComs(const std::shared_ptr<ComsC> &coms)
  {
    assert(coms.get() != this); // On the off chance something tried to make a loop.
    std::lock_guard<std::mutex> lock(m_accessTx);

    m_genericHandlerId.Remove();
    m_coms = coms;
    if(m_coms) {
      m_genericHandlerId = m_coms->SetGenericHandler([this](const uint8_t *data,int len) mutable
                                {
                                  ProcessPacket(data,len);
                                }
      );
    }
  }

  //! Close connection
  void ComsProxyC::Close()
  {
    m_coms->Close();
    std::cerr << "Disconnecting proxy. " << std::endl;
    SetComs(std::make_shared<ComsC>());
  }

  //! Is connection ready ?
  bool ComsProxyC::IsReady() const
  {
    std::lock_guard<std::mutex> lock(m_accessTx);
    if(!m_coms)
      return false;
    return m_coms->IsReady();
  }

  bool ComsProxyC::Open(const std::string &portAddr)
  {
    std::shared_ptr<ComsC> coms;
    auto colonAt = portAddr.find(':');
    std::string prefix;
    if(colonAt != std::string::npos) {
      prefix = portAddr.substr(0,colonAt);
    }
    std::cerr << "Got prefix '" << prefix << "' " << std::endl;
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
    SetComs(coms);
    return true;
  }

  //! Send packet
  void ComsProxyC::SendPacketWire(const uint8_t *buff,int len)
  {
    std::lock_guard<std::mutex> lock(m_accessTx);
    if(!m_coms)
      return ;
    m_coms->SendPacket(buff,len);
  }


}
