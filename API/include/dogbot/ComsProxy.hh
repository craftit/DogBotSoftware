#ifndef DOGBOG_COMSPROXY_HEADER
#define DOGBOG_COMSPROXY_HEADER 1

#include "dogbot/Coms.hh"

namespace DogBotN {


  //! Communication proxy that allows switching between sources
  //! after setup

  class ComsProxyC
   : public ComsC
  {
  public:
    ComsProxyC(const std::shared_ptr<ComsC> &coms);

    //! default
    ComsProxyC();

    //! Destructor
    // Disconnects and closes file descriptors
    virtual ~ComsProxyC();

    //! Set coms object to use
    void SetComs(const std::shared_ptr<ComsC> &coms);

    //! Open a port.
    virtual bool Open(const std::string &portAddr) override;

    //! Close connection
    virtual void Close() override;

    //! Is connection ready ?
    virtual bool IsReady() const override;

    //! Send packet
    virtual void SendPacketWire(const uint8_t *data,int len) override;


  protected:
    CallbackHandleC m_genericHandlerId;
    std::shared_ptr<ComsC> m_coms;
    mutable std::mutex m_accessTx;

  };
}
#endif
