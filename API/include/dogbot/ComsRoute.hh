#ifndef DOGBOG_COMSROUTE_HEADER
#define DOGBOG_COMSROUTE_HEADER 1

#include "dogbot/Coms.hh"

namespace DogBotN {


  //! Communication router that allows multiple sources to be connected

  class ComsRouteC
   : public ComsC
  {
  public:
    ComsRouteC(const std::shared_ptr<ComsC> &coms);

    //! default
    ComsRouteC();

    //! Destructor
    // Disconnects and closes file descriptors
    virtual ~ComsRouteC();

    //! Add coms channel
    void AddComs(const std::shared_ptr<ComsC> &coms);

    //! Open a port.
    virtual bool Open(const std::string &portAddr) override;

    //! Close connection
    virtual void Close() override;

    //! Is connection ready ?
    virtual bool IsReady() const override;

    //! Send packet
    virtual void SendPacketWire(const uint8_t *data,int len) override;


  protected:
    std::vector<CallbackHandleC> m_genericHandlerId;
    std::vector<std::shared_ptr<ComsC> > m_coms; // Array of coms handles
    std::vector<std::shared_ptr<ComsC> > m_route; // Coms class to use for each device
    mutable std::mutex m_accessTx;

  };
}
#endif
