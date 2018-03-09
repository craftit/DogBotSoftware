#ifndef DOGBOG_COMSZMQSERVER_HEADER
#define DOGBOG_COMSZMQSERVER_HEADER 1

#include "dogbot/Coms.hh"
#include <zmq.hpp>

namespace DogBotN {

  //! Server

  class ComsZMQServerC
  {
  public:
    ComsZMQServerC(const std::shared_ptr<ComsC> &coms,std::shared_ptr<spdlog::logger> &log);

    //! Make sure everything is disconnected.
    ~ComsZMQServerC();

    //! Set verbose mode for dumping messages
    void SetVerbose(bool verbose)
    { m_verbose = verbose; }

    //! Run server
    void Run(const std::string &addr);

  protected:
    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");

    std::shared_ptr<ComsC> m_coms;


    bool m_verbose = false;
    bool m_terminate = false;
    CallbackHandleC m_genericHandlerId;
  };

}

#endif
