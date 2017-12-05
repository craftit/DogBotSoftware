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

    //! Run server
    void Run(const std::string &addr);

  protected:
    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");

    std::shared_ptr<ComsC> m_coms;

    std::shared_ptr<zmq::socket_t> m_server; // Receives commands from clients for forwarding on
    std::shared_ptr<zmq::socket_t> m_pub;   // Publish state messages

    bool m_terminate = false;
    int m_genericHandlerId = -1;
  };

}

#endif
