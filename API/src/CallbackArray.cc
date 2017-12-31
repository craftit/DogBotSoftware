
#include "dogbot/CallbackArray.hh"

namespace DogBotN {

  //! virtual destructor to keep the compiler happy.
  AbstractCallbackArrayC::~AbstractCallbackArrayC()
  {}

  // -----------------------------------------------------

  //! Remove callback from list.

  void CallbackHandleC::Remove()
  {
    if(m_cb == 0)
      return;
    m_cb->Remove(m_id);
    m_cb = 0;
  }

  // -----------------------------------------------------

  CallbackSetC::~CallbackSetC()
  { RemoveAll(); }

  void CallbackSetC::RemoveAll()
  {
    while(!m_callbacks.empty()) {
      m_callbacks.back().Remove();
      m_callbacks.pop_back();
    }
  }

}
