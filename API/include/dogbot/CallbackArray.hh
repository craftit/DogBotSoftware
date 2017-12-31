#ifndef CALLBACK_ARRAY_HEADER
#define CALLBACK_ARRAY_HEADER 1

#include <thread>
#include <vector>
#include <functional>
#include <cassert>
#include <mutex>

namespace DogBotN {

  template<typename FuncT> class CallbackArrayC;


  //! Array of callbacks to do on an event.

  template<typename FuncT>
  class CallbackArrayC
  {
  public:
    CallbackArrayC();

    std::vector<FuncT> Calls()
    {
      std::lock_guard<std::mutex> lock(m_mutexAccess);
      return m_callbacks;
    }

    int Add(const FuncT &callback)
    {
      std::lock_guard<std::mutex> lock(m_mutexAccess);
      // Free slot anywhere?
      for(int i = 0;i < (int) m_callbacks.size();i++) {
        if(!m_callbacks[i]) {
          m_callbacks[i] = callback;
          return i;
        }
      }
      // Just create a new one
      int ret = (int) m_callbacks.size();
      m_callbacks.push_back(callback);
      return ret;
    }

    void Remove(int id)
    {
      assert(id >= 0);
      assert(id < m_callbacks.size());
      std::lock_guard<std::mutex> lock(m_mutexAccess);
      m_callbacks[id] = typename FuncT();
    }

  protected:
    std::mutex m_mutexAccess;
    std::vector<FuncT> m_callbacks;
  };

  template<typename FuncT>
  class CallbackHandleC
  {
  public:
    CallbackHandleC(CallbackArrayC<FuncT> *cb,int id)
  protected:
    int m_id = 0;
    CallbackArrayC<FuncT> *m_cb = 0;
  };


}


#endif
