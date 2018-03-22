#ifndef DOGBOT_CALLBACK_ARRAY_HEADER
#define DOGBOT_CALLBACK_ARRAY_HEADER 1

#include <thread>
#include <vector>
#include <functional>
#include <cassert>
#include <mutex>

namespace DogBotN {

  template<typename FuncT> class CallbackArrayC;

  //! Abstract callback array base class.
  //! This provides an interface to for managing callback removal

  class AbstractCallbackArrayC
  {
  public:
    //!
    AbstractCallbackArrayC()
    {}

    //! virtual destructor to keep the compiler happy.
    virtual ~AbstractCallbackArrayC();

    //! Remove a callback from the list.
    virtual void Remove(int id) = 0;
  };

  //! Handle for a single callback
  class CallbackHandleC
  {
  public:
    //! Default constructor,
    //! Creates an invalid handle. It is safe to call Remove() on it which will have no effect.
    CallbackHandleC()
    {}

    //! Create from an array entry and an id
    CallbackHandleC(AbstractCallbackArrayC *cb,int id)
      : m_cb(cb),
        m_id(id)
    {}

    //! Remove callback from list if handle is valid, otherwise take no action.
    //! The handle will be changed to an invalid state after call back is removed.
    void Remove();

    //! Test if the callback is still active.
    bool IsActive() const
    { return m_cb != 0; }

  protected:
    AbstractCallbackArrayC *m_cb = 0;
    int m_id = -1;
  };

  //! Store a set of call backs that will be removed either when 'RemoveAll()' method is called or when the class is destructed.

  class CallbackSetC
  {
  public:
    //! Create an empty callback set.
    CallbackSetC()
    {}

    //! Destructor,
    // This will disconnect all stored callbacks.
    ~CallbackSetC();

    //! Add a new callback to the set
    CallbackSetC &operator+=(const CallbackHandleC &handle) {
      m_callbacks.push_back(handle);
      return *this;
    }

    //! Remove and disconnect all stored callbacks
    void RemoveAll();

  protected:
    std::vector<CallbackHandleC> m_callbacks;
  };

  //! Array of call backs to do on an event.

  template<typename FuncT>
  class CallbackArrayC
   : public AbstractCallbackArrayC
  {
  public:
    //! Default constructor
    CallbackArrayC()
    {}

    //! Get the array of functions
    //! Some handles may be invalid, so they need to be checked before calling.
    std::vector<FuncT> Calls()
    {
      std::lock_guard<std::mutex> lock(m_mutexAccess);
      return m_callbacks;
    }

    //! Add a new call back to the list.

    CallbackHandleC Add(const FuncT &callback)
    {
      std::lock_guard<std::mutex> lock(m_mutexAccess);
      // Free slot anywhere?
      for(int i = 0;i < (int) m_callbacks.size();i++) {
        if(!m_callbacks[i]) {
          m_callbacks[i] = callback;
          return CallbackHandleC(this,i);
        }
      }
      // Just create a new one
      int ret = (int) m_callbacks.size();
      m_callbacks.push_back(callback);
      return CallbackHandleC(this,ret);
    }

    //! Remove a callback

    virtual void Remove(int id) override
    {
      assert(id >= 0);
      assert(id < (int) m_callbacks.size());
      std::lock_guard<std::mutex> lock(m_mutexAccess);
      m_callbacks[id] = FuncT();
    }

  protected:
    std::mutex m_mutexAccess;
    std::vector<FuncT> m_callbacks;
  };




}


#endif
