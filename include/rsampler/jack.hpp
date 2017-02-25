#pragma once

#include <utility>

//#include <rsampler/pipe.hpp>

//#include <jack/jack.h>

namespace rsampler{
  namespace jack{
    template<typename, typename>
    struct _wrapper;
    template<typename WrappedReturnType, typename... WrappedArgs, typename Fn>
    struct _wrapper<WrappedReturnType(WrappedArgs...), Fn>{
      Fn fn;
      _wrapper(Fn fn): fn{std::move(fn)}{}
      WrappedReturnType operator()(WrappedArgs... args){
        return fn(std::forward<WrappedArgs>(args)...);
      }
    };

    template<typename Signature, typename Fn>
    auto _make_wrapper(Fn fn){
      return _wrapper<Signature, Fn>{fn};
    }

    class c_string{
      const char* p;
    public:
      c_string(const std::string& s): p{s.c_str()}{}
      c_string(const char* s): p{s}{}
      operator const char*(){
        return p;
      }
    };

    class client_t{
      std::unique_ptr<jack_client_t, decltype(jack_client_close)> ptr;
    public:
      client_t(jack_client_t* p): ptr{p, jack_client_close}{}
      operator jack_client_t*(){
        return ptr.get();
      }
    };

    client_t client_open(c_string name, jack_options_t options){
      auto* p = jack_client_open(name, options, nullptr);
      assert(p);
      return p;
    }

  }
}
