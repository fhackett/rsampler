#pragma once

#include <range/v3/all.hpp>

namespace rsampler{

  /*template<typename Data>
  class any_pipe: public pipe_facade<any_pipe<Data>, Data>{
    struct interface{
      virtual buffer_view<Data> const get_frame() const = 0;
      virtual void advance() = 0;
      virtual std::size_t width() const = 0;
      virtual ~interface(){}
    };
    template<typename Implementation>
    struct interface_impl{
      Implementation impl;
      std::decay_t<decltype(begin(std::declval<Implementation>()))> _begin;
      std::decay_t<decltype(end(std::declval<Implementation>()))> _end;
      interface_impl(Implementation i):
        impl{std::move(i)},
        _begin{begin(impl)},
        _end{end(impl)}
      {}
      buffer_view<Data> const get_frame() const override{
        return *_begin;
      }
      void advance() override{
        ++_begin;
      }
      std::size_t width() const override{
        width(impl);
      }
    };
    std::unique_ptr<interface> impl;
  public:
    template<typename Implementation>
    any_pipe(Implementation i): impl{std::make_unique<interface_impl<Implementation>>(std::move(i))}{}
    buffer_view<Data> const get_frame() const{
      return impl->get_frame();
    }
    void advance(){
      impl->advance();
    }
    std::size_t width() const override{
      return impl->width();
    }
  };*/

  template<typename Input, typename Output>
  auto connect(Input l, Output r){
    return std::move(r).connect(l);
  }

  template<typename Frame>
  void zero_frame(Frame& d){
    ranges::fill(d, 0);
  }

  template<typename Frame>
  void init_frame(Frame&, std::size_t){
    // pass
  }

  template<typename Input, typename Output>
  auto operator>>(Input l, Output r){
    return connect(std::move(l), std::move(r));
  }

  template<typename Pipe>
  std::size_t width(Pipe const& p){
    return ranges::size(*ranges::begin(p));
  }

  template<typename Connection>
  class connection_facade: public ranges::view_facade<Connection>{
    friend ranges::range_access;

    Connection& derived(){
      return *static_cast<Connection*>(this);
    }
    Connection const& derived() const{
      return *static_cast<Connection const*>(this);
    }

    struct sentinel{};
    struct cursor{
      using Data = std::decay_t<decltype(std::declval<Connection>().get_frame())>;
      cursor() = default;
      cursor(Connection* p):
        conn{p}
      {}
      Data const& read() const{
        return conn->get_frame();
      }
      bool equal(sentinel const&) const{
        return conn->is_done();
      }
      void next(){
        conn->advance();
      }
    private:
      Connection* conn = nullptr;
    };

    cursor begin_cursor() const{
      return {const_cast<Connection*>(&derived())};
    }
    sentinel end_cursor() const{
      return {};
    }
    friend cursor;
  };
}
