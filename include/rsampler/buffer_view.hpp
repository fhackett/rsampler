#pragma once

#include <type_traits>
#include <range/v3/all.hpp>

namespace rsampler{

  template<typename Data>
  class iterable_view: public ranges::view_facade<iterable_view<Data>, ranges::finite>{
    friend ranges::range_access;

    struct erased_iterator_pair{
      virtual Data const& dereference() const = 0;
      virtual void increment() = 0;
      virtual bool test_at_end() const = 0;
    };
    template<typename Iterable>
    struct erased_iterator_pair_impl: erased_iterator_pair{
      using Iterator = std::decay_t<decltype(ranges::begin(std::declval<Iterable>()))>;
      using Sentinel = std::decay_t<decltype(ranges::end(std::declval<Iterable>()))>;
      Iterator _current;
      Sentinel _end;
      erased_iterator_pair_impl() = default;
      erased_iterator_pair_impl(Iterable& c):
        _current{ranges::begin(c)},
        _end{ranges::end(c)}
      {}
      Data const& dereference() const override{
        return *_current;
      }
      void increment() override{
        ++_current;
      }
      bool test_at_end() const override{
        return _current == _end;
      }
    };
    erased_iterator_pair* _it = nullptr;

    struct sentinel{};
    struct cursor{
    private:
      erased_iterator_pair* v{nullptr};
    public:
      cursor() = default;
      cursor(erased_iterator_pair* p):
        v{p}
      {}
      Data const & read() const{
        return v->dereference();
      }
      bool equal(sentinel const &) const{
        return v->test_at_end();
      }
      void next(){
        v->increment();
      }
    };
    cursor begin_cursor() const{
      return {_it};
    }
    sentinel end_cursor() const{
      return {};
    }
  public:
    template<typename Iterable>
    iterable_view(Iterable&& container, erased_iterator_pair_impl<Iterable>&& _it = erased_iterator_pair_impl<Iterable>{}):
      _it{&_it}
    {
      _it = erased_iterator_pair_impl<Iterable>{container};
    }
  };
}
