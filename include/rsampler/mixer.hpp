#pragma once

#include <rsampler/pipe.hpp>
#include <range/v3/all.hpp>

namespace rsampler{
  template<typename Data>
  class mixer{
    template<typename Sources>
    struct connection: public connection_facade<connection<Sources>>{
      Sources sources;
      Data current_frame;
      using SourceIterator = ranges::range_iterator_t<ranges::range_value_t<Sources>>;//std::decay_t<decltype(ranges::begin(*ranges::begin(std::declval<Sources>())))>;
      std::vector<SourceIterator> source_iterators;
      std::size_t _width;
      connection(Sources _sources):
        sources{std::move(_sources)},
        current_frame{}
      {
        using rsampler::width;
        using rsampler::init_frame;
        _width = ranges::size(sources) != 1 ?
          ranges::accumulate(
              ranges::view::zip_with([](auto const& a, auto const& b){
                assert(width(a) == width(b));
                return width(a);
              }, sources, ranges::view::tail(sources)),
              0,
              [](std::size_t acc, std::size_t w){ return w; })
          : width(*ranges::begin(sources));
        init_frame(current_frame, _width);
        source_iterators = ranges::to_<std::vector<SourceIterator>>(
          ranges::view::transform(sources, [](auto& src){
            return ranges::begin(src);
          }));
        calculate();
      }

      void calculate(){
        zero_frame(current_frame);
        ranges::for_each(source_iterators,[&](auto& it){
          ranges::copy(
            ranges::view::zip_with([](auto const& a, auto const& b){
              return a+b;
            }, current_frame, *it),
            ranges::begin(current_frame));
        });
      }

      Data const& get_frame() const{
        return current_frame;
      }
      void advance(){
        ranges::for_each(source_iterators,[&](auto& it){
          ++it;
        });
        if(!is_done()){
          calculate();
        }
      }
      bool is_done() const{
        return ranges::any_of(
          ranges::view::zip_with([](auto& it, auto& src){
            return it == ranges::end(src);
          },source_iterators, sources),
          [](bool b){
            return b;
          });
      }
    };
  public:
    template<typename Sources>
    connection<Sources> connect(Sources sources){
      return {std::move(sources)};
    }
  };
}
