#pragma once

#include <rsampler/pipe.hpp>
#include <rsampler/data_store.hpp>
#include <rsampler/delayed_ptr.hpp>

namespace rsampler{
  template<typename Sample>
  class sampler: public connection_facade<sampler<Sample>>{
    friend connection_facade<sampler<Sample>>;

    using Data = ranges::range_value_t<Sample>;
    data_store::id_t id;
    delayed::delayed_ptr<Sample>& sample;
    std::size_t& position;
    Data current_frame;

    void advance(){
      ++position;
    }
    Data const& get_frame() const{
      if(sample && ranges::begin(*sample) + position < ranges::end(*sample)){
        ranges::copy(*(ranges::begin(*sample)+position), ranges::begin(const_cast<Data&>(current_frame)));
      }else{
        zero_frame(const_cast<Data&>(current_frame));
      }
      return current_frame;
    }
    bool is_done() const{
      return false;
    }
  public:
    sampler(data_store::id_t _id, std::size_t _width):
      id{_id},
      sample{data_store::ensure<delayed::delayed_ptr<Sample>>(id, 0, nullptr)},
      position{data_store::ensure<std::size_t>(id, 1, 0)}
    {
      init_frame(current_frame, _width);
    }
  };
}
