#pragma once

#include <rsampler/pipe.hpp>
#include <rsampler/data_store.hpp>

#include <samplerate.h>

namespace rsampler{
  class resampler{
    data_store::id_t id;
    int converter_type;

    template<typename Source>
    struct connection: public connection_facade<connection<Source>>{
      using Data = std::decay_t<decltype(*std::declval<ranges::range_iterator_t<Source>>())>;

      struct callback_state_t{
        Source source;
        ranges::range_iterator_t<Source> iterator;
        Data data;
        callback_state_t(Source _source):
          source{std::move(_source)},
          iterator{ranges::begin(source)}
        {
          init_frame(data, width(source));
        }
      };

      bool _is_done = false;
      data_store::id_t id;
      double& ratio;
      Data output_data;
      std::unique_ptr<callback_state_t> callback_state;
      std::unique_ptr<SRC_STATE, void(*)(SRC_STATE*)> src_state;
      int err;

      connection(data_store::id_t _id, Source _source, int _converter_type):
        id{std::move(_id)},
        ratio{data_store::ensure<double>(id, 0, 1)},
        callback_state{std::make_unique<callback_state_t>(std::move(_source))},
        src_state{
          src_callback_new(
            [](void* _callback_state, float** data) -> long{
              auto* callback_state = static_cast<callback_state_t*>(_callback_state);
              if(callback_state->iterator != ranges::end(callback_state->source)){
                ranges::copy(*callback_state->iterator, ranges::begin(callback_state->data));
                ++callback_state->iterator;
                *data = callback_state->data.data();
                return 1;
              }else{
                return 0;
              }
            }, _converter_type, width(callback_state->source), &err, callback_state.get()),
          [](SRC_STATE* ptr){src_delete(ptr);}
        }
      {
        assert(src_state);
        init_frame(output_data, width(callback_state->source));
        advance();
      }

      void advance(){
        long generated_frames = src_callback_read(src_state.get(), ratio, 1, output_data.data());
        if(generated_frames == 0){
          _is_done = true;
        }
      }
      Data const& get_frame() const{
        return output_data;
      }
      bool is_done() const{
        return _is_done;
      }
    };
  public:
    template<typename Source>
    connection<Source> connect(Source src){
      return connection<Source>{id, std::move(src), converter_type};
    }

    resampler(data_store::id_t _id, int _converter_type):
      id{_id},
      converter_type{_converter_type}
    {}
  };
}
