#include <memory>

#include <samplerate.h>

namespace samplerate{
  class state{
    static void _delete(SRC_STATE* ptr){
      if(ptr){
        src_delete(ptr);
      }
    }
    int err;
    std::unique_ptr<SRC_STATE, void(*)(SRC_STATE*)> s;
    std::vector<float> buffer_space;
    std::size_t num_channels;

    struct cbdata_record{
      long(*callback)(void*,float*&);
      void* cb_data;
    };
    std::unique_ptr<cbdata_record> cbdata;
  public:
    state(std::size_t channels=1, int converter_type = SRC_SINC_FASTEST):
      s{nullptr, _delete},
      cbdata{new cbdata_record{nullptr, nullptr}},
      buffer_space(channels*BUFFER_SIZE, 0.0),
      num_channels{channels}
    {
      s = std::unique_ptr<SRC_STATE, void(*)(SRC_STATE*)>{
        src_callback_new(
          [](void* _self, float** data) -> long{
            cbdata_record* cb = (cbdata_record*)_self;
            if(cb->callback){
              return cb->callback(cb->cb_data, *data);
            }else{
              return 0;
            }
          },
          converter_type,
          channels,
          &err,
          cbdata.get()),
        _delete
      };
    }

    template<typename Fn, typename OutputIterator>
    void read(double ratio, std::size_t frames, OutputIterator output, Fn cb){
      cbdata->callback = [](void* ptr, float*& data) -> long{
        Fn& cb = *reinterpret_cast<Fn*>(ptr);
        return cb(data);
      };
      cbdata->cb_data = &cb;

      std::size_t frames_read = 0;
      while(frames_read < frames){
        std::size_t gen = src_callback_read(s.get(), ratio, std::min(BUFFER_SIZE, frames - frames_read), buffer_space.data());
        assert(gen != 0);
        output = std::copy_n(buffer_space.begin(), gen*num_channels, output);
        frames_read += gen;
      }

      cbdata->callback = nullptr;
      cbdata->cb_data = nullptr;
      return frames_read;
    }
  };
}
