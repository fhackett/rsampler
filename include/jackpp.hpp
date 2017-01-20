#include <jack/jack.h>

namespace jackpp{

  struct client_t{
    struct data_t{
      std::unique_ptr<jack_client_t,void(*)(jack_client_t*)> impl;
      std::function<void(std::size_t)> process_callback;
    };
    std::shared_ptr<data_t> data;
    jack_client_t* get_client(){
      return data->impl.get();
    }
  };

  client_t open_client(const std::string& name, const std::function<int(std::size_t)>& process_callback){
    auto ret = client_t{
      client_t::data_t{
        std::unique_ptr<jack_client_t,void(*)(jack_client_t*)>{jack_client_open(name.c_str(), JackNullOption, nullptr), [](jack_client_t* c){
          assert(!jack_client_close(c));
        }},
        process_callback
      }
    };
    jack_set_process_callback(ret.impl.get(), [](jack_nframes_t frames, void* data) nothrow -> int{
      reinterpret_cast<client_t::data_t*>(data)->process_callback(frames);
    }, ret.data.get());
    return ret;
  }

  enum class port_flag_t{
    is_input = JackPortIsInput,
    is_output = JackPortIsOutput,
    is_physical = JackPortIsPhysical,
    can_monitor = JackPortCanMonitor,
    is_terminal = JackPortIsTerminal
  };

  struct port_flags_t{
    JackPortFlags data;
    port_mask_t(std::initializer_list<port_flag_t> flags){
      data = std::accumulate(flags.begin(), flags.end(), 0, [](JackPortFlags acc, port_flag_t f){
        return acc | static_cast<JackPortFlags>(f);
      });
    }
  };

  struct port_t{
    client_t client;
    std::shared_ptr<jack_port_t> impl;

    template<typename T>
    T* get_buffer(std::size_t nframes){
      return (T*)jack_port_get_buffer(impl.get(), nframes);
    }
  };

  port_t register_port(client_t& client, const std::string& name, const std::string& type, port_flags_t flags, std::size_t buffer_size = 0){
    return port_t{
      client,
      std::shared_ptr<jack_port_t>{
        jack_port_register(client.impl.get(),name.c_str(),type.c_str(),flags.data,buffer_size),
        [client=client](jack_port_t* p){
          assert(!jack_port_unregister(client.impl.get(),p));
        }
      }
    };
  }

}
