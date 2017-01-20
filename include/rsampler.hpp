#include <memory>
#include <algorithm>

#include <resampler.hpp>

namespace rsampler{
  template<typename Value>
  class buffer_view{
    Value* _data;
    std::size_t length;
  public:
    using iterator = Value*;
    using const_iterator = Value const *;

    buffer_view(Value* data, std::size_t length):
      _data{data}, length{length}{}

    template<Container>
    buffer_view(const Container& c): buffer_view{c.data(),c.size()}{}

    iterator begin(){
      return cbegin();
    }
    iterator end(){
      return cend();
    }
    const_iterator cbegin() const {
      return _data;
    }
    const_iterator cend() const{
      return _data + length;
    }
    Value& operator[](std::size_t pos){
      return _data[pos];
    }
    std::size_t size() const{
      return length;
    }
    Value* data(){
      return _data;
    }
  };

  template<typename MutableIterator>
  class additive_output_iterator{
    MutableIterator ptr;
  public:
    additive_output_iterator(MutableIterator p): ptr{p}{}
    additive_output_iterator<MutableIterator>& operator++(){
      ++ptr;
      return *this;
    }
    additive_output_iterator<MutableIterator> operator++(int){
      return additive_output_iterator<MutableIterator>{ptr++};
    }
    class adder{
      MutableIterator m;
    public:
      adder(MutableIterator m): m{m}{}
      adder& operator=(auto const& v){
        *m += v;
        return *this;
      }
    };
    adder operator*(){
      return adder{*ptr};
    }
  };

  template<typename MutableIterator>
  auto make_additive_output_iterator(MutableIterator it){
    return additive_output_iterator<MutableIterator>{it};
  }

  namespace data_store{
    struct _record_base{
      virtual ~_record_base() = default;
    };
    template<typename T>
    struct _record: _record_base{
      _record(T&& t): data{std::make_shared<T>(std::move(t))}{}
      std::shared_ptr<T> data;
    };
    struct _id{
      unsigned int _num;
      _id(unsigned int num): _num{num}{}
      ~_id();
    };
    unsigned int _next_free_id = 1;
    std::unordered_map<unsigned int,std::map<std::size_t,std::unique_ptr<_record_base>>> _store;
    _id::~_id(){
      _store.erase(_num);
      _next_free_id = _num;
    }

    using id_t = std::shared_ptr<_id>;

    id_t make_id(){
      unsigned int tmp = _next_free_id;
      while(_store.count(tmp) && tmp != 0){
        ++tmp;
      }
      _next_free_id = tmp + 1;
      _store.insert(std::make_pair(tmp,std::map<std::size_t,std::unique_ptr<_record_base>>{}));
      return std::make_shared<_id>(tmp);
    };

    template<typename T, typename Fn>
    std::shared_ptr<T> lazy_ensure(const id_t& id, std::size_t index, Fn fn){
      auto& map = _store[id->_num];
      auto it = map.find(index);
      if(it != map.end()){
        auto ptr = it->get();
        return dynamic_cast<_record<T>&>(*ptr).data;
      }else{
        auto rec = std::make_unique<_record<T>>(fn());
        map[index] = rec;
        return rec->data;
      }
    }

    template<typename T>
    std::shared_ptr<T> ensure(const id_t& id, std::size_t index, T&& v){
      return lazy_ensure(id,index,[&]{
        return std::move(v);
      });
    }
  }

  template<Value>
  using buffer_t = std::array<Value,BUFFER_SIZE>;

  template<typename Value>
  class source_t{
    struct base_t{
      virtual void generate(buffer_view<Value>, std::size_t) = 0;
      virtual long arity() const = 0;
      virtual ~base(){}
    };
    template<typename Type>
    struct impl_t: base_t{
      Type obj;
      impl_t(Type&& obj): obj{std::move(obj)}{}
      void generate(buffer_view<Value> buf, std::size_t count) override{
        generate(obj, buf, count);
      }
      long arity() const override{
        arity(obj);
      }
    };
    std::unique_ptr<base_t> impl;
  public:
    template<typename Type>
    source_t(Type&& tp): impl{std::make_unique{impl_t<Type>{std::move(tp)}}} {}

    void generate(buffer_view<Value> buf, std::size_t count){
      impl->generate(buf,count);
    }
    long arity() const{
      return impl->arity();
    }
  };
  template<typename Value>
  long arity(const source_t<Value>& src){
    return src.arity();
  }

  using frame_t = float;

  struct mixer_t{
    std::vector<source_t<frame_t>> sources;
  };
  mixer_t make_mixer(std::vector<source_t<frame_t>>&& source){
    return mixer_t{
      std::move(sources)
    };
  }

  void generate(mixer_t& m, buffer_view<frame_t> output, std::size_t count){
    buffer_t<frame_t> buffer;
    std::fill_n(output.begin(), count, 0);

    for(auto& src: m.sources){
      std::size_t generated = 0;
      auto out_it = make_additive_output_iterator(output.begin());
      while(generated < count){
        std::size_t const gen = std::min(count-generated, BUFFER_SIZE);
        src.generate(buffer, gen);
        out_it = std::copy_n(buffer.begin(), gen, out_it));
        generated += gen;
      }
    }
  }
  long arity(const mixer_t&){
    assert(false);
  }

  /*template<typename T, typename OutputIterator>
  std::size_t interleave(buffer_view<buffer_view<T>> bbv, OutputIterator out){
    for(std::size_t pos = 0;;++pos)
      for(auto& bv : bbv){
        if(pos >= bv.size()){
          return pos;
        }
        *out = bv[pos];
        ++out;
      }
    }
  }*/
  void interleave(const auto& buffers, auto&& dest){
    for(const auto& buf: buffers){
      ranges::copy(buf, dest | ranges::view::stride(dest, arity(buffers)));
    }
    
  }

  /*template<typename T, typename InputIterator, typename Sentinel>
  std::size_t deinterleave(InputIterator in, Sentinel end, buffer_view<buffer_view<T>> bbv){
    auto bv_it = bbv.begin();
    std::size_t pos = 0;
    for(;in!=end;++in){
      if(bv_it == bbv.end()){
        bv_it = bbv.begin();
        ++pos;
      }
      (*bv_it)[pos] = *in;
    }
    return pos;
  }*/
  void deinterleave(const auto& src, auto&& buffers){
    for(auto&& buf: buffers){
      ranges::copy(src | ranges::view::stride(src, arity(buffers)), buf);
    }
  }

  struct resampler_t{
    data_store::id_t id;
    source_t<buffer_view<frame_t>> sources;
    std::vector<buffer_t<frame_t>> temp_in_buffers;
    std::vector<frame_t> out_buffer;
    std::shared_ptr<double> ratio;
    std::shared_ptr<std::vector<float>> in_buffer;
    std::shared_ptr<resampler::state> state;
    std::size_t num_channels;
  };
  resampler_t make_resampler(data_store::id_t id, source_t<buffer_view<frame_t>>&& sources){
    std::size_t num_channels = arity(sources);
    return resampler_t{
      id,
      std::move(sources),
      std::vector<buffer_t<frame_t>>(num_channels),
      std::vector<frame_t>(num_channels*BUFFER_SIZE),
      data_store::ensure<double>(id,0,1.0),
      data_store::lazy_ensure<std::vector<float>>(id,1,[&]{
        return std::vector<float>(num_channels*BUFFER_SIZE);
      }),
      data_store::lazy_ensure<resampler::state>(id,2,[&]{
        return resampler::state(num_channels);
      }),
      num_channels
    };
  }
  void generate(resampler_t& self, buffer_view<buffer_view<frame_t>> output, std::size_t count){
    self.state->read(*self.ratio, count, out_buffer.begin(), [&](float*& ptr){
      self.sources.generate(
        buffer_view<buffer_view<frame_t>>{self.temp_in_buffers},
        std::min(BUFFER_SIZE,std::max(1,count/(*self.ratio))));
      interleave(self.temp_in_buffers, self.in_buffer);
      ptr = self.in_buffer->data();
    });
    deinterleave(out_buffer, output);
  }
  long arity(const resampler_t& rs){
    return rs.num_channels;
  }

  class sample_t{
    delayed::delayed_ptr<std::vector<frame_t>> data;
  public:
    sample_t():
      data{delayed::make_delayed<std::vector<frame_t>>()}
    {}
    sample_t(std::vector<frame_t>&& d):
      data{delayed::make_delayed<std::vector<frame_t>>(std::move(d))}
    {}
    auto begin() const{
      return data->begin();
    }
    auto end() const{
      return data->end();
    }
    std::size_t size() const{
      return data->size();
    }
  };

  struct sampler_t{
    data_store::id_t id;
    std::shared_ptr<sample_t> sample;
    std::shared_ptr<std::size_t> start, play_count;
  };
  sampler_t make_sampler(data_store::id_t id){
    return sampler_t{
      id,
      data_store::ensure(id,0,sample_t{}),
      data_store::ensure(id,1,0),
      data_store::ensure(id,2,0)
    };
  }
  void generate(sampler_t& self, buffer_view<frame_t> output, std::size_t count){
    std::size_t play_count = std::min(self.play_count,count);
    auto out = output.begin();
    if(self.sample){
      out = std::copy_n(self.sample->begin() + start, play_count, out);
    }
    std::fill_n(out,count-play_count,0.0);
    *self.start += play_count;
    *self.play_count -= play_count;
  }
  long arity(const sampler_t&){
    return 1;
  }

  template<typename Data>
  class sink_t{
    struct base_t{
      virtual void set_source(source_t<Data>&& src) = 0;
      virtual void run(std::size_t) = 0;
      virtual ~base_t() = default;
    };
    template<typename T>
    struct impl_t: base_t{
      T data;
      impl_t(T&& t):
        data{std::move(t)}
      {}
      void set_source(source_t<Data>&& src) override{
        set_source(data, std::move(src));
      }
      void run(std::size_t frames) override{
        run_sink(data, frames);
      }
    };
    std::unique_ptr<base_t> impl;
  public:
    void set_source(source_t<Data>&& src){
      impl->replace_source(std::move(src));
    }
    void run(std::size_t frames){
      impl->run(frames);
    }
  };

  class interface_t{
    struct base_t{
      virtual sink_t<buffer_view<frame_t>> register_sink(const std::string&, source_t<buffer_view<frame_t>>&&) = 0;
      virtual ~base_t() = default;
    };
    template<typename T>
    struct impl_t: base_t{
      impl_t(T&& t): data{std::move(t)}{}
      T data;
      sink_t<buffer_view<frame_t>> register_sink(const std::string& name, source_t<buffer_view<frame_t>>&& src) override{
        return register_sink(data,name,src);
      }
    };
    std::unique_ptr<base_t> impl;
  public:
    template<typename T>
    interface_t(T&& t):
      impl{std::make_unique<impl_t<T>>(std::move(t))}
    {}

    sink_t<buffer_view<frame_t>> register_sink(const std::string& name, source_t<buffer_view<frame_t>>&& src){
      return impl->register_sink(name, std::move(src));
    }

  };

  struct jack_interface_t{
    struct data_t{
      delayed::delayed_ptr<std::vector<sink_t<buffer_view<frame_t>>>> callbacks;
    };
    jackpp::client_t client;
    std::shared_ptr<data_t> data;
  };

  jack_interface_t make_jack_interface(const std::string& name){
    auto data = std::make_shared<jack_interface_t::data_t>(jack_interface_t::data_t{
      delayed::make_delayed<std::vector<sink_t<buffer_view<frame_t>>>>()
    });
    return jack_interface_t{
      jackpp::open_client(name, [data=data](std::size_t frames){
        auto d = data->callbacks;
        for(auto& sink: *d){
          sink.run(frames);
        }
      }),
      data
    };
  }

}
