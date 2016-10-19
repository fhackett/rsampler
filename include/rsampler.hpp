#pragma once

#include <array>
#include <vector>
#include <memory>
#include <limits>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <set>
#include <functional>
#include <atomic>
#include <mutex>
#include <thread>
#include <cstdint>

#include <cassert>

#include <jack/jack.h>
#include <samplerate.h>
#include <sndfile.h>

#include <iostream>

namespace rsampler{

  typedef float frame_t;
  typedef std::size_t frames_t;

  const frames_t BUFFER_SIZE = 64;
  typedef std::array<frame_t, BUFFER_SIZE> buffer_t;

  template<typename T>
  class nbqueue{
    std::array<T, BUFFER_SIZE> buffer;
    std::atomic<std::size_t> read_head{0}, write_head{0};
    std::mutex m;
  public:
    void put(T&& t){
      std::lock_guard<std::mutex> lock(m);
      while((write_head+1)%buffer.size() == read_head){
        std::this_thread::yield();
      }
      buffer[write_head++] = std::move(t);
    }
    template<typename Fn>
    bool get(Fn fn){
      if(read_head!=write_head){
        std::size_t rh = read_head;
        fn(buffer[rh]);
        read_head = (rh + 1) % buffer.size();
        return true;
      }
      return false;
    }
  };

  template<typename... Args>
  void trampoline(Args...){}

  template<typename... Args, typename Fn>
  void for_each(Fn&& fn, Args&... args){
    auto exec = [&](auto&& x){
      fn(std::move(x));
      return 0;
    };
    trampoline(exec(std::move(args))...);
  }

  template<typename... VectorArgs>
  class vector_rvalue_interface{
    using vec = std::vector<VectorArgs...>;
    vec v;
  public:
    vector_rvalue_interface(vec&& v): v{std::move(v)}{}
    template<typename... Args>
    vector_rvalue_interface(Args... args){
      v.reserve(sizeof...(args));
      for_each<Args...>([&](typename vec::value_type&& t){
        v.emplace_back(std::move(t));
      }, args...);
    }
    operator vec() &&{
      return std::move(v);
    }
  };

  namespace gc{
    class garbage_collector{
      struct _record_base{
        std::atomic<int> refcount{0};
        virtual ~_record_base(){}
      };
      template<typename T>
      struct _record: _record_base{
        T data;
        _record(T&& t): data{std::move(t)}{}
      };
      std::set<std::unique_ptr<_record_base>> records;
    protected:
      std::map<unsigned int,std::function<void()>> leak_suppressors;
      std::set<unsigned int> tags;
      unsigned int next_tag = 1;
    public:
      template<typename T>
      class gc_ptr{
        _record_base* impl;
        T* ptr;
      protected:
        gc_ptr(_record_base* _impl, T* ptr): impl{_impl}, ptr{ptr}{
          if(impl){
            impl->refcount++;
          }
        }
      public:
        gc_ptr(): gc_ptr{nullptr, nullptr}{}
        gc_ptr(const gc_ptr<T>& p): gc_ptr{p.impl, p.ptr}{}
        template<typename U>
        gc_ptr(const gc_ptr<U>& p): gc_ptr{p.impl, p.ptr}{}
        gc_ptr<T>& operator=(const gc_ptr<T>& p){
          if(impl){
            impl->refcount--;
          }
          impl = p.impl;
          ptr = p.ptr;
          if(impl){
            impl->refcount++;
          }
          return *this;
        }
        bool unique() const{
          assert(impl);
          return impl->refcount == 1;
        }
        T& operator*(){
          assert(ptr);
          return *ptr;
        }
        const T& operator*() const{
          assert(ptr);
          return *ptr;
        }
        T* operator->(){
          assert(ptr);
          return ptr;
        }
        T const * operator->() const{
          assert(ptr);
          return ptr;
        }
        operator bool() const{
          return impl;
        }
        template<typename U>
        gc_ptr<U> dynamic_pointer_cast() const{
          U* other_ptr = dynamic_cast<U*>(ptr);
          if(other_ptr){
            return gc_ptr<U>{impl, other_ptr};
          }else{
            return gc_ptr<U>{};
          }
        }
        ~gc_ptr(){
          if(impl){
            impl->refcount--;
          }
        }

        friend class garbage_collector;
      };
      template<typename T, typename... Args>
      gc_ptr<T> make_gc(Args... args){
        auto p = std::make_unique<_record<T>>(T(std::forward<Args>(args)...));
        auto result = gc_ptr<T>{p.get(), &p->data};
        records.insert(std::move(p));
        return result;
      }
      unsigned int make_tag(){
        unsigned int tag;
        while(tag = next_tag++, tags.count(tag) || tag == 0);
        tags.insert(tag);
        return tag;
      }
      class leak_suppressor{
        unsigned int tag;
        garbage_collector& gc;
      protected:
        leak_suppressor(unsigned int tag, garbage_collector& gc):
          tag{tag},
          gc{gc}
        {}
      public:
        leak_suppressor(const leak_suppressor&) = delete;
        leak_suppressor& operator=(const leak_suppressor&) = delete;
        leak_suppressor(leak_suppressor&& l):
          tag{l.tag},
          gc{l.gc}
        {
          l.tag = 0;
        }
        ~leak_suppressor(){
          if(tag){
            gc.tags.erase(tag);
            gc.leak_suppressors.erase(tag);
          }
        }

        friend class garbage_collector;
      };
      friend class leak_suppressor;
      leak_suppressor make_leak_suppressor(std::function<void()> fn){
        unsigned int tag = make_tag();
        leak_suppressors.insert(std::make_pair(tag, fn));
        return leak_suppressor{tag, *this};
      }
      void collect_garbage(){
        for(auto& ls: leak_suppressors){
          ls.second();
        }
        auto it = records.begin();
        while(it != records.end()){
          if((*it)->refcount == 0){
            it = records.erase(it);
          }else{
            ++it;
          }
        }
      }
    };

    template<typename T>
    using gc_ptr = garbage_collector::gc_ptr<T>;
    using leak_suppressor = garbage_collector::leak_suppressor;

    garbage_collector& get_default_gc(){
      static garbage_collector gc;
      return gc;
    }

    template<typename T, typename... Args>
    gc_ptr<T> make_gc(Args... args){
      auto& gc = get_default_gc();
      return gc.make_gc<T>(std::forward<Args>(args)...);
    }

    leak_suppressor make_leak_suppressor(std::function<void()> fn){
      return get_default_gc().make_leak_suppressor(fn);
    }

    void collect_garbage(){
      get_default_gc().collect_garbage();
    }
  }

  class buffer_view{
    frame_t* _data;
    frames_t length;
  public:
    typedef frame_t* iterator;
    typedef frame_t const* const_iterator;

    buffer_view(): _data{nullptr}, length{0}{}
    buffer_view(frame_t* data, frames_t length): _data{data}, length{length}{}
    template<typename Container>
    buffer_view(Container& _data): _data(_data.data()), length{_data.size()}{}

    void set(frame_t* data, frames_t length){
      this->_data = data;
      this->length = length;
    }
    operator bool() const{
      return _data;
    }
    frame_t& operator[](frames_t i){
      return *(_data + i);
    }
    frames_t size() const{
      return length;
    }
    frame_t* data() const{
      return _data;
    }
    iterator begin() const{
      return _data;
    }
    const_iterator cbegin() const{
      return _data;
    }
    iterator end() const{
      return _data + length;
    }
    const_iterator cend() const{
      return _data + length;
    }
    buffer_view subdivide(frames_t start, frames_t len) const{
      return buffer_view{_data + start, len};
    }
  };

  class buffer_view_list{
    void* _data;
    std::size_t length;
    buffer_view(*converter)(void*,std::size_t);
  public:
    struct iterator{
      void* _data;
      std::size_t pos;
      buffer_view(*converter)(void*,std::size_t);

      iterator& operator++(){
        ++pos;
        return *this;
      }
      iterator operator++(int){
        auto cp = *this;
        ++pos;
        return cp;
      }
      iterator& operator--(){
        --pos;
        return *this;
      }
      iterator operator--(int){
        auto cp = *this;
        --pos;
        return cp;
      }
      buffer_view operator*() const{
        return converter(_data, pos);
      }
      std::ptrdiff_t operator-(const iterator& it) const{
        return pos - it.pos;
      }
      iterator operator-(std::ptrdiff_t d) const{
        auto cp = *this;
        cp.pos -= d;
        return cp;
      }
      iterator operator+(std::ptrdiff_t d) const{
        auto cp = *this;
        cp.pos += d;
        return cp;
      }
    };
    typedef iterator const_iterator;

    buffer_view_list():
      _data{nullptr},
      length{0},
      converter{nullptr}
    {}
    template<typename Container, typename = typename std::enable_if<!std::is_same<Container, buffer_view_list>::value, void>::type>
    buffer_view_list(Container& _data):
      _data{reinterpret_cast<void*>(_data.data())},
      length{_data.size()},
      converter{[](void* ptr, std::size_t pos){
        auto* p = reinterpret_cast<typename Container::value_type*>(ptr);
        return buffer_view{p[pos]};
      }}
    {}
    operator bool() const{
      return _data;
    }
    buffer_view operator[](std::size_t i){
      return converter(_data, i);
    }
    std::size_t size() const{
      return length;
    }
    iterator at(std::size_t i) const{
      return iterator{_data, i, converter};
    };
    iterator begin() const{
      return at(0);
    }
    const_iterator cbegin() const{
      return at(0);
    }
    iterator end() const{
      return at(length);
    }
    const_iterator cend() const{
      return at(length);
    }
  };

  class sample: public buffer_view{
    gc::gc_ptr<std::vector<frame_t>> frames;
  public:
    sample(std::vector<frame_t>&& f): frames{gc::make_gc<std::vector<frame_t>>(std::move(f))}{}
    template<typename... Args>
    sample(Args... args):
      frames{gc::make_gc<std::vector<frame_t>>(std::forward<Args>(args)...)}
    {
      set(frames->data(), frames->size());
    }
  };

  namespace sndfile{
    enum class type: int{
      wav = SF_FORMAT_WAV, /* Microsoft WAV format (little endian). */
      aiff = SF_FORMAT_AIFF, /* Apple/SGI AIFF format (big endian). */
      au = SF_FORMAT_AU, /* Sun/NeXT AU format (big endian). */
      raw = SF_FORMAT_RAW, /* RAW PCM data. */
      paf = SF_FORMAT_PAF, /* Ensoniq PARIS file format. */
      svx = SF_FORMAT_SVX, /* Amiga IFF / SVX8 / SV16 format. */
      nist = SF_FORMAT_NIST, /* Sphere NIST format. */
      voc = SF_FORMAT_VOC, /* VOC files. */
      ircam = SF_FORMAT_IRCAM, /* Berkeley/IRCAM/CARL */
      w64 = SF_FORMAT_W64, /* Sonic Foundry's 64 bit RIFF/WAV */
      mat4 = SF_FORMAT_MAT4, /* Matlab (tm) V4.2 / GNU Octave 2.0 */
      mat5 = SF_FORMAT_MAT5, /* Matlab (tm) V5.0 / GNU Octave 2.1 */
      pvf = SF_FORMAT_PVF, /* Portable Voice Format */
      xi = SF_FORMAT_XI, /* Fasttracker 2 Extended Instrument */
      htk = SF_FORMAT_HTK, /* HMM Tool Kit format */
      sds = SF_FORMAT_SDS, /* Midi Sample Dump Standard */
      avr = SF_FORMAT_AVR, /* Audio Visual Research */
      wavex = SF_FORMAT_WAVEX, /* MS WAVE with WAVEFORMATEX */
      sd2 = SF_FORMAT_SD2, /* Sound Designer 2 */
      flac = SF_FORMAT_FLAC, /* FLAC lossless file format */
      caf = SF_FORMAT_CAF, /* Core Audio File format */
      wve = SF_FORMAT_WVE, /* Psion WVE format */
      ogg = SF_FORMAT_OGG, /* Xiph OGG container */
      mpc2k = SF_FORMAT_MPC2K, /* Akai MPC 2000 sampler */
      rf64 = SF_FORMAT_RF64 /* RF64 WAV file */
    };
    enum class subtype: int{
      pcm_s8 = SF_FORMAT_PCM_S8, /* Signed 8 bit data */
      pcm_16 = SF_FORMAT_PCM_16, /* Signed 16 bit data */
      pcm_24 = SF_FORMAT_PCM_24, /* Signed 24 bit data */
      pcm_32 = SF_FORMAT_PCM_32, /* Signed 32 bit data */

      pcm_u8 = SF_FORMAT_PCM_U8, /* Unsigned 8 bit data (WAV and RAW only) */

      _float = SF_FORMAT_FLOAT, /* 32 bit float data */
      _double = SF_FORMAT_DOUBLE, /* 64 bit float data */

      ulaw = SF_FORMAT_ULAW, /* U-Law encoded. */
      alaw = SF_FORMAT_ALAW, /* A-Law encoded. */
      ima_adpcm = SF_FORMAT_IMA_ADPCM, /* IMA ADPCM. */
      ms_adpcm = SF_FORMAT_MS_ADPCM, /* Microsoft ADPCM. */

      gsm610 = SF_FORMAT_GSM610, /* GSM 6.10 encoding. */
      vox_adpcm = SF_FORMAT_VOX_ADPCM, /* Oki Dialogic ADPCM encoding. */

      g721_32 = SF_FORMAT_G721_32, /* 32kbs G721 ADPCM encoding. */
      g723_24 = SF_FORMAT_G723_24, /* 24kbs G723 ADPCM encoding. */
      g723_40 = SF_FORMAT_G723_40, /* 40kbs G723 ADPCM encoding. */

      dwvw_12 = SF_FORMAT_DWVW_12, /* 12 bit Delta Width Variable Word encoding. */
      dwvw_16 = SF_FORMAT_DWVW_16, /* 16 bit Delta Width Variable Word encoding. */
      dwvw_24 = SF_FORMAT_DWVW_24, /* 24 bit Delta Width Variable Word encoding. */
      dwvw_n = SF_FORMAT_DWVW_N, /* N bit Delta Width Variable Word encoding. */

      dpcm_8 = SF_FORMAT_DPCM_8, /* 8 bit differential PCM (XI only) */
      dpcm_16 = SF_FORMAT_DPCM_16, /* 16 bit differential PCM (XI only) */

      vorbis = SF_FORMAT_VORBIS /* Xiph Vorbis encoding. */
    };
    enum class endianness: int{
      file = SF_ENDIAN_FILE, /* Default file endian-ness. */
      little = SF_ENDIAN_LITTLE, /* Force little endian-ness. */
      big = SF_ENDIAN_BIG, /* Force big endian-ness. */
      cpu = SF_ENDIAN_CPU /* Force CPU endian-ness. */
    };

    class file{
      typedef std::unique_ptr<SNDFILE, void(*)(SNDFILE*)> _ptr_t;
      _ptr_t ptr;
      SF_INFO info;
      file(_ptr_t&& p, SF_INFO&& i):
        ptr{std::move(p)},
        info{std::move(i)}
      {}
    public:

      static file read(const std::string& name){
        SF_INFO info{0};
        _ptr_t p{sf_open(name.c_str(), SFM_READ, &info), [](SNDFILE* p){
          sf_close(p);
        }};
        return file{std::move(p), std::move(info)};
      }

      auto format() const{
        return std::make_tuple(
          static_cast<type>(info.format & SF_FORMAT_TYPEMASK),
          static_cast<subtype>(info.format & SF_FORMAT_SUBMASK),
          static_cast<endianness>(info.format & SF_FORMAT_ENDMASK)
        );
      }
      frames_t size() const{
        return info.frames;
      }
      unsigned int channels() const{
        return info.channels;
      }

      struct _channel{
        file& _file;
        unsigned int number;
        frames_t read(buffer_view buffer){
          return _file.read(number, buffer);
        }
        operator sample(){
          std::vector<frame_t> tmp(_file.size());
          frames_t r = read(buffer_view{tmp});
          tmp.erase(tmp.begin()+r, tmp.end());
          return tmp;
        }
      };
      _channel channel(unsigned int const number){
        return _channel{*this, number};
      }

      frames_t read(unsigned int const channel_num, buffer_view buffer){
        assert(channel_num < channels());
        std::vector<frame_t> tmp(channels() * buffer.size());
        frames_t const count = sf_read_float(ptr.get(), tmp.data(), tmp.size());
        for(frames_t i = 0; i < buffer.size(); ++i){
          buffer[i] = tmp[channels()*i + channel_num];
        }
        return count/channels();
      }
    };
  }

  class param_store{
  public:
    typedef unsigned int index;
    class id{
      gc::gc_ptr<index> r;
    protected:
      id(param_store& p){
        index& tmp = p.next_id;
        r = gc::make_gc<index>(tmp++);
        while(p.ids_in_use.count(*this)){
          *r = tmp++;
        }
        p.ids_in_use.insert(*this);
      }
    public:
      bool operator<(const id& b) const{
        return *r < *b.r;
      }

      friend class param_store;
    };
  private:
    struct param_storage{
      virtual void assign(const param_storage&) = 0;
      virtual gc::gc_ptr<param_storage> clone(param_store&, id, index) const = 0;
      virtual ~param_storage(){}
    };
    template<typename T>
    struct param_impl: param_storage{
      param_impl(const T& t): data{t}{}
      param_impl(T&& t): data{std::move(t)}{}
      T data;
      void assign(const param_storage& other) override{
        param_impl<T> const * p = dynamic_cast<param_impl<T> const *>(&other);
        assert(p);
        data = p->data;
      }
      gc::gc_ptr<param_storage> clone(param_store& store, id owner, index name) const override{
        return store._ensure_param<T>(owner, name, [this]{return data;});
      }
    };

    std::map<std::pair<id,index>, gc::gc_ptr<param_storage>> store;
    std::set<id> ids_in_use;
    index next_id = 1;
    gc::leak_suppressor leak_suppressor;
  public:
    param_store():
      // let the gc collect store params that are no longer used
      leak_suppressor{gc::make_leak_suppressor([this](){
        {
          auto it = store.begin();
          while(it != store.end()){
            if(it->second.unique()){
              it = store.erase(it);
            }else{
              ++it;
            }
          }
        }
        {
          auto it = ids_in_use.begin();
          while(it != ids_in_use.end()){
            if(it->r.unique()){
              it = ids_in_use.erase(it);
            }else{
              ++it;
            }
          }
        }
      })}
    {}
    template<typename T>
    class param{
      gc::gc_ptr<param_impl<T>> impl;
    protected:
      param(gc::gc_ptr<param_impl<T>> impl): impl{impl}{}
    public:
      param(): impl{}{}
      T& operator*(){
        return impl->data;
      }
      const T& operator*() const{
        return impl->data;
      }
      T* operator->(){
        return &impl->data;
      }
      T const * operator->() const{
        return &impl->data;
      }

      friend class param_store;
      friend class generic_param;
    };

    class generic_param{
      gc::gc_ptr<param_storage> impl;
    public:
      template<typename T>
      generic_param(const param<T>& p): impl{p.impl}{}
      template<typename T>
      operator param<T>(){
        auto i = impl.dynamic_pointer_cast<param_impl<T>>();
        assert(i);
        return param<T>{i};
      }
      void assign(const generic_param& p){
        impl->assign(*p.impl);
      }
      generic_param clone(param_store& store, id owner, index name){
        return impl->clone(store, owner, name);
      }
    protected:
      generic_param(const gc::gc_ptr<param_storage>& ptr): impl{ptr}{}
    };

    template<typename T, typename Name, typename F>
    typename std::enable_if<std::is_convertible<decltype(std::declval<F>()()), T>::value, param<T>>::type
    ensure_param(id owner, Name name, F fn){
      return param<T>{_ensure_param<T>(owner, static_cast<index>(name), fn)};
    }

    template<typename T, typename Name>
    param<T> ensure_param(id owner, Name name, const T& t = T{}){
      return ensure_param<T>(owner, name, [&]{
        return t;
      });
    }

    template<typename T, typename Name>
    param<T> find_param(id owner, Name name){
      auto p = std::make_pair(owner, static_cast<index>(name));
      assert(store.count(p));
      auto ptr = store[p].dynamic_pointer_cast<param_impl<T>>();
      assert(ptr);
      return param<T>{ptr};
    }

    id make_id(){
      return id{*this};
    }

    class param_set{
      std::map<index, generic_param> params;
      id owner;
      param_store& store;
    public:
      using iterator = decltype(params)::iterator;

      param_set(param_store& store, id owner):
        owner{owner},
        store{store}
      {}

      id id(){
        return owner;
      }
      template<typename T, typename Index, typename Val>
      param<T> add_param(Index name, Val val){
        auto tmp = store.ensure_param<T>(owner, static_cast<index>(name), val);
        params.insert(std::make_pair(static_cast<index>(name), tmp));
        return tmp;
      }
      void add_gen_param(index name, generic_param p){
        params.insert(std::make_pair(name, p.clone(store, owner, name)));
      }
      iterator begin(){
        return params.begin();
      }
      iterator end(){
        return params.end();
      }
      param_set clone(param_store& store, param_store::id owner){
        param_set other{store, owner};
        for(auto const& p : params){
          other.add_gen_param(p.first, p.second);
        }
        return other;
      }
      generic_param& at(index name){
        return params.at(name);
      }
      generic_param const & at(index name) const{
        return params.at(name);
      }
      void assign(const param_set& s){
        for(auto& p: params){
          p.second.assign(s.at(p.first));
        }
      }
    };

    template<typename T, typename F>
    gc::gc_ptr<param_impl<T>> _ensure_param(id owner, index name, F fn){
      auto p = std::make_pair(owner, static_cast<index>(name));
      if(store.count(p)){
        auto ptr = store[p].dynamic_pointer_cast<param_impl<T>>();
        assert(ptr);
        return ptr;
      }else{
        auto ptr = gc::make_gc<param_impl<T>>(fn());
        store[p] = ptr;
        return ptr;
      }
    }
  };

  class sample_source{
  public:
    struct impl{
      virtual frames_t generate(buffer_view, frames_t const) const = 0;
      virtual frames_t available_frames() const = 0;
      virtual void advance(frames_t const) = 0;
      virtual bool uniform() const = 0;
      virtual ~impl(){}
    };
    template<typename Impl, typename = typename std::enable_if<std::is_base_of<impl, Impl>::value, void>::type>
    sample_source(Impl&& impl): _impl{std::make_unique<Impl>(std::move(impl))}
    {}
    frames_t generate(buffer_view buffer, frames_t const frames) const{
      return _impl->generate(buffer, frames);
    }
    frames_t available_frames() const{
      return _impl->available_frames();
    }
    void advance(frames_t const frames){
      _impl->advance(frames);
    }
    bool uniform() const{
      return _impl->uniform();
    }

    friend class sample_source_p;
  protected:
    std::unique_ptr<impl> _impl;
    sample_source(std::unique_ptr<impl>&& ptr): _impl{std::move(ptr)}{}
  };

  class sample_source_p{
  public:
    struct impl: sample_source::impl{
      impl(param_store& store, param_store::id owner): params{store, owner}{}
      param_store::param_set& param_set(){
        return params;
      }
    private:
      param_store::param_set params;
    };

    template<typename Impl, typename = typename std::enable_if<std::is_base_of<impl, Impl>::value, void>::type>
    sample_source_p(Impl&& impl): _impl{std::make_unique<Impl>(std::move(impl))}
    {}
    sample_source_p(sample_source&& src){
      // automatically upcast from sample_source
      impl* p = dynamic_cast<impl*>(src._impl.get());
      assert(p);
      src._impl.release();
      _impl.reset(p);
    }
    operator sample_source() &&{
      return sample_source{std::move(_impl)};
    }

    frames_t generate(buffer_view buffer, frames_t const frames) const{
      return _impl->generate(buffer, frames);
    }
    frames_t available_frames() const{
      return _impl->available_frames();
    }
    void advance(frames_t const frames){
      _impl->advance(frames);
    }
    bool uniform() const{
      return _impl->uniform();
    }
    param_store::param_set const & param_set() const{
      return _impl->param_set();
    }
  protected:
    std::unique_ptr<impl> _impl;
  };

  class multichannel_source{
  public:
    struct impl{
      virtual frames_t generate(buffer_view_list, frames_t const) const = 0;
      virtual frames_t available_frames() const = 0;
      virtual void advance(frames_t const) = 0;
      virtual std::size_t channels() const = 0;
      virtual ~impl(){}
    };
    multichannel_source() = delete;
    template<typename Impl, typename = typename std::enable_if<std::is_base_of<impl, Impl>::value, void>::type>
    multichannel_source(Impl&& impl): _impl{std::make_unique<Impl>(std::move(impl))}
    {}
    frames_t generate(buffer_view_list lst, frames_t const frames) const{
      return _impl->generate(lst, frames);
    }
    frames_t available_frames() const{
      return _impl->available_frames();
    }
    void advance(frames_t const frames){
      _impl->advance(frames);
    }
    std::size_t channels() const{
      return _impl->channels();
    }
  private:
    std::unique_ptr<impl> _impl;
  };

  struct _generic_multichannel_impl: multichannel_source::impl{
    std::vector<sample_source> sources;
    std::vector<frames_t> frames_generated;

    _generic_multichannel_impl(std::vector<sample_source>&& chs):
      sources{std::move(chs)}
    {
      frames_generated.assign(sources.size(), 0);
    }
    frames_t generate(buffer_view_list lst, frames_t const frames) const override{
      assert(lst.size() == sources.size() && lst.size() == frames_generated.size());
      auto& fgenerated = const_cast<std::vector<frames_t>&>(frames_generated);
      std::transform(
        sources.begin(), sources.end(),
        lst.cbegin(),
        fgenerated.begin(),
        [frames](sample_source const& src, buffer_view buffer){
          return src.generate(buffer, frames);
        });
      auto it = std::min_element(fgenerated.begin(), fgenerated.end());
      return it == fgenerated.end() ? 0 : *it;
    }
    frames_t available_frames() const override{
      auto& fgenerated = const_cast<std::vector<frames_t>&>(frames_generated);
      std::transform(
        sources.cbegin(), sources.cend(),
        fgenerated.begin(),
        [](const sample_source& src){
          return src.available_frames();
        });
      auto it = std::min_element(fgenerated.begin(), fgenerated.end());
      return it == fgenerated.end() ? 0 : *it;
    }
    void advance(frames_t const count) override{
      for(auto& src: sources){
        src.advance(count);
      }
    }
    std::size_t channels() const override{
      return sources.size();
    }
  };

  multichannel_source multichannel(vector_rvalue_interface<sample_source>&& sources){
    return _generic_multichannel_impl{std::move(sources)};
  }

  sample_source _buffer_proxy(buffer_t& buffer, frames_t& available){
    struct _proxy: sample_source::impl{
      buffer_t& buffer;
      frames_t& available;
      _proxy(buffer_t& buffer, frames_t& available):
        buffer{buffer},
        available{available}
      {}
      frames_t generate(buffer_view bview, frames_t const frames) const override{
        frames_t const to_generate = std::min({frames, available, buffer.size()});
        std::copy_n(buffer.begin(), to_generate, bview.begin());
        return to_generate;
      }
      frames_t available_frames() const override{
        return std::min(available, buffer.size());
      }
      void advance(frames_t const count) override{}
      bool uniform() const override{
        return true;
      }
    };
    return _proxy{buffer, available};
  }

  template<bool check_uniform = true>
  class multichannel_insert: public _generic_multichannel_impl{
    multichannel_source msrc;
    std::vector<buffer_t> buffers;
    frames_t buffers_availability;
  public:
    template<typename Fn>
    multichannel_insert(multichannel_source&& _msrc, Fn fn):
      msrc{std::move(_msrc)},
      buffers_availability{0},
      _generic_multichannel_impl{std::vector<sample_source>{}}
    {
      buffers.assign(msrc.channels(), buffer_t{});

      std::vector<sample_source> proxy_srcs;
      proxy_srcs.reserve(msrc.channels());
      std::transform(
        buffers.begin(), buffers.end(),
        std::back_inserter(proxy_srcs),
        [this](auto& buffer){
          return _buffer_proxy(buffer, buffers_availability);
        });

      auto tmp_sources = fn(std::move(proxy_srcs));
      std::move(tmp_sources.begin(), tmp_sources.end(), std::back_inserter(sources));
      frames_generated.assign(sources.size(), 0);

      if(check_uniform){
        assert(std::all_of(
          sources.begin(), sources.end(),
          [](sample_source& src){
            return src.uniform();
          }));
      }
    }
    frames_t generate(buffer_view_list lst, frames_t const frames) const override{
      frames_t const actual_frames = msrc.generate(
        buffer_view_list{const_cast<std::vector<buffer_t>&>(buffers)},
        frames);
      const_cast<frames_t&>(buffers_availability) = actual_frames;
      return _generic_multichannel_impl::generate(lst, actual_frames);
    }
    frames_t available_frames() const override {
      const_cast<frames_t&>(buffers_availability) = msrc.available_frames();
      return _generic_multichannel_impl::available_frames();
    }
    void advance(frames_t const count) override{
      msrc.advance(count);
      _generic_multichannel_impl::advance(count);
    }
  };

  /*template<typename Fn>
  multichannel_source multichannel_insert(multichannel_source&& msrc, Fn fn){
    struct _insert: _generic_multichannel_impl{
      multichannel_source msrc;
      std::vector<buffer_t> buffers;
      frames_t buffers_availability;

      _insert(param_store& controller, multichannel_source&& msrc, Fn fn):
        msrc{std::move(msrc)},
        buffers_availability{0},
        _generic_multichannel_impl{std::vector<sample_source>{}}
      {
        buffers.assign(channels(), buffer_t{});

        std::vector<sample_source> proxy_srcs;
        proxy_srcs.reserve(msrc.channels());
        std::transform(
          buffers.begin(), buffers.end(),
          std::back_inserter(proxy_srcs),
          [](auto& pair){
            return _buffer_proxy(pair.first, pair.second);
          });
        sources = fn(std::move(proxy_srcs));

        assert(std::all_of(
          sources.begin(), sources.end(),
          [](sample_source& src){
            return src.uniform();
          }));
      }
      frames_t generate(buffer_view_list lst, frames_t const frames) const override{
        frames_t const actual_frames = msrc.generate(
          buffer_view_list{buffers},
          frames);
        const_cast<frames_t&>(buffers_availability) = actual_frames;
        return _generic_multichannel_impl::generate(lst, actual_frames);
      }
      frames_t available_frames() const{
        const_cast<frames_t&>(buffers_availability) = msrc.available_frames();
        return _generic_multichannel_impl::available_frames();
      }
      void advance(frames_t const count) override{
        msrc.advance(count);
        _generic_multichannel_impl::advance(count);
      }
    };
    return _insert{std::move(msrc), fn};
  }*/

  class controller: public multichannel_source::impl{
    param_store::param_set master_params;
    std::vector<param_store::param_set> slave_params;
    multichannel_source msrc;

    void update(){
      for(auto& p: slave_params){
        p.assign(master_params);
      }
    }
  public:
    controller(param_store& store, param_store::id name, multichannel_source&& msrc, std::vector<param_store::param_set>&& slave_params):
      master_params{
        (assert(slave_params.size() >= 1), slave_params.front().clone(store, name))},
      slave_params{std::move(slave_params)},
      msrc{std::move(msrc)}
    {}

    frames_t generate(buffer_view_list buffers, frames_t const frames) const override{
      const_cast<controller&>(*this).update();
      return msrc.generate(buffers, frames);
    }
    frames_t available_frames() const override{
      const_cast<controller&>(*this).update();
      return msrc.available_frames();
    }
    void advance(frames_t count) override{
      update();
      msrc.advance(count);
    }
    std::size_t channels() const override{
      return msrc.channels();
    }
  };

  template<typename Fn>
  multichannel_source make_controlled_set(param_store& store, param_store::id name, multichannel_source&& msrc, Fn fn){
    std::vector<param_store::param_set> slaves;
    multichannel_source result = multichannel_insert<false>{std::move(msrc), [&fn, &slaves](std::vector<sample_source>&& srcs){
      std::vector<sample_source_p> result = fn(std::move(srcs));
      slaves.reserve(result.size());
      std::transform(
        result.begin(), result.end(),
        std::back_inserter(slaves),
        [](const sample_source_p& src){
          return src.param_set();
        });
      return std::move(result);
    }};
    return controller{store, name, std::move(result), std::move(slaves)};
  }

  sample_source mixer(std::vector<sample_source>&& _sources){
    struct _mixer: sample_source::impl{
      std::vector<sample_source> sources;
      _mixer(std::vector<sample_source>&& _sources):
        sources{std::move(_sources)}
      {}

      frames_t generate(buffer_view buffer, frames_t const frames) const override{
        std::array<frame_t, BUFFER_SIZE> tmp;
        frames_t used = std::min(available_frames(), frames);
        std::fill_n(buffer.begin(), used, 0);
        for(auto&& src: sources){
          src.generate(buffer_view{tmp}, used);
          std::transform(
            buffer.begin(), buffer.begin() + used,
            tmp.begin(),
            [](frame_t a, frame_t b){
              return a+b;
            });
        }
        return used;
      }
      frames_t available_frames() const override{
        auto it = std::min_element(sources.begin(), sources.end(), [](const sample_source& a, const sample_source& b){
          return a.available_frames() < b.available_frames();
        });
        return std::min(it != sources.end() ? it->available_frames() : std::numeric_limits<frames_t>::max(), BUFFER_SIZE);
      }
      void advance(frames_t const frames) override{
        for(auto& src: sources){
          src.advance(frames);
        }
      }
      bool uniform() const override{
        return true;
      }
    };
    return _mixer{std::move(_sources)};
  }

  class sampler: public sample_source_p::impl{
    param_store::param<frames_t> _offset;
    param_store::param<frames_t> _length;
    param_store::param<sample> _sample;
  public:
    enum class param: param_store::index{
      offset,
      sample,
      length
    };

  public:
    sampler(param_store& controller, param_store::id name):
      sample_source_p::impl{controller, name}
    {
      auto& ps = param_set();
      _offset = ps.add_param<frames_t>(param::offset, 0);
      _length = ps.add_param<frames_t>(param::length, 0);
      _sample = ps.add_param<sample>(param::sample, [](){return sample{};});
    }
    frames_t generate(buffer_view buffer, frames_t const frames) const override{
      frames_t const& length = *_length;
      sample const & s = *_sample;
      frames_t const & offset = *_offset;

      frames_t frames_used = std::min({frames, length, s.size()-offset});
      std::copy_n(s.cbegin() + offset, frames_used, buffer.begin());
      std::fill_n(buffer.begin() + frames_used, frames - frames_used, 0);
      return frames;
    }
    frames_t available_frames() const override{
      return std::numeric_limits<frames_t>::max();
    }
    void advance(frames_t const count) override{
      *_length = count > *_length ? 0 : *_length - count;
      *_offset = *_length > 0 ? *_offset + count : 0;
    }
    bool uniform() const override{
      return true;
    }
  };

  namespace samplerate{
    class state{
      static void _delete(SRC_STATE* ptr){
        if(ptr){
          src_delete(ptr);
        }
      }
      int err;
      std::unique_ptr<SRC_STATE, void(*)(SRC_STATE*)> s;

      struct cbdata_record{
        long(*callback)(void*,float*&);
        void* cb_data;
      };
      std::unique_ptr<cbdata_record> cbdata;
    public:
      state(int converter_type = SRC_SINC_FASTEST):
        s{nullptr, _delete},
        cbdata{new cbdata_record{nullptr, nullptr}}
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
            1,
            &err,
            cbdata.get()),
          _delete
        };
      }

      template<typename Fn>
      frames_t read(double ratio, frames_t frames, buffer_view buffer, Fn cb){
        cbdata->callback = [](void* ptr, float*& data) -> long{
          Fn& cb = *reinterpret_cast<Fn*>(ptr);
          return cb(data);
        };
        cbdata->cb_data = &cb;
        frames_t frames_read =  src_callback_read(s.get(), ratio, std::min(buffer.size(), frames), buffer.data());
        cbdata->callback = nullptr;
        cbdata->cb_data = nullptr;
        return frames_read;
      }
    };
  }

  class resampler: public sample_source_p::impl{
    sample_source src;
    enum class private_param: param_store::index{
      input_buffer = 1,
      output_buffer,
      output_frames_available,
      output_buffer_start,
      state
    };
    param_store::param<buffer_t> input_buffer;
    param_store::param<buffer_t> output_buffer;
    param_store::param<frames_t> output_frames_available;
    param_store::param<frames_t> output_buffer_start;
    param_store::param<std::shared_ptr<samplerate::state>> state;

    param_store::param<double> ratio;

    static long _buffer_callback(resampler* self, frame_t*& data){
      buffer_t& b = *(self->input_buffer);
      frames_t generated = self->src.generate(
        buffer_view{b},
        b.size());
      self->src.advance(generated);
      return generated;
    }
    auto _get_buffer_callback(){
      return [this](float*& data) -> long{
        frames_t generated = src.generate(
          buffer_view{*input_buffer},
          input_buffer->size());
        src.advance(generated);
        data = input_buffer->data();
        return generated;
      };
    }
  public:
    enum class param: param_store::index{
      ratio = 0
    };
    resampler(param_store& controller, param_store::id name, sample_source&& src):
      sample_source_p::impl{controller, name},
      src{std::move(src)},
      input_buffer{controller.ensure_param<buffer_t>(name, private_param::input_buffer)},
      output_buffer{controller.ensure_param<buffer_t>(name, private_param::output_buffer)},
      output_frames_available{controller.ensure_param<frames_t>(name, private_param::output_frames_available, 0)},
      output_buffer_start{controller.ensure_param<frames_t>(name, private_param::output_buffer_start, 0)},
      state{controller.ensure_param<std::shared_ptr<samplerate::state>>(name, private_param::state, []{
        return std::make_shared<samplerate::state>();
      })}
    {
      ratio = param_set().add_param<double>(param::ratio, 1);
    }

      frames_t generate(buffer_view buffer, frames_t const frames) const override{
        if(*output_frames_available == 0){
          // semantically generate still acts like it's const - results should never change
          // for the same invocation without advancing the sample_source.
          // that said, generate does modify state, and so the constness we must fudge
          frames_t samples_produced = const_cast<samplerate::state&>(**state).read(
            *ratio, frames,
            buffer_view{const_cast<frame_t*>(output_buffer->data()), output_buffer->size()},
            const_cast<resampler*>(this)->_get_buffer_callback());
          const_cast<frames_t&>(*output_buffer_start) = 0;
          const_cast<frames_t&>(*output_frames_available) = samples_produced;
        }
        frames_t generated = std::min(frames, *output_frames_available);
        std::copy_n(output_buffer->cbegin() + *output_buffer_start, generated, buffer.begin());
        return generated;
      }
      frames_t available_frames() const override{
        if(*output_frames_available == 0){
          return std::min<frames_t>({
            static_cast<frames_t>(src.available_frames() * (*ratio)),
            output_buffer->size(),
            1});
        }else{
          return *output_frames_available;
        }
      }
      void advance(frames_t const count) override{
        if(count > *output_frames_available){
          frames_t frames_to_skip = count - *output_frames_available;
          for(frames_t skipped = 0; skipped < frames_to_skip;){
            skipped += (*state)->read(
              *ratio,
              frames_to_skip - skipped,
              *output_buffer,
              _get_buffer_callback());
          }
          *output_frames_available = 0;
          *output_buffer_start = 0;
        }else{
          *output_frames_available -= count;
          *output_buffer_start += count;
        }
      }
      bool uniform() const override{
        return false;
      }
  };

  multichannel_source make_resampler_set(param_store& store, param_store::id name, multichannel_source&& msrc, const std::vector<param_store::id>& ids){
    return make_controlled_set(store, name, std::move(msrc), [&ids, &store](std::vector<sample_source>&& srcs){
      std::vector<sample_source_p> result;
      result.reserve(srcs.size());
      std::transform(
        srcs.begin(), srcs.end(),
        ids.begin(),
        std::back_inserter(result),
        [&store](sample_source& src, param_store::id id){
          return resampler{store, id, std::move(src)};
        });
      return std::move(result);
    });
  }

  class event{
    struct _impl_base{
      frames_t const offset;
      std::pair<param_store::id, param_store::id> const name;
      _impl_base(frames_t offset, std::pair<param_store::id, param_store::id> name):
        offset{offset},
        name{name}
      {}
      virtual void apply(param_store&) const = 0;
      virtual ~_impl_base(){}
    };
    template<typename T>
    struct _impl: _impl_base{
      T const value;
      void apply(param_store& ps) const override{
        auto p = ps.find_param<T>(name.first, name.second);
        *p = value;
      }
      _impl(frames_t offset, std::pair<param_store::id, param_store::id> name, const T& value):
        _impl_base{offset, name},
        value{value}
      {}
    };
    std::unique_ptr<_impl_base> impl;
  public:
    event(): impl{nullptr}{}
    template<typename T>
    event(frames_t offset, std::pair<param_store::id, param_store::id> name, const T& value):
      impl{std::make_unique<_impl<T>>(offset, name, value)}
    {}
    frames_t offset() const{
      assert(impl);
      return impl->offset;
    }
    void apply(param_store& ps) const{
      assert(impl);
      impl->apply(ps);
    }
  };

  class sequencer: sample_source_p::impl{
    param_store& controller;
    sample_source src;
  public:
    enum class param: param_store::index{
      position,
      events
    };
    typedef gc::gc_ptr<const std::vector<event>> events_t;

    sequencer(param_store& controller, param_store::id name, sample_source&& src):
      sample_source_p::impl{controller, name},
      controller{controller},
      src{std::move(src)}
    {
      _position = param_set().add_param<frames_t>(param::position, 0);
      _events = param_set().add_param<events_t>(param::events, events_t{});
    }

    frames_t generate(buffer_view buffer, frames_t const frames) const override{
      return src.generate(buffer, std::min(frames, available_frames()));
    }

    frames_t available_frames() const override{
      auto& events_p = *_events;
      if(events_p){
        auto& events = *events_p;
        auto it = std::upper_bound(events.cbegin(), events.cend(), *_position, [](const frames_t& a, const event& b){
          return a < b.offset();
        });
        if(it == events.cend()){
          return src.available_frames();
        }else{
          return std::min(it->offset() - *_position, src.available_frames());
        }
      }else{
        return src.available_frames();
      }
    }

    void advance(frames_t const frames) override{
      auto& events_p = *_events;
      if(events_p){
        auto& events = *events_p;
        auto it = std::upper_bound(events.begin(), events.end(), *_position, [](const frames_t& a, const event& b){
          return a < b.offset();
        });
        frames_t last_pos = *_position;
        for(; it != events.end() && it->offset() <= (*_position + frames); ++it){
          it->apply(controller);
          if(it->offset() > last_pos){
            src.advance(it->offset() - last_pos);
            last_pos = it->offset();
          }
        }
      }
      *_position += frames;
    }
    bool uniform() const override{
      return true;
    }
  private:
    param_store::param<frames_t> _position;
    param_store::param<events_t> _events;
  };

  /*sample_source sequencer(param_store& controller, param_store::id name, sample_source&& src){
    struct _sequencer: sample_source::impl{
      param_store& controller;
      sample_source src;

      enum class private_param: param_store::id{
        position,
        events
      };
      typedef gc::gc_ptr<const std::vector<event>> events_t;

      param_store::param<frames_t> position;
      param_store::param<events_t> _events;

      _sequencer(param_store& controller, param_store::id name, sample_source&& src):
        controller{controller},
        src{std::move(src)},
        position{controller.ensure_param<frames_t>(name, private_param::position, 0)},
        _events{controller.ensure_param<events_t>(name, private_param::events, [](){
          return events_t{};
        })}
      {}

      frames_t generate(buffer_view buffer, frames_t const frames) const override{
        return src.generate(buffer, std::min(frames, available_frames()));
      }

      frames_t available_frames() const override{
        auto& events = **_events;
        auto it = std::upper_bound(events.cbegin(), events.cend(), *position, [](const frames_t& a, const event& b){
          return a < b.offset();
        });
        if(it == events.cend()){
          return src.available_frames();
        }else{
          return std::min(it->offset() - *position, src.available_frames());
        }
      }

      void advance(frames_t const frames) override{
        auto& events = **_events;
        auto it = std::upper_bound(events.begin(), events.end(), *position, [](const frames_t& a, const event& b){
          return a < b.offset();
        });
        frames_t last_pos = *position;
        for(; it != events.end() && it->offset() <= (*position + frames); ++it){
          it->apply(controller);
          if(it->offset() > last_pos){
            src.advance(it->offset() - last_pos);
            last_pos = it->offset();
          }
        }
        *position += frames;
      }
      bool uniform() const override{
        return true;
      }
    };
    return _sequencer{controller, name, std::move(src)};
  }*/

  class event_dispatcher{
    nbqueue<event> events_buffer;
    param_store& controller;
  public:
    event_dispatcher(param_store& controller):
      controller{controller}
    {}
    void post_event(event&& e){
      events_buffer.put(std::move(e));
    }
    void execute_events(){
      while(events_buffer.get([this](event& e){
        e.apply(controller);
      }));
    }
  };

  namespace jack{
    class processor_t{
      struct base{
        virtual void call(frames_t) = 0;
        virtual ~base(){}
      };
      template<typename Fn>
      struct proc: base{
        Fn fn;
        void call(frames_t frames) override{
          fn(frames);
        }
        proc(Fn&& fn): fn{std::move(fn)}{}
      };
      std::unique_ptr<base> ptr;
    public:
      template<typename Fn, typename = typename std::enable_if<!std::is_same<Fn,processor_t>::value,void>::type>
      processor_t(Fn&& fn): ptr{std::make_unique<proc<Fn>>(std::move(fn))}{}
      processor_t(processor_t&&) = default;
      processor_t(const processor_t&) = delete;
      void operator()(frames_t frames){
        ptr->call(frames);
      }
    };

    class client{
      event_dispatcher& dispatcher;
      std::unordered_map<unsigned int, processor_t> processors;

      struct tag_manager{
        client& c;
        unsigned int tag;
        tag_manager(const tag_manager&) = delete;
        tag_manager(client& c, unsigned int tag): c{c}, tag{tag}{}
        tag_manager(tag_manager&& t):
          c{t.c}
        {
          tag = t.tag;
          t.tag = 0;
        }
        tag_manager& operator=(tag_manager&& t){
          assert(&t.c == &c);
          if(tag != 0){
            c.unset_processor(tag);
          }
          tag = t.tag;
          t.tag = 0;
          return *this;
        }
        tag_manager& operator=(const tag_manager&) = delete;
        ~tag_manager(){
          if(tag != 0){
            c.unset_processor(tag);
          }
        }
      };

    public:
      class port{
        std::shared_ptr<jack_port_t> _port;
        client& _client;
        tag_manager tagman;
      protected:
        port(client& _client, unsigned int tag, const std::string& name):
          _port{
            jack_port_register(_client._client.get(), name.c_str(), JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0),
            [&_client](jack_port_t* p){
              jack_port_unregister(_client._client.get(), p);
            }
          },
          _client{_client},
          tagman{_client, tag}
        {
          assert(_port);
          std::cout << "register port" << std::endl;
        }
        static buffer_view get_buffer(const std::shared_ptr<jack_port_t>& p, frames_t frames){
          frame_t* data = reinterpret_cast<frame_t*>(jack_port_get_buffer(p.get(), frames));
          return buffer_view{data, frames};
        }
        processor_t make_processor(sample_source&& _src){
          return [
              this,
              src = std::move(_src)]
            (frames_t frames) mutable{
              buffer_view buffer = get_buffer(frames);
              frames_t frames_generated = 0;
              while(frames_generated < frames){
                frames_t current_frames = src.generate(
                  buffer.subdivide(
                    frames_generated,
                    frames - frames_generated),
                  frames-frames_generated);
                src.advance(current_frames);
                frames_generated += current_frames;
              }
            };
        }
      public:
        buffer_view get_buffer(frames_t frames){
          return get_buffer(_port, frames);
        }
        void set_src(sample_source&& src){
          auto p = make_processor(std::move(src));
          _client.set_processor(tagman.tag, std::move(p));
        }

        friend class client;
      };
      class port_set{
        std::vector<port> ports;
        tag_manager tagman;
        client& _client;

        processor_t make_processor(multichannel_source&& msrc){
          assert(msrc.channels() == ports.size());
          return [
              this,
              msrc = std::move(msrc),
              buffer_views = std::vector<buffer_view>(ports.size())]
            (frames_t frames) mutable{
              std::transform(
                ports.begin(), ports.end(),
                buffer_views.begin(),
                [frames](port& p){
                  return p.get_buffer(frames);
                });
              frames_t frames_so_far = 0;
              while(frames_so_far < frames){
                frames_t generated = msrc.generate(
                  buffer_view_list{buffer_views},
                  frames - frames_so_far);
                msrc.advance(generated);
                frames_so_far += generated;
              }
            };
        }
      public:
        port_set(client& c, unsigned int tag, std::vector<port>&& ports):
          ports{std::move(ports)},
          _client{c},
          tagman{c, tag}
        {}
        void set_src(multichannel_source&& msrc){
          auto p = make_processor(std::move(msrc));
          _client.set_processor(tagman.tag, std::move(p));
        }
        friend class client;
      };

      client(const std::string& name, event_dispatcher& dispatcher):
        dispatcher{dispatcher}
      {
        _client = std::shared_ptr<jack_client_t>(
          jack_client_open(name.c_str(), JackNullOption, nullptr),
          [](jack_client_t* p){
            jack_client_close(p);
          });
        assert(_client);
        assert(!jack_set_process_callback(_client.get(), [](jack_nframes_t frames, void* _self){
          client& self = *reinterpret_cast<client*>(_self);
          self.dispatcher.execute_events();
          for(auto& p: self.processors){
            p.second(frames);
          }
          return 0;
        }, this));
      }

      client(const client&) = delete;

      void activate(){
        assert(!jack_activate(_client.get()));
      }

      port register_port(const std::string& name, sample_source&& src){
        port p = register_port(name);
        p.set_src(std::move(src));
        return p;
      }
      port_set register_port_set(std::size_t channels, const std::string& name, multichannel_source&& msrc){
        std::vector<port> ports;
        assert(channels == msrc.channels());
        ports.reserve(channels);
        for(std::size_t i = 0; i < channels; ++i){
          ports.push_back(
            register_port(name + "_" + std::to_string(i)));
        }
        port_set ps{*this, make_processor_tag(), std::move(ports)};
        ps.set_src(std::move(msrc));
        return ps;
      }

      friend class port;
      friend class port_set;
    protected:
      std::shared_ptr<jack_client_t> _client;

      port register_port(const std::string& name){
        return {*this, make_processor_tag(), name};
      }
      void set_processor(unsigned int tag, processor_t&& proc){
        processors.insert(std::make_pair(tag, std::move(proc)));
      }
      void unset_processor(unsigned int tag){
        processors.erase(tag);
      }
      unsigned int make_processor_tag(){
        static unsigned int t = 1;
        unsigned int tmp = ++t;
        while(processors.count(tmp) > 0 && tmp != 0){
          tmp = ++t;
        }
        return tmp;
      }
    };

    using port = client::port;
    using port_set = client::port_set;
  }

}
