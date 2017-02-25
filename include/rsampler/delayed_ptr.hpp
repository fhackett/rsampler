#pragma once

#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <unordered_set>
#include <chrono>
#include <iostream>


namespace rsampler{
  namespace delayed{
    template<typename T>
    class delayed_ptr;
    template<typename T>
    class atomic_delayed_ptr;

    namespace{
      struct control_block_t{
        std::atomic<long> count{1};
        virtual ~control_block_t() = default;
      };
      template<typename T, typename Deleter>
      struct control_block_impl_t: control_block_t{
        std::unique_ptr<T,Deleter> ptr;
        control_block_impl_t(T* t, Deleter deleter):
          ptr{t,deleter}
        {}
      };
      template<typename T>
      struct control_block_inline_impl_t: control_block_t{
        T data;
        control_block_inline_impl_t(T&& t):
          data{std::move(t)}
        {}
      };

      class gc{
        std::atomic<unsigned int> _gc_count{0};
        bool _exit{false};
        std::mutex _delayed_mutex;
        std::unordered_set<std::unique_ptr<control_block_t>> _delayed_store;

        std::thread _delayed_deleter{[this]{
          using namespace std::chrono_literals;
          while(!_exit){
            std::cout << "Wake up gc\n";
            while(_gc_count != 0){
              std::unique_lock<std::mutex> guard{_delayed_mutex};
              auto it = _delayed_store.begin();
              while(it != _delayed_store.end()){
                std::cout << "Count: " << (*it)->count << " gc_count: " << _gc_count << "\n";
                if(!(*it)->count){
                  it = _delayed_store.erase(it);
                  --_gc_count;
                }else{
                  ++it;
                }
              }
            }
            std::this_thread::sleep_for(10ms);
          }
        }};

      public:
        void request_gc_cycle(){
          ++_gc_count;
        }
        void reset_control_block(control_block_t* control_block){
          if(control_block){
            auto result = control_block->count.fetch_sub(1);
            if(result == 1){
              request_gc_cycle();
            }
          }
        }
        control_block_t* register_control_block(control_block_t* control_block){
          std::lock_guard<std::mutex> guard{_delayed_mutex};
          _delayed_store.insert(std::unique_ptr<control_block_t>{control_block});
          return control_block;
        }
        ~gc(){
          {
            std::unique_lock<std::mutex> guard{_delayed_mutex};
            _exit = true;
          }
          _delayed_deleter.join();
        }
      } _gc;
    }

    template<typename T>
    class delayed_ptr{
    protected:
      control_block_t* control_block;
      T* observer;
      template<typename Y>
      void copy_from(const delayed_ptr<Y>& other){
        control_block = other.control_block;
        observer = other.observer;
        if(control_block){
          ++control_block->count;
        }
      }
      template<typename Y>
      void move_from(delayed_ptr<Y>&& other){
        control_block = other.control_block;
        observer = other.observer;
        other.observer = nullptr;
        other.control_block = nullptr;
      }
      delayed_ptr(T* o, control_block_t* c):
        observer{o},
        control_block{c}
      {}
    public:
      friend class atomic_delayed_ptr<T>;

      delayed_ptr():
        control_block{nullptr},
        observer{nullptr}
      {}
      delayed_ptr(std::nullptr_t):
        control_block{nullptr},
        observer{nullptr}
      {}
      delayed_ptr(delayed_ptr<T> const& other){
        copy_from(other);
      }
      delayed_ptr(delayed_ptr<T>&& other){
        move_from(std::move(other));
      }
      template<typename Y>
      delayed_ptr(delayed_ptr<Y>&& other){
        move_from(std::move(other));
      }
      template<typename Y>
      delayed_ptr(const delayed_ptr<Y>& other){
        copy_from(other);
      }
      void reset(){
        _gc.reset_control_block(control_block);
        control_block = nullptr;
        observer = nullptr;
      }
      template<typename Y, typename Deleter = std::default_delete<Y>>
      void reset(Y* ptr, Deleter d = Deleter()){
        reset();
        control_block = _gc.register_control_block(
          new control_block_impl_t<Y,Deleter>{ptr,d});
        observer = ptr;
      }
      template<typename Y>
      void reset(Y&& y){
        reset();
        auto tmp = new control_block_inline_impl_t<Y>{std::move(y)};
        control_block = _gc.register_control_block(tmp);
        observer = &tmp->data;
      }
      operator bool() const{
        return control_block;
      }
      const T* get() const{
        return observer;
      }
      T* get(){
        return observer;
      }
      const T* operator->() const{
        return observer;
      }
      T* operator->(){
        return observer;
      }
      T& operator*(){
        return *observer;
      }
      const T& operator*() const{
        return *observer;
      }
      long use_count() const{
        return control_block->count;
      }
      void swap(delayed_ptr<T>& other){
        T* tmpo = observer;
        control_block_t* tmpc = control_block;
        observer = other.observer;
        control_block = other.control_block;
        other.observer = tmpo;
        other.control_block = tmpc;
      }
      template<typename Y>
      delayed_ptr<T>& operator=(const delayed_ptr<Y>& other){
        reset();
        copy_from(other);
        return *this;
      }
      delayed_ptr<T>& operator=(delayed_ptr<T> const& other){
        reset();
        copy_from(other);
        return *this;
      }
      delayed_ptr<T>& operator=(delayed_ptr<T>&& other){
        reset();
        move_from(std::move(other));
        return *this;
      }
      template<typename Y>
      delayed_ptr<T>& operator=(delayed_ptr<Y>&& other){
        reset();
        move_from(std::move(other));
        return *this;
      }
      ~delayed_ptr(){
        reset();
      }
    };

    template<typename T>
    class atomic_delayed_ptr{
      std::atomic<delayed_ptr<T> const*> current_ptr{new delayed_ptr<T>{}};
      std::atomic<unsigned int> reader_count{0};

      void set(delayed_ptr<T> ptr, std::memory_order mo=std::memory_order_seq_cst){
        auto old = std::unique_ptr<delayed_ptr<T> const>{
          current_ptr.exchange(new delayed_ptr<T>{std::move(ptr)}, mo)};
        // if someone's reading and might not have incremented the refcount yet, wait for them
        // before potentially scheduling a delete
        while(reader_count!=0);
      }
    public:
      atomic_delayed_ptr(){}
      atomic_delayed_ptr(delayed_ptr<T> ptr){
        set(std::move(ptr));
      }
      atomic_delayed_ptr(const atomic_delayed_ptr<T>&) = delete;
      atomic_delayed_ptr(atomic_delayed_ptr<T>&&) = delete;
      atomic_delayed_ptr<T>& operator=(const atomic_delayed_ptr<T>&) = delete;
      atomic_delayed_ptr<T>& operator=(const atomic_delayed_ptr<T>&&) = delete;
      delayed_ptr<T> operator=(delayed_ptr<T> other) noexcept{
        set(std::move(other));
      }
      bool is_lock_free() const noexcept{
        return current_ptr.is_lock_free();
      }
      void store(delayed_ptr<T> desired, std::memory_order order=std::memory_order_seq_cst) noexcept{
        set(desired.observer, desired.control_block, order);
        desired.observer = nullptr;
        desired.control_block = nullptr;
      }
      delayed_ptr<T> load(std::memory_order order=std::memory_order_seq_cst) const noexcept{
        // signal no one should delete this until we have incremented the refcount
        ++(const_cast<atomic_delayed_ptr<T>&>(*this).reader_count);
        delayed_ptr<T> const * r = current_ptr.load(order);
        auto ret = delayed_ptr<T>{*r};
        --(const_cast<atomic_delayed_ptr<T>&>(*this).reader_count);
        return ret;
      }
      operator delayed_ptr<T>() const noexcept{
        return load();
      }
      delayed_ptr<T> exchange(delayed_ptr<T> desired, std::memory_order order=std::memory_order_seq_cst){
        auto old = std::unique_ptr<delayed_ptr<T> const>{
          current_ptr.exchange(new delayed_ptr<T>{std::move(desired)}, order)};
        // wait for readers to increment their refcount
        while(reader_count!=0);
        return delayed_ptr<T>{*old};
      }
      ~atomic_delayed_ptr(){
        delete current_ptr;
      }
    };

    template<typename T, typename... Args>
    delayed_ptr<T> make_delayed(Args... args){
      delayed_ptr<T> ptr;
      ptr.reset(T(std::forward<Args>(args)...));
      return ptr;
    }
  }

    /*template<typename T>
    class atomic_delayed_ptr{
      struct rec_t{
        T* observer;
        control_block_t* control_block;
      };
      std::atomic<rec_t> rec;
      std::atomic<unsigned int> reader_count{0};

      void set(T* observer, control_block_t* control_block, std::memory_order mo=std::memory_order_seq_cst){
        rec_t old = rec.exchange(rec_t{observer, control_block}, mo);
        // if someone's reading and might not have incremented the refcount yet, wait for them
        // before potentially scheduling a delete
        while(reader_count!=0);
        _gc.reset_control_block(old.control_block);
      }
    public:
      atomic_delayed_ptr(): rec{rec_t{nullptr, nullptr}}{}
      atomic_delayed_ptr(delayed_ptr<T> ptr): atomic_delayed_ptr(){
        set(ptr.observer, ptr.control_block);
        ptr.control_block = nullptr;
        ptr.observer = nullptr;
      }
      atomic_delayed_ptr(const atomic_delayed_ptr<T>&) = delete;
      atomic_delayed_ptr(atomic_delayed_ptr<T>&&) = delete;
      atomic_delayed_ptr<T>& operator=(const atomic_delayed_ptr<T>&) = delete;
      atomic_delayed_ptr<T>& operator=(const atomic_delayed_ptr<T>&&) = delete;
      delayed_ptr<T> operator=(delayed_ptr<T> other) noexcept{
        set(other.observer, other.control_block);
        other.control_block = nullptr;
        other.observer = nullptr;
      }
      bool is_lock_free() const noexcept{
        return rec.is_lock_free();
      }
      void store(delayed_ptr<T> desired, std::memory_order order=std::memory_order_seq_cst) noexcept{
        set(desired.observer, desired.control_block, order);
        desired.observer = nullptr;
        desired.control_block = nullptr;
      }
      delayed_ptr<T> load(std::memory_order order=std::memory_order_seq_cst) const noexcept{
        // signal no one should delete this until we have incremented the refcount
        ++(const_cast<atomic_delayed_ptr<T>&>(*this).reader_count);
        auto r = rec.load(order);
        if(r.control_block){
          ++(r.control_block->count);
        }
        --(const_cast<atomic_delayed_ptr<T>&>(*this).reader_count);
        auto ret = delayed_ptr<T>{r.observer, r.control_block};
        return ret;
      }
      operator delayed_ptr<T>() const noexcept{
        return load();
      }
      delayed_ptr<T> exchange(delayed_ptr<T> desired, std::memory_order order=std::memory_order_seq_cst){
        if(desired.control_block){
          ++(desired.control_block->count);
        }
        rec_t old = rec.exchange(rec_t{desired.observer, desired.control_block}, order);
        // wait for readers to increment their refcount
        while(reader_count!=0);
        return delayed_ptr<T>{old.observer, old.control_block};
      }
    };

    template<typename T, typename... Args>
    delayed_ptr<T> make_delayed(Args... args){
      delayed_ptr<T> ptr;
      ptr.reset(T{std::forward<Args>(args)...});
      return ptr;
    }
  }*/
}
