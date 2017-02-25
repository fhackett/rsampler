#pragma once

#include <memory>
#include <map>
#include <unordered_map>
#include <atomic>


namespace rsampler{
  namespace data_store{
    struct _record_base{
      virtual ~_record_base() = default;
    };
    template<typename T>
    struct _record: _record_base{
      _record(T t): data{std::make_unique<T>(std::move(t))}{}
      std::unique_ptr<T> data;
    };
    struct _map_record_t{
      std::atomic<std::size_t> refcount{0};
      std::map<std::size_t,std::unique_ptr<_record_base> const> map;
    };
    class id_t{
      _map_record_t& rec;
      std::size_t const num;
      id_t(std::size_t num, _map_record_t& rec):
        rec{rec},
        num{num}
      {
        ++rec.refcount;
      }
    public:
      id_t(const id_t& i):
        rec{i.rec},
        num{i.num}
      {
        ++rec.refcount;
      }
      bool operator==(const id_t& other) const{
        return num == other.num;
      }
      bool operator!=(const id_t& other) const{
        return num != other.num;
      }
      bool operator<(const id_t& other) const{
        return num < other.num;
      }
      bool operator>(const id_t& other) const{
        return num > other.num;
      }
      bool operator>=(const id_t& other) const{
        return num >= other.num;
      }
      bool operator<=(const id_t& other) const{
        return num <= other.num;
      }
      ~id_t();

      friend id_t make_id();
      template<typename T, typename Fn>
      friend T& lazy_ensure(const id_t&, std::size_t, Fn);
    };
    unsigned int _next_free_id = 1;
    std::unordered_map<std::size_t,std::unique_ptr<_map_record_t>> _store;
    id_t::~id_t(){
      std::size_t v = --rec.refcount;
      if(v == 0){
        _store.erase(num);
        _next_free_id = num;
      }
    }

    id_t make_id(){
      unsigned int tmp = _next_free_id;
      while(_store.count(tmp) && tmp != 0){
        ++tmp;
      }
      _next_free_id = tmp + 1;
      auto res = _store.emplace(
        std::make_pair(
          tmp,
          std::make_unique<_map_record_t>()));
      return id_t{tmp, *res.first->second};
    };

    template<typename T, typename Fn>
    T& lazy_ensure(const id_t& id, std::size_t index, Fn fn){
      auto& map = _store.at(id.num)->map;
      auto it = map.find(index);
      if(it != map.end()){
        auto ptr = it->second.get();
        return *dynamic_cast<_record<T>&>(*ptr).data;
      }else{
        auto record = std::make_unique<_record<T>>(fn());
        auto& result = *record->data;
        auto res = map.emplace(
          std::make_pair(
            index,
            std::move(record)));
        return result;
      }
    }

    template<typename T>
    T& ensure(const id_t& id, std::size_t index, T v){
      return lazy_ensure<T>(id,index,[&]{
        return std::move(v);
      });
    }
  }
}
