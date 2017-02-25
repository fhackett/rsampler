#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <rsampler/pipe.hpp>
#include <rsampler/mixer.hpp>
#include <rsampler/resampler.hpp>
#include <rsampler/sampler.hpp>
#include <rsampler/delayed_ptr.hpp>

class frame_t{
  std::vector<float> data;
public:
  using iterator = std::vector<float>::iterator;
  using const_iterator = std::vector<float>::const_iterator;

  template<typename... Args>
  frame_t(Args... args): data{std::forward<Args>(args)...}
  {}
  bool operator==(frame_t const& other) const{
    return ranges::equal(data, other.data);
  }
  iterator begin(){
    return data.begin();
  }
  iterator end(){
    return data.end();
  }
  const_iterator begin() const{
    return data.begin();
  }
  const_iterator end() const{
    return data.end();
  }
  friend std::ostream& operator<<(std::ostream&, frame_t const&);
  friend void init_frame(frame_t&, std::size_t);
};

void init_frame(frame_t& f, std::size_t w){
  f.data.resize(w, 0);
}

std::ostream& operator<<(std::ostream& out, frame_t const& f){
  out << "frame{";
  bool first = true;
  for(float d: f.data){
    if(!first){
      out << ",";
    }
    out << d;
    first = false;
  }
  out << "}";
  return out;
}

namespace std{
  template<std::size_t N>
  std::ostream& operator<<(std::ostream& out, std::array<float, N> const& f){
    out << "frame{";
    bool first = true;
    for(float d: f){
      if(!first){
        out << ",";
      }
      out << d;
      first = false;
    }
    out << "}";
    return out;
  }
}

TEST_CASE("mix 2 vectors of frames"){
  std::vector<frame_t> a{
    {1.0f,2.0f,3.0f},
    {4.0f,5.0f,6.0f},
    {7.0f,8.0f,9.0f}
  };
  std::vector<frame_t> b{
    {7.0f,8.0f,9.0f},
    {4.0f,5.0f,6.0f},
    {1.0f,2.0f,2.0f}
  };

  std::vector<frame_t> expected{
    {8.0f,10.0f,12.0f},
    {8.0f,10.0f,12.0f},
    {8.0f,10.0f,11.0f}
  };

  std::vector<std::vector<frame_t>> sources{a,b};
  auto m = rsampler::mixer<frame_t>{};
  auto conn = sources >> m;

  REQUIRE(expected == ranges::to_<std::vector<frame_t>>(conn));
}

TEST_CASE("mix one vector of frames"){
  std::vector<frame_t> a{
    {1.0f,2.0f,3.0f},
    {4.0f,5.0f,6.0f},
    {7.0f,8.0f,9.0f}
  };
  std::vector<std::vector<frame_t>> sources{a};

  auto m = rsampler::mixer<frame_t>{};
  auto conn = sources >> m;

  REQUIRE(a == ranges::to_<std::vector<frame_t>>(conn));
}

TEST_CASE("mix one vector of double iterables"){
  using d = std::array<double, 1>;
  std::vector<d> a{
    {1.0},
    {4.0},
    {7.0},
  };
  std::vector<std::vector<d>> sources{a};

  auto m = rsampler::mixer<d>{};
  auto conn = sources >> m;

  REQUIRE(a == ranges::to_<std::vector<d>>(conn));
}

TEST_CASE("resampler one channel"){
  auto id = rsampler::data_store::make_id();
  using d = std::array<float, 1>;
  std::vector<d> source{
    {1.0},
    {4.0},
    {7.0},
  };
  // kind of a hack, but this is what 1-1 sample convert with zero-order
  // hold algorithm looks like apparently
  std::vector<d> expected{
    {1},
    {1},
    {4}
  };

  auto conn = source >> rsampler::resampler(id, SRC_ZERO_ORDER_HOLD);

  REQUIRE(expected == ranges::to_<std::vector<d>>(conn));
}

TEST_CASE("sampler basic"){
  auto id = rsampler::data_store::make_id();
  using d = std::array<float, 1>;
  using v = std::vector<d>;
  using dv = rsampler::delayed::delayed_ptr<v>;
  auto s1 = rsampler::delayed::make_delayed<std::vector<d>>(v{
    {1.0},
    {4.0},
    {7.0}
  });
  auto s2 = rsampler::delayed::make_delayed<std::vector<d>>(v{
    {2.0},
    {5.0},
    {8.0}
  });

  auto sampler = rsampler::sampler<std::vector<d>>{id, 1};
  auto& sample = rsampler::data_store::ensure<dv>(id, 0, dv{nullptr});
  auto& position = rsampler::data_store::ensure<std::size_t>(id, 1, 0);

  auto a = ranges::to_<std::vector<d>>(sampler | ranges::view::take(5));
  REQUIRE(a == (std::vector<d>{
    {0},
    {0},
    {0},
    {0},
    {0}
  }));
  position = 0;
  sample = s1;
  auto b = ranges::to_<std::vector<d>>(sampler | ranges::view::take(5));
  REQUIRE(b == (std::vector<d>{
    {1.0},
    {4.0},
    {7.0},
    {0},
    {0}
  }));
  position = 1;
  sample = s2;
  auto c = ranges::to_<std::vector<d>>(sampler | ranges::view::take(5));
  REQUIRE(c == (std::vector<d>{
    {5},
    {8},
    {0},
    {0},
    {0}
  }));
}
