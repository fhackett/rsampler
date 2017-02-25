#define CATCH_CONFIG_MAIN

#include <catch.hpp>
#include <rsampler/data_store.hpp>

using rsampler::data_store::make_id;
using rsampler::data_store::ensure;
using rsampler::data_store::lazy_ensure;

TEST_CASE("multiple_ids"){
  auto id1 = make_id();
  auto id2 = make_id();
  auto id3 = make_id();
  {
    auto id4 = make_id();
    CHECK(id4 != id1);
    CHECK(id4 != id2);
    CHECK(id4 != id3);
    CHECK(id4 == id4);
  }
  auto id4 = make_id();
  CHECK(id1 == id1);
  CHECK(id1 != id2);
  CHECK(id1 != id3);
  CHECK(id1 != id4);

  CHECK(id2 == id2);
  CHECK(id2 != id3);
  CHECK(id2 != id4);

  CHECK(id3 == id3);
  CHECK(id3 != id4);

  CHECK(id4 == id4);
}

TEST_CASE("repeated ensures"){
  auto id = make_id();
  auto& v1 = ensure<int>(id, 1, 3);
  auto& v2 = ensure<int>(id, 1, 4);
  SECTION("return the same pointer"){
    CHECK(&v1 == &v2);
  }
  SECTION("keep the first value"){
    CHECK(v1 == 3);
  }
}

TEST_CASE("lazy ensure is lazy"){
  auto id = make_id();
  int call_count = 0;
  auto& v1 = lazy_ensure<int>(id, 1, [&]{
    ++call_count;
    return call_count;
  });
  auto& v2 = lazy_ensure<int>(id, 1, [&]{
    ++call_count;
    return call_count;
  });
  CHECK(call_count == 1);
  CHECK(&v1 == &v2);
  CHECK(v1 == 1);
}
