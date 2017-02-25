#include <rsampler/delayed_ptr.hpp>

using namespace rsampler::delayed;

int main(){
  auto ptr = make_delayed<int>(0);
  atomic_delayed_ptr<int> x{ptr};
  delayed_ptr<int> ptr2 = x;
  auto ptr3 = x.exchange(make_delayed<int>(1));
  return *ptr3;
}
