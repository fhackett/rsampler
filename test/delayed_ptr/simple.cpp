#include <rsampler/delayed_ptr.hpp>

using namespace rsampler::delayed;

int main(){
    auto ptr = make_delayed<int>(0);
    return *ptr;
}
