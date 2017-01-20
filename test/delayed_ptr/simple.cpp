#include <delayed_ptr.hpp>

using namespace delayed;

int main(){
    auto ptr = make_delayed<int>(0);
    return *ptr;
}
