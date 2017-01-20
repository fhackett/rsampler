#include <iostream>
#include <future>

#include <delayed_ptr.hpp>

using namespace delayed;

struct on_delete{
    std::shared_ptr<int> ptr;
    on_delete(std::function<void()> fn): ptr{new int,
        [=](int* p){
            delete p;
            fn();
        }}
    {}
};

int main(){
    std::promise<int> p1;
    {
        auto ptr = make_delayed<on_delete>([&](){
            std::cout << "destruct\n";
            p1.set_value(1);
        });
    }
    std::cout << p1.get_future().get() << "\n";

    std::promise<int> p2;
    {
        auto ptr = make_delayed<on_delete>([&](){
            p2.set_value(2);
        });
    }
    std::cout << p2.get_future().get() << "\n";
    return 0;
}
