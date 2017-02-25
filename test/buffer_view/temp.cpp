#include <list>
#include <vector>
#include <iostream>
#include <rsampler/buffer_view.hpp>

#include <range/v3/all.hpp>

void print(rsampler::iterable_view<int> vs){
  ranges::for_each(vs, [](int x){
    std::cout << x << "\n";
  });
  for(auto it = vs.begin(); it != vs.end(); ++it){
    std::cout << *it << "\n";
  }
}

int main(){
  print(std::list<int>{1,2,3,4,5});
  print(std::vector<int>{6,7,8,9,10});
  auto v = std::vector<int>{11,12,13,14,15};
  print(v | ranges::view::all);
  print(v);
  int arr[]{16,17,18,19,20};
  print(arr);
  return 0;
}
