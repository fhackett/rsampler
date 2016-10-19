#include <rsampler.hpp>
#include <iostream>

using namespace rsampler;

int main(){
  param_store ps;

  sample s1 = sndfile::file::read("../westside.flac").channel(0);
  sample s2 = sndfile::file::read("../westside.flac").channel(1);

  param_store::id a = ps.make_id(), b = ps.make_id(), rs = ps.make_id();
  ps.ensure_param<frames_t>(a, sampler::param::offset, 0);
  ps.ensure_param<frames_t>(a, sampler::param::length, s1.size());
  ps.ensure_param<sample>(a, sampler::param::sample, s1);

  ps.ensure_param<frames_t>(b, sampler::param::offset, 0);
  ps.ensure_param<frames_t>(b, sampler::param::length, s2.size());
  ps.ensure_param<sample>(b, sampler::param::sample, s2);

  ps.ensure_param<double>(rs, resampler::param::ratio, 2);

  multichannel_source msrc = make_resampler_set(ps, rs,
      multichannel({
        sampler(ps, a),
        sampler(ps, b)
      }), {ps.make_id(), ps.make_id()});

  std::vector<buffer_t> buffers{2, buffer_t{}};
  std::size_t i = 0;
  while(i < s1.size()*2){
    std::size_t generated = msrc.generate(buffer_view_list{buffers}, BUFFER_SIZE);
    //std::cout << generated << "\n";
    i += generated;
    msrc.advance(generated);
  }
  std::cout << "final: " << i << std::endl;
  return 0;
}
