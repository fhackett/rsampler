#include <rsampler.hpp>

using namespace rsampler;

int main(){
  param_store ps;
  event_dispatcher ed{ps};

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

  jack::client c{"basic_test", ed};
  auto p = c.register_port_set(2, "test",
    make_resampler_set(ps, rs,
      multichannel({
        sampler(ps, a),
        sampler(ps, b)
      }), {ps.make_id(), ps.make_id()}));
  c.activate();

  while(true){}

  return 0;
}
