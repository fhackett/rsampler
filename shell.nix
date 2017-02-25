with import <nixpkgs> {};
  stdenv.mkDerivation{
    name = "rsamplerEnv";
    buildInputs = [ cmake libjack2 gdb libsamplerate libsndfile valgrind doxygen armadillo gcc6 ];
  }
