with import <nixpkgs> {};
  clangStdenv.mkDerivation{
    name = "rsamplerEnv";
    buildInputs = [ cmake libjack2 gdb libsamplerate libsndfile valgrind doxygen ];
  }
