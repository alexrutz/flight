#!/bin/bash
# Builds JSBSim + bridge.cpp to WebAssembly. Run inside the emscripten/emsdk image:
#   docker run --rm -v /root/flight/wasm:/src -w /src emscripten/emsdk ./build.sh
# Produces out/ (browser build, copy to ../public/jsbsim) and out-node/ (for tests).
set -euo pipefail
OUT=/src/out
NODE_OUT=/src/out-node
mkdir -p build "$OUT" "$NODE_OUT"
cd build
if [ ! -f Makefile ]; then
  emcmake cmake ../jsbsim -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_MODULE=OFF -DBUILD_DOCS=OFF \
    -DCMAKE_CXX_FLAGS="-O2 -fexceptions" -DCMAKE_C_FLAGS="-O2" > cmake.log
fi
emmake make -j"$(nproc)" libJSBSim 2>&1 | grep -E "error|Error" || true
LIB=$(find . -name 'libJSBSim.a' | head -1)
cd /src
COMMON=(-O2 -fexceptions -std=c++17 bridge.cpp "build/$LIB" -I jsbsim/src -I build/src
  -sMODULARIZE -sEXPORT_ES6 -sEXPORT_NAME=createJSBSim -sALLOW_MEMORY_GROWTH
  -sEXPORTED_RUNTIME_METHODS=ccall,cwrap,HEAPF32 -sEXPORTED_FUNCTIONS=_malloc,_free)
em++ "${COMMON[@]}" -sENVIRONMENT=web --preload-file data@/data -o "$OUT/jsbsim.js"
em++ "${COMMON[@]}" -sENVIRONMENT=node --embed-file data@/data -o "$NODE_OUT/jsbsim.mjs"
ls -la "$OUT" "$NODE_OUT"
