# Sky Hopper — mobile flight training simulator

Airbus A320 (default) and Cessna 172 flight simulator for smartphones, with
[JSBSim](https://github.com/JSBSim-Team/jsbsim) flight dynamics compiled to
WebAssembly, real-world terrain and satellite imagery anywhere on Earth, and
every paved airport from OurAirports.

    npm start                          # http://0.0.0.0:8102
    node scripts/build-airports.mjs    # rebuild public/data/airports.json

## Features

- **A320**: own JSBSim model (`wasm/aircraft-src/gen_a320.py`) and an Airbus-style
  flight control computer in C++ (`wasm/fcc.h`, runs at 120 Hz): normal law with
  auto-trim, pitch/bank/alpha/high-speed protections, flare mode, turn coordination,
  autothrust. JS side (`public/js/aircraft/a320-systems.js`): thrust levers with
  detents, A/THR (SPEED, THR CLB/IDLE, alpha floor), AP/FD modes (SRS, RWY TRK, HDG,
  V/S, OP CLB/DES, ALT*/ALT, LOC*/LOC, G/S*/G/S), ground spoilers, autobrake,
  reversers, steering, characteristic speeds (VLS, F, S, green dot, V1/VR/V2, Vapp).
- **Training aids**: Airbus PFD with FMA, flight director and FPV, ILS on every
  runway (synthetic, 3°), PAPI, callouts (100 kt, V1, rotate, RA, minimum, retard,
  spoilers, reverse green, decel), GPWS sink rate / pull up / too low gear, and a
  landing report (sink rate, g, speed, touchdown point, centreline).
- **World**: quadtree LOD terrain from Terrarium elevation tiles with Sentinel-2
  cloudless imagery, exact WGS84/ECEF geometry (earth curvature), runways with ICAO
  markings fitted to the terrain, physically based sky, shadows. JSBSim's ground
  model samples the same elevation function (`elevation.js`) via a moving patch.

## Layout

- `server.js` — static server + caching tile proxy (`/tiles/elev`, `/tiles/sat`, cache in `./cache`)
- `public/js/main.js` — scenarios, main loop, events, camera, HUD
- `public/js/{geo,elevation,airports,terrain,runways,sky}.js` — world
- `public/js/{pfd,controls,audio,sim}.js` — instruments, input, sound, JSBSim wrapper
- `public/js/aircraft/` — models and systems for both aircraft
- `wasm/` — `bridge.cpp` (C API + ground callback), `fcc.h`, `build.sh`, tests in `wasm/test/`

## Rebuilding JSBSim

    cd wasm
    git clone --depth 1 https://github.com/JSBSim-Team/jsbsim.git   # if missing
    python3 aircraft-src/gen_a320.py
    docker run --rm -v "$PWD":/src -w /src -e HOME=/tmp emscripten/emsdk ./build.sh
    cp out/* ../public/jsbsim/
    node test/a320-laws.mjs            # headless flight-law tests (uses out-node/)

## Data and licences

Imagery: Sentinel-2 cloudless 2016 by EOX IT Services GmbH (contains modified
Copernicus Sentinel data 2016), CC BY 4.0. Elevation: Terrain Tiles on AWS Open
Data (SRTM, GMTED, ETOPO1 and others). Airports: OurAirports (public domain).
Flight dynamics: JSBSim (LGPL). The A320 model is built from public data and is
not validated against manufacturer data.
