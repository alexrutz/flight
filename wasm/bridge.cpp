// Thin C API around JSBSim for use from JavaScript (WebAssembly build).
//
// The browser owns the terrain: it uploads a small geographic heightmap patch
// around the aircraft (meters ASL on a regular lat/lon grid), which a custom
// ground callback samples so the gear interacts with the terrain that is drawn.
// The patch is refreshed by JS as the aircraft moves.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <emscripten/emscripten.h>

#include "FGFDMExec.h"
#include "initialization/FGInitialCondition.h"
#include "input_output/FGGroundCallback.h"
#include "input_output/FGPropertyManager.h"
#include "models/FGInertial.h"
#include "fcc.h"

using namespace JSBSim;

namespace {

constexpr double FT = 0.3048;
constexpr double DEG = M_PI / 180;

// Heightmap patch on a lat/lon grid. Row 0 is the northern edge.
struct Patch {
  std::vector<float> h;
  int rows = 0, cols = 0;
  double latN = 0, lonW = 0, dLat = 1, dLon = 1;

  double at(double latDeg, double lonDeg) const {
    if (h.empty()) return 0;
    double fy = (latN - latDeg) / dLat, fx = (lonDeg - lonW) / dLon;
    // Outside the patch: clamp to the edge (JS refreshes the patch long before).
    fx = std::min(std::max(fx, 0.0), cols - 1.000001);
    fy = std::min(std::max(fy, 0.0), rows - 1.000001);
    const int i = (int)fx, j = (int)fy;
    const double u = fx - i, v = fy - j;
    const int k = j * cols + i;
    const double a = h[k], b = h[k + 1], c = h[k + cols], d = h[k + cols + 1];
    return a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v;
  }
};

Patch patch;

class PatchGround : public FGGroundCallback {
 public:
  PatchGround(double a, double b) : a_(a), b_(b) {}

  double GetAGLevel(double, const FGLocation& loc, FGLocation& contact,
                    FGColumnVector3& normal, FGColumnVector3& vel,
                    FGColumnVector3& angularVel) const override {
    vel.InitMatrix();
    angularVel.InitMatrix();
    FGLocation l = loc;
    l.SetEllipse(a_, b_);
    const double lat = l.GetGeodLatitudeRad();
    const double lon = l.GetLongitude();
    const double latDeg = lat / DEG, lonDeg = lon / DEG;

    const double hM = patch.at(latDeg, lonDeg);
    // Slope from samples 2 m apart.
    const double e = 2.0;
    const double mLat = 111132.92 - 559.82 * cos(2 * lat);
    const double mLon = 111412.84 * cos(lat) - 93.5 * cos(3 * lat);
    const double dEast = (patch.at(latDeg, lonDeg + e / mLon) - patch.at(latDeg, lonDeg - e / mLon)) / (2 * e);
    const double dNorth = (patch.at(latDeg + e / mLat, lonDeg) - patch.at(latDeg - e / mLat, lonDeg)) / (2 * e);

    FGColumnVector3 enu(-dEast, -dNorth, 1.0);
    enu.Normalize();
    const double sLat = sin(lat), cLat = cos(lat), sLon = sin(lon), cLon = cos(lon);
    const FGColumnVector3 east(-sLon, cLon, 0.0);
    const FGColumnVector3 north(-sLat * cLon, -sLat * sLon, cLat);
    const FGColumnVector3 up(cLat * cLon, cLat * sLon, sLat);
    normal = east * enu(1) + north * enu(2) + up * enu(3);

    const double hFt = hM / FT;
    contact.SetEllipse(a_, b_);
    contact.SetPositionGeodetic(lon, lat, hFt);
    return l.GetGeodAltitude() - hFt;
  }

  void SetEllipse(double a, double b) override { a_ = a; b_ = b; }

 private:
  double a_, b_;
};

std::unique_ptr<FGFDMExec> fdm;
std::vector<SGPropertyNode*> props;
fcc::Computer computer;

}  // namespace

extern "C" {

// Copies a rows x cols heightmap (meters, row 0 = north) from the JS heap.
EMSCRIPTEN_KEEPALIVE void jsb_set_patch(const float* heights, int rows, int cols, double latN,
                                        double lonW, double dLat, double dLon) {
  patch.h.assign(heights, heights + rows * cols);
  patch.rows = rows;
  patch.cols = cols;
  patch.latN = latN;
  patch.lonW = lonW;
  patch.dLat = dLat;
  patch.dLon = dLon;
}

EMSCRIPTEN_KEEPALIVE double jsb_patch_at(double lat, double lon) { return patch.at(lat, lon); }

// Loads an aircraft from the preloaded /data tree. Returns 1 on success.
EMSCRIPTEN_KEEPALIVE int jsb_load(const char* model) {
  props.clear();
  fdm = std::make_unique<FGFDMExec>();
  fdm->SetRootDir(SGPath("/data"));
  fdm->SetAircraftPath(SGPath("aircraft"));
  fdm->SetEnginePath(SGPath("engine"));
  fdm->SetSystemsPath(SGPath("systems"));
  fdm->SetDebugLevel(0);
  auto inertial = fdm->GetInertial();
  inertial->SetGroundCallback(new PatchGround(inertial->GetSemimajor(), inertial->GetSemiminor()));
  try {
    if (!fdm->LoadModel(model)) return 0;
  } catch (...) {
    return 0;
  }
  fdm->Setdt(1.0 / 120.0);
  computer = fcc::Computer();
  computer.Init(fdm.get());
  return 1;
}

// Returns a handle used by jsb_get/jsb_set, or -1 if the property is unknown.
EMSCRIPTEN_KEEPALIVE int jsb_prop(const char* name, int create) {
  if (!fdm) return -1;
  SGPropertyNode* node = fdm->GetPropertyManager()->GetNode(name, create != 0);
  if (!node) return -1;
  props.push_back(node);
  return (int)props.size() - 1;
}

EMSCRIPTEN_KEEPALIVE double jsb_get(int id) { return props[id]->getDoubleValue(); }
EMSCRIPTEN_KEEPALIVE void jsb_set(int id, double v) { props[id]->setDoubleValue(v); }

// Applies the ic/* properties set beforehand.
EMSCRIPTEN_KEEPALIVE int jsb_run_ic() {
  if (!fdm) return 0;
  try {
    fdm->ResetToInitialConditions(0);
    computer.Reset();
    return fdm->RunIC() ? 1 : 0;
  } catch (...) {
    return 0;
  }
}

// Trims the aircraft for steady flight (mode 1 = full, 2 = ground). Returns 0
// if the trim did not converge.
EMSCRIPTEN_KEEPALIVE int jsb_trim(int mode) {
  if (!fdm) return 0;
  try {
    fdm->DoTrim(mode);
    return 1;
  } catch (...) {
    return 0;
  }
}

// Tells the flight control computer the current trim so it starts in balance.
EMSCRIPTEN_KEEPALIVE void jsb_fcc_seed(double elevatorNoseUp, int airborne) {
  computer.SeedTrim(elevatorNoseUp, airborne != 0);
}

// Advances the simulation by `steps` fixed time steps. Returns 0 on failure.
EMSCRIPTEN_KEEPALIVE int jsb_step(int steps) {
  if (!fdm) return 0;
  const double dt = fdm->GetDeltaT();
  try {
    for (int i = 0; i < steps; i++) {
      computer.Update(dt);
      if (!fdm->Run()) return 0;
    }
  } catch (...) {
    return 0;
  }
  return 1;
}

EMSCRIPTEN_KEEPALIVE double jsb_dt() { return fdm ? fdm->GetDeltaT() : 0; }

}  // extern "C"
