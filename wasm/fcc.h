// Simplified Airbus-style flight control computer, executed once per JSBSim
// time step (120 Hz) before the flight model runs.
//
//  - Pitch normal law: stick commands load factor (C*-like) with automatic
//    trim, pitch attitude, angle-of-attack and high-speed protections.
//  - Flare mode below 50 ft RA: pitch attitude memorised and slowly lowered,
//    so the pilot has to flare.
//  - Roll law: stick commands roll rate, bank held when released, positive
//    spiral stability beyond 33 deg, 67 deg bank limit.
//  - Yaw: turn coordination and yaw damping, pedals added on top.
//  - Ground law: direct stick-to-surface, blended into flight law after liftoff.
//  - Autothrust speed mode and autopilot inner loops (bank / flight path angle).
//
// Inputs and outputs are JSBSim properties so the JavaScript side can drive
// and display everything.
#pragma once

#include <algorithm>
#include <cmath>

#include "FGFDMExec.h"
#include "input_output/FGPropertyManager.h"

namespace fcc {

using JSBSim::FGFDMExec;

constexpr double D2R = M_PI / 180.0, R2D = 180.0 / M_PI;
inline double clampv(double v, double lo, double hi) { return std::min(hi, std::max(lo, v)); }

// Interpolates a per-configuration table (conf position 0..4).
inline double confLerp(const double (&t)[5], double conf) {
  conf = clampv(conf, 0, 4);
  int i = std::min(3, (int)conf);
  double f = conf - i;
  return t[i] + (t[i + 1] - t[i]) * f;
}

constexpr double ALPHA_PROT[5] = {11.0, 14.0, 14.0, 13.8, 12.5};
constexpr double ALPHA_MAX[5] = {13.5, 16.5, 16.3, 16.0, 15.0};

class Computer {
 public:
  void Init(FGFDMExec* fdm) {
    auto pm = fdm->GetPropertyManager();
    auto n = [&](const char* name) { return pm->GetNode(name, true); };
    // configuration / pilot inputs
    active_ = n("fbw/active");
    stickPitch_ = n("fbw/stick-pitch");
    stickRoll_ = n("fbw/stick-roll");
    pedals_ = n("fbw/pedals");
    apEngaged_ = n("fbw/ap-engaged");
    apPhi_ = n("fbw/ap-phi-cmd-deg");
    apFpa_ = n("fbw/ap-fpa-cmd-deg");
    athrActive_ = n("athr/active");
    athrSpeed_ = n("athr/speed-cmd-kts");
    athrLimit_ = n("athr/thrust-limit");
    manualThrust_ = n("athr/manual-thrust");
    // sensors
    nz_ = n("accelerations/Nz");
    p_ = n("velocities/p-rad_sec");
    q_ = n("velocities/q-rad_sec");
    r_ = n("velocities/r-rad_sec");
    phi_ = n("attitude/phi-rad");
    theta_ = n("attitude/theta-rad");
    alpha_ = n("aero/alpha-deg");
    beta_ = n("aero/beta-rad");
    vc_ = n("velocities/vc-kts");
    vt_ = n("velocities/vt-fps");
    qbar_ = n("aero/qbar-psf");
    wowL_ = n("gear/unit[1]/WOW");
    wowR_ = n("gear/unit[2]/WOW");
    agl_ = n("position/h-agl-ft");
    gamma_ = n("flight-path/gamma-rad");
    conf_ = n("fcs/conf-pos");
    gearPos_ = n("gear/gear-pos-norm");
    hdot_ = n("velocities/h-dot-fps");
    // outputs
    elev_ = n("fcs/elevator-cmd-norm");
    ail_ = n("fcs/aileron-cmd-norm");
    rud_ = n("fcs/rudder-cmd-norm");
    thr0_ = n("fcs/throttle-cmd-norm[0]");
    thr1_ = n("fcs/throttle-cmd-norm[1]");
    law_ = n("fbw/law");
    protOut_ = n("fbw/alpha-prot-active");
    phiTargetOut_ = n("fbw/phi-target-deg");
    nzCmdOut_ = n("fbw/nz-cmd");
    alphaProtOut_ = n("fbw/alpha-prot-deg");
    alphaMaxOut_ = n("fbw/alpha-max-deg");
    athrCmdOut_ = n("athr/throttle-cmd");
    Reset();
  }

  void Reset() {
    integ_ = 0;
    flightBlend_ = 0;
    airborneTime_ = 0;
    groundTime_ = 1;
    phiTarget_ = 0;
    stickWasActive_ = false;
    flare_ = false;
    flareTime_ = 0;
    flareTheta_ = 0;
    rollInteg_ = 0;
    alphaLaw_ = false;
    alphaInteg_ = 0;
    athrThr_ = -1;
    lastVc_ = -1;
    accel_ = 0;
  }

  // Seeds the integrator with the current trim (used after JSBSim trim).
  void SeedTrim(double elevatorNoseUp, bool airborne) {
    integ_ = elevatorNoseUp;
    flightBlend_ = airborne ? 1 : 0;
    airborneTime_ = airborne ? 10 : 0;
    groundTime_ = airborne ? 0 : 1;
    phiTarget_ = phi_->getDoubleValue() * R2D;
  }

  void Update(double dt) {
    if (active_->getDoubleValue() < 0.5) return;
    UpdateThrust(dt);

    const double qbar = std::max(qbar_->getDoubleValue(), 1.0);
    const double sched = clampv(Q_REF / qbar, 0.35, 3.0);  // gain schedule on dynamic pressure
    const bool onGround = wowL_->getDoubleValue() > 0.5 || wowR_->getDoubleValue() > 0.5;
    const double phi = phi_->getDoubleValue() * R2D;
    const double theta = theta_->getDoubleValue() * R2D;
    const double alpha = alpha_->getDoubleValue();
    const double conf = conf_->getDoubleValue();
    const double vc = vc_->getDoubleValue();
    const double ra = agl_->getDoubleValue() - RA_OFFSET_FT;
    const double stickP = clampv(stickPitch_->getDoubleValue(), -1, 1);
    const double stickR = clampv(stickRoll_->getDoubleValue(), -1, 1);
    const bool ap = apEngaged_->getDoubleValue() > 0.5 && !onGround;

    if (onGround) {
      airborneTime_ = 0;
      groundTime_ += dt;
    } else {
      airborneTime_ += dt;
      groundTime_ = 0;
    }
    // Flight law fades in over 5 s after liftoff; ground law 1 s after touchdown.
    if (airborneTime_ > 0.2) flightBlend_ = std::min(1.0, flightBlend_ + dt / 5.0);
    if (groundTime_ > 1.0 && vc < 100) flightBlend_ = std::max(0.0, flightBlend_ - dt / 2.0);

    // ------------------------------------------------------------------ pitch
    const double aProt = confLerp(ALPHA_PROT, conf), aMax = confLerp(ALPHA_MAX, conf);
    const double phiComp = std::min(std::abs(phi), 33.0) * D2R;
    const double comp = std::cos(theta * D2R) / std::cos(phiComp);
    const bool flapsOut = conf > 0.5;
    double nzStick = stickP >= 0 ? stickP * (flapsOut ? 1.0 : 1.5) : stickP * (flapsOut ? 1.0 : 2.0);
    double nzCmd = comp + nzStick;

    if (ap && std::abs(stickP) < 0.1) {
      // Autopilot: flight path angle command
      const double fpaErr = apFpa_->getDoubleValue() - gamma_->getDoubleValue() * R2D;
      nzCmd = comp + clampv(fpaErr * 0.06, -0.3, 0.3);
    }
    // Pitch attitude protection
    if (theta > 25) nzCmd = std::min(nzCmd, comp + (30 - theta) * 0.1);
    if (theta < -10) nzCmd = std::max(nzCmd, comp + (-15 - theta) * 0.1);
    // High speed protection (VMO 350 kt)
    if (vc > 356) nzCmd = std::max(nzCmd, comp + (vc - 356) * 0.04);
    nzCmd = clampv(nzCmd, flapsOut ? 0.0 : -1.0, flapsOut ? 2.0 : 2.5);

    // Flare mode
    const bool gearDown = gearPos_->getDoubleValue() > 0.9;
    if (!flare_ && !onGround && flightBlend_ > 0.99 && gearDown && ra < 50 && hdot_->getDoubleValue() < 0) {
      flare_ = true;
      flareTime_ = 0;
      flareTheta_ = theta;
    }
    if (flare_ && (ra > 100 || (onGround && groundTime_ > 1.0))) flare_ = false;

    const double q = q_->getDoubleValue();
    bool prot = false;
    double elevNoseUp;
    if (flare_) {
      flareTime_ += dt;
      const double thetaCmd = flareTheta_ - std::min(flareTime_ / 8.0, 1.0) * 2.0 + (stickP > 0 ? stickP * 7 : stickP * 5);
      const double e = (thetaCmd - theta) * D2R;
      integ_ = clampv(integ_ + sched * K_FLARE_I * e * dt, -1, 1);
      elevNoseUp = integ_ + sched * (K_FLARE_P * e - K_Q * q);
    } else {
      const double e = nzCmd - nz_->getDoubleValue();
      elevNoseUp = integ_ + sched * (K_P * e - K_Q * q);
      // Angle-of-attack protection: a separate alpha law (stick commands alpha
      // between alpha prot and alpha max); the more nose-down law wins.
      bool alphaLaw = false;
      if (!onGround && alpha > aProt - 3.0) {
        const double alphaCmd = aProt + std::max(0.0, stickP) * (aMax - aProt);
        const double ea = (alphaCmd - alpha) * D2R;
        if (!alphaLaw_) alphaInteg_ = integ_;
        const double elevA = alphaInteg_ + sched * (K_ALPHA_P * ea - K_Q * q);
        if (elevA < elevNoseUp) {
          alphaLaw = true;
          alphaInteg_ = clampv(alphaInteg_ + sched * K_ALPHA_I * ea * dt, -1, 1);
          elevNoseUp = elevA;
          integ_ = alphaInteg_;  // keep the load factor law from winding up
        }
      }
      alphaLaw_ = alphaLaw;
      prot = alphaLaw && alpha > aProt - 0.5;
      if (!alphaLaw) integ_ = clampv(integ_ + sched * K_I * e * dt, -1, 1);
    }
    const double direct = stickP * GROUND_PITCH_GAIN + TAKEOFF_TRIM;
    if (flightBlend_ < 1) {
      // On the ground the integrator follows the stick so the transition is smooth.
      if (flightBlend_ <= 0) integ_ = direct;
      elevNoseUp = direct * (1 - flightBlend_) + elevNoseUp * flightBlend_;
    }
    elev_->setDoubleValue(clampv(-elevNoseUp, -1, 1));

    // ------------------------------------------------------------------- roll
    const double p = p_->getDoubleValue() * R2D;
    const bool stickActive = std::abs(stickR) > 0.03;
    const double bankLimit = prot || vc > 356 ? 45 : 67;
    double pFeed = 0;
    if (ap && !stickActive) {
      const double cmd = clampv(apPhi_->getDoubleValue(), -30, 30);
      phiTarget_ += clampv(cmd - phiTarget_, -5 * dt, 5 * dt);
    } else if (stickActive) {
      pFeed = stickR * 15.0;
      phiTarget_ = clampv(phiTarget_ + pFeed * dt, -bankLimit, bankLimit);
      // Don't let the target run away from the aircraft.
      phiTarget_ = clampv(phiTarget_, phi - 10, phi + 10);
    } else {
      if (stickWasActive_) phiTarget_ = phi;
      if (std::abs(phiTarget_) > 33) phiTarget_ -= std::copysign(std::min(5 * dt, std::abs(phiTarget_) - 33), phiTarget_);
    }
    stickWasActive_ = stickActive;
    const double pDes = clampv((phiTarget_ - phi) * 1.5, -15, 15) + pFeed * 0.6;
    rollInteg_ = clampv(rollInteg_ + (phiTarget_ - phi) * 0.002 * dt * sched, -0.2, 0.2);
    double ail = sched * K_ROLL * (pDes - p) * D2R + rollInteg_;
    if (flightBlend_ < 1) {
      ail = stickR * (1 - flightBlend_) + ail * flightBlend_;
      if (flightBlend_ <= 0) {
        phiTarget_ = phi;
        rollInteg_ = 0;
      }
    }
    ail_->setDoubleValue(clampv(ail, -1, 1));

    // -------------------------------------------------------------------- yaw
    const double pedals = clampv(pedals_->getDoubleValue(), -1, 1);
    double rud = -pedals;
    if (!onGround) {
      const double v = std::max(vt_->getDoubleValue(), 100.0);
      const double rCoord = 32.17 * std::tan(phi * D2R) / v;
      rud += sched * (-K_BETA * beta_->getDoubleValue() + K_YAW_DAMP * (r_->getDoubleValue() - rCoord)) * flightBlend_;
    }
    rud_->setDoubleValue(clampv(rud, -1, 1));

    law_->setDoubleValue(flare_ ? 2 : flightBlend_ > 0.5 ? 1 : 0);
    protOut_->setDoubleValue(prot ? 1 : 0);
    phiTargetOut_->setDoubleValue(phiTarget_);
    nzCmdOut_->setDoubleValue(nzCmd);
    alphaProtOut_->setDoubleValue(aProt);
    alphaMaxOut_->setDoubleValue(aMax);
  }

 private:
  void UpdateThrust(double dt) {
    const double vc = vc_->getDoubleValue();
    if (lastVc_ < 0) lastVc_ = vc;
    accel_ += ((vc - lastVc_) / dt - accel_) * std::min(1.0, dt / 0.5);  // kt/s, filtered
    lastVc_ = vc;
    const double manual = clampv(manualThrust_->getDoubleValue(), 0, 1);
    double thr = manual;
    if (athrActive_->getDoubleValue() > 0.5) {
      const double limit = clampv(athrLimit_->getDoubleValue(), 0, 1);
      if (athrThr_ < 0) athrThr_ = thr0_->getDoubleValue();  // engage from the current thrust
      const double err = athrSpeed_->getDoubleValue() - vc;
      // Velocity-form PI: aim for an acceleration proportional to the speed error.
      const double accelCmd = clampv(err * 0.3, -1.5, 1.5);
      athrThr_ = clampv(athrThr_ + (accelCmd - accel_) * 0.09 * dt, 0, limit);
      thr = athrThr_;
    } else {
      athrThr_ = -1;
    }
    athrCmdOut_->setDoubleValue(thr);
    thr0_->setDoubleValue(thr);
    thr1_->setDoubleValue(thr);
  }

  // Tuning
  static constexpr double Q_REF = 211.0;  // psf, 250 kt at sea level
  static constexpr double K_P = 0.35, K_I = 0.9, K_Q = 1.6;
  static constexpr double K_FLARE_P = 3.0, K_FLARE_I = 1.0;
  static constexpr double K_ALPHA_P = 3.0, K_ALPHA_I = 2.0;
  static constexpr double K_ROLL = 2.4;
  static constexpr double K_BETA = 2.0, K_YAW_DAMP = 3.0;
  static constexpr double TAKEOFF_TRIM = 0.08;
  static constexpr double GROUND_PITCH_GAIN = 0.55;
  static constexpr double RA_OFFSET_FT = 11.0;  // CG height above the wheels

  SGPropertyNode *active_, *stickPitch_, *stickRoll_, *pedals_, *apEngaged_, *apPhi_, *apFpa_;
  SGPropertyNode *athrActive_, *athrSpeed_, *athrLimit_, *manualThrust_;
  SGPropertyNode *nz_, *p_, *q_, *r_, *phi_, *theta_, *alpha_, *beta_, *vc_, *vt_, *qbar_;
  SGPropertyNode *wowL_, *wowR_, *agl_, *gamma_, *conf_, *gearPos_, *hdot_;
  SGPropertyNode *elev_, *ail_, *rud_, *thr0_, *thr1_;
  SGPropertyNode *law_, *protOut_, *phiTargetOut_, *nzCmdOut_, *alphaProtOut_, *alphaMaxOut_, *athrCmdOut_;

  double integ_, flightBlend_, airborneTime_, groundTime_, phiTarget_, rollInteg_;
  bool stickWasActive_, flare_, alphaLaw_ = false;
  double alphaInteg_ = 0;
  double flareTime_, flareTheta_;
  double athrThr_, lastVc_, accel_;
};

}  // namespace fcc
