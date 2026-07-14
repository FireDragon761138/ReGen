// ReGen.cpp
// ReGen -- lossy-audio reconstruction for game/legacy content (VST 2.4, 64-bit)
// for Equalizer APO.  8-in/8-out, channel-role aware (standard Windows 7.1).
//
// The spine of the plugin is a per-group ROLLOFF SERVO: two steep (12th-order
// IIR highpass) probes straddle the current corner estimate. If the upper
// probe still sees energy, real content extends higher -> corner slews up
// (fast). If even the lower probe is dead, content stops below the estimate ->
// corner slews down (slow). Gated peak-hold meters keep bass-only passages
// and silence from dragging the corner around. Everything downstream keys off
// that one measurement:
//
//   cleanup   adaptive 12 dB/oct lowpass just above the corner buries codec
//             swirl/hash (Cleanup knob = dry/filtered mix)
//   smoother  envelope-ratio gain on the top octave clamps warbly level
//             flutter without touching stable content (Smooth knob)
//   regen     upper mids (3-8 kHz) -> soft asymmetric saturator -> highpassed
//             above the corner and mixed in, level-linked to the real
//             midrange envelope so quiet passages don't hiss (Regen knob)
//   transient zero-lookahead attack boost masks pre-echo smear (Attack knob)
//
// A "deficit" factor derived from the corner (0 at ~19.5 kHz, 1 at <=16 kHz)
// scales the whole chain, so pristine content gets an automatic light touch --
// no preset system needed.
//
// Channel roles (Windows 7.1: 0 FL 1 FR 2 FC 3 LFE 4 BL 5 BR 6 SL 7 SR):
//   FL/FR  full chain            FC     speech-tuned: no transient shaper,
//   SL/SR  cleanup + light regen        sibilance detector ducks the exciter
//   BL/BR  cleanup + light regen  LFE   untouched passthrough
// Detection and dynamics gains are shared per pair so imaging never wanders.
//
// Params: 0 Cleanup  1 Smooth  2 Regen  3 Attack  4 Mix  5 Freeze(0/1)
//
// Build: see build_mingw.bat.  License: BSD-2-Clause.

#include "vst2_min.h"
#include <cmath>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <xmmintrin.h>   // MXCSR access for FTZ/DAZ denormal flushing

#if defined(_WIN32)
#include <windows.h>
#include <commctrl.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const VstInt32 kNumParams  = 6;
static const int      kNumSliders = 5;                 // Freeze is a checkbox
enum { P_CLEAN = 0, P_SMOOTH, P_REGEN, P_TRANS, P_MIX, P_FREEZE };

enum { CH_FL = 0, CH_FR, CH_FC, CH_LFE, CH_BL, CH_BR, CH_SL, CH_SR };

// effVendorSpecific query: index='Roff', value=group 0..3 -> corner in Hz.
#define REGEN_ROLLOFF_QUERY CCONST('R', 'o', 'f', 'f')

// Control-rate block: adaptive coefficients and the servo update every
// kCtrl samples (~1.3 ms at 48 kHz); per-sample cost stays pure filtering.
static const int kCtrl = 64;

// ------------------------------------------------------------- primitives ----
struct OnePoleLP {
    double a = 0.0, z = 0.0;
    void setCutoff(double fs, double fc) {
        if (fc > fs * 0.49) fc = fs * 0.49;
        a = 1.0 - std::exp(-2.0 * M_PI * fc / fs);
    }
    inline double process(double x) { z += a * (x - z); return z; }
    void reset() { z = 0.0; }
};

// RBJ biquad, direct form II transposed. Coefficients may be swapped while
// running (the servo slews slowly, so no audible state transient).
struct Biquad {
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
    inline double process(double x) {
        double y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
    void reset() { z1 = z2 = 0.0; }
    void setLP(double fs, double fc, double Q) {
        double cw, al, a0 = pre(fs, fc, Q, cw, al);
        b0 = (1 - cw) * 0.5 / a0; b1 = (1 - cw) / a0; b2 = b0;
        a1 = -2 * cw / a0; a2 = (1 - al) / a0;
    }
    void setHP(double fs, double fc, double Q) {
        double cw, al, a0 = pre(fs, fc, Q, cw, al);
        b0 = (1 + cw) * 0.5 / a0; b1 = -(1 + cw) / a0; b2 = b0;
        a1 = -2 * cw / a0; a2 = (1 - al) / a0;
    }
    void setBP(double fs, double fc, double Q) {   // constant 0 dB peak
        double cw, al, a0 = pre(fs, fc, Q, cw, al);
        b0 = al / a0; b1 = 0; b2 = -al / a0;
        a1 = -2 * cw / a0; a2 = (1 - al) / a0;
    }
private:
    static double pre(double fs, double& fc, double Q, double& cw, double& al) {
        if (fc > fs * 0.47) fc = fs * 0.47;
        if (fc < 10.0)      fc = 10.0;
        double w = 2.0 * M_PI * fc / fs;
        cw = std::cos(w); al = std::sin(w) / (2.0 * Q);
        return 1.0 + al;
    }
};

// 12th-order Butterworth highpass (6 cascaded biquads). This steepness is the
// whole detector: a 2nd-order probe's skirt leaks strong tones from an octave
// away at only -12 dB, which swamps a -36 dB alive/dead threshold. At 72
// dB/oct the tonal-leakage error in the corner estimate stays under ~0.1 oct.
struct HP12 {
    Biquad s[6];
    void set(double fs, double fc) {
        static const double Q[6] = { 3.8306, 1.3066, 0.8213, 0.6302, 0.5412, 0.5043 };
        for (int i = 0; i < 6; ++i) s[i].setHP(fs, fc, Q[i]);
    }
    inline double process(double x) {
        for (int i = 0; i < 6; ++i) x = s[i].process(x);
        return x;
    }
    void reset() { for (int i = 0; i < 6; ++i) s[i].reset(); }
};

// Attack/release envelope follower on a rectified input.
struct EnvAR {
    double v = 0, ka = 0, kr = 0;
    void init(double fs, double attMs, double relMs) {
        ka = 1.0 - std::exp(-1.0 / (fs * 0.001 * attMs));
        kr = 1.0 - std::exp(-1.0 / (fs * 0.001 * relMs));
        v = 0.0;
    }
    inline double next(double a) { v += ((a > v) ? ka : kr) * (a - v); return v; }
    void reset() { v = 0.0; }
};

// One-pole smoother so knob moves don't zipper.
struct Smooth {
    double v = 0, coeff = 0;
    void init(double fs, double ms, double start) {
        coeff = std::exp(-1.0 / (fs * 0.001 * ms));
        v = start;
    }
    inline double next(double target) { return v = target + coeff * (v - target); }
    void snap(double target) { v = target; }   // idle-gate wake: settle instantly
};

// Soft clipper: linear below 0.8, smooth knee to a 1.0 ceiling.
static inline double softclip(double x) {
    const double t = 0.8;
    double a = std::fabs(x);
    if (a <= t) return x;
    double s = (x < 0.0) ? -1.0 : 1.0;
    return s * (t + (1.0 - t) * std::tanh((a - t) / (1.0 - t)));
}

static inline double clamp01(double v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

// Clamped Pade tanh: within ~1e-3 of tanh for |x|<3, hard 1 beyond. Plenty
// for waveshaping and much cheaper than libm's tanh at 7 calls per sample.
static inline double fastTanh(double x) {
    if (x >  3.0) return  1.0;
    if (x < -3.0) return -1.0;
    double x2 = x * x;
    return x * (27.0 + x2) / (27.0 + 9.0 * x2);
}

// Asymmetric soft saturator for the exciter: the bias term adds even
// harmonics (2nd fills the octave right above the source band), tanh adds
// odd. The DC offset it introduces dies in the injection highpass.
static inline double shapeHF(double n) {
    const double tb = 0.53704956699803528;   // tanh(0.6)
    return fastTanh(1.6 * n + 0.6) - tb;
}

// ------------------------------------------------------------ role group ----
// A group is one detection unit: a stereo pair or a mono channel. Detection
// runs on the pair's mono sum; time-varying GAINS derived from shared
// envelopes are applied identically to both channels so imaging stays put.
struct Group {
    int  chA = 0, chB = -1;               // chB < 0 -> mono group
    bool useSmooth = false, useTrans = false, useDuck = false;
    double exScale = 1.0;

    // --- rolloff servo ---
    Biquad refBP;                         // broad 1-4 kHz mid reference
    HP12   probeLo, probeHi;              // straddle the corner estimate
    double msRef = 0, msLo = 0, msHi = 0; // 200 ms mean-square meters
    double hRef = 0, hLo = 0, hHi = 0;    // gated peak-hold versions
    double wideMS = 0;                    // wideband gate meter
    double cornerLog = 0, appliedLog = -1;
    double deficit = 0;                   // 0 = pristine, 1 = badly bandlimited

    // --- adaptive processing ---
    Biquad    cleanLP[2];
    OnePoleLP smLP[2];                    // top-band split (hb = x - lp)
    EnvAR     hbFast, hbSlow;             // shared flutter envelopes
    OnePoleLP ex3[2], ex8[2];             // 3-8 kHz exciter source band
    Biquad    injHP[2][2];                // 4th-order injection highpass
    EnvAR     midEnv;                     // shared level link
    OnePoleLP dkLo1, dkLo2, dkHi1, dkHi2; // sibilance bands (center only)
    EnvAR     envSib, envVoice;
    EnvAR     trFast, trSlow;             // transient detector (fronts only)

    void configure(double fs, double cornerMaxLog) {
        refBP.setBP(fs, 2000.0, 0.8);
        for (int c = 0; c < 2; ++c) {
            ex3[c].setCutoff(fs, 3000.0);
            ex8[c].setCutoff(fs, 8000.0);
        }
        dkLo1.setCutoff(fs, 1000.0); dkLo2.setCutoff(fs, 3000.0);
        dkHi1.setCutoff(fs, 5000.0); dkHi2.setCutoff(fs, 9000.0);
        hbFast.init(fs, 1.0, 15.0);   hbSlow.init(fs, 40.0, 80.0);
        midEnv.init(fs, 4.0, 50.0);
        envSib.init(fs, 3.0, 50.0);   envVoice.init(fs, 3.0, 50.0);
        trFast.init(fs, 0.5, 40.0);   trSlow.init(fs, 15.0, 140.0);
        cornerLog = cornerMaxLog;
        appliedLog = -1.0;            // force coefficient apply on next ctrl
    }

    void resetSignalState() {         // clears audio state, keeps the learned corner
        refBP.reset(); probeLo.reset(); probeHi.reset();
        msRef = msLo = msHi = wideMS = 0.0;
        for (int c = 0; c < 2; ++c) {
            cleanLP[c].reset(); smLP[c].reset();
            ex3[c].reset(); ex8[c].reset();
            injHP[c][0].reset(); injHP[c][1].reset();
        }
        hbFast.reset(); hbSlow.reset(); midEnv.reset();
        envSib.reset(); envVoice.reset();
        trFast.reset(); trSlow.reset();
        dkLo1.reset(); dkLo2.reset(); dkHi1.reset(); dkHi2.reset();
    }

    // Control-rate servo step + adaptive coefficient refresh.
    void ctrl(double fs, bool frozen, double stepUp, double stepDown,
              double holdAtt, double holdRel, double cornerMinLog,
              double cornerMaxLog) {
        bool gate = wideMS > 3e-7;    // ~-65 dBFS: below this, hold everything
        auto hold = [&](double& h, double e) {
            if (e > h)     h += holdAtt * (e - h);
            else if (gate) h += holdRel * (e - h);
        };
        hold(hRef, msRef); hold(hLo, msLo); hold(hHi, msHi);

        if (!frozen && gate && hRef > 1e-8) {
            double th = hRef * 2.5e-4 + 1e-12;      // -36 dB alive threshold
            if      (hHi > th) cornerLog += stepUp;   // real content above -> open
            else if (hLo < th) cornerLog -= stepDown; // nothing at corner -> close
            if (cornerLog < cornerMinLog) cornerLog = cornerMinLog;
            if (cornerLog > cornerMaxLog) cornerLog = cornerMaxLog;
        }

        double corner = std::exp2(cornerLog);
        deficit = clamp01((19500.0 - corner) / 3500.0);

        if (std::fabs(cornerLog - appliedLog) > 5e-4) {
            appliedLog = cornerLog;
            probeLo.set(fs, corner);
            probeHi.set(fs, std::min(corner * 1.30, fs * 0.47));
            int nch = (chB >= 0) ? 2 : 1;
            for (int c = 0; c < nch; ++c) {
                cleanLP[c].setLP(fs, corner * 1.25, 0.7071);
                injHP[c][0].setHP(fs, corner * 1.08, 0.5412);
                injHP[c][1].setHP(fs, corner * 1.08, 1.3066);
                smLP[c].setCutoff(fs, corner * 0.55);
            }
        }
    }

    // Per-sample processing. x[] holds the working samples for all 8 channels
    // (only ours are touched); dry[] is the untouched input. kMS100/kMS200 are
    // the meter one-pole coefficients; cln/smo/reg/trn are the smoothed knobs.
    inline void tick(double* x, const double* dry, double kMS100, double kMS200,
                     double cln, double smo, double reg, double trn) {
        int a = chA, b = chB;
        double xa = x[a], xb = (b >= 0) ? x[b] : 0.0;

        // -- detection taps (meters free-run; gating happens in ctrl()) --
        double m = (b >= 0) ? 0.5 * (xa + xb) : xa;
        wideMS += kMS100 * (m * m - wideMS);
        double r  = refBP.process(m);   msRef += kMS200 * (r * r - msRef);
        double lo = probeLo.process(m); msLo  += kMS200 * (lo * lo - msLo);
        double hi = probeHi.process(m); msHi  += kMS200 * (hi * hi - msHi);

        // -- cleanup: bury swirl/hash just above the corner --
        double cEff = cln * deficit;
        double lpA = cleanLP[0].process(xa); xa += cEff * (lpA - xa);
        if (b >= 0) { double lpB = cleanLP[1].process(xb); xb += cEff * (lpB - xb); }

        // -- top-octave flutter smoother (attenuate-only, shared gain) --
        // Skipped when its gain is zero (pristine content / knob down); the
        // gain ramps continuously through zero, so re-engaging never clicks
        // even though the envelopes restart cold.
        if (useSmooth && smo * deficit > 1e-5) {
            double hbA = xa - smLP[0].process(xa);
            double hbB = (b >= 0) ? xb - smLP[1].process(xb) : 0.0;
            double he  = (b >= 0) ? 0.5 * (std::fabs(hbA) + std::fabs(hbB))
                                  : std::fabs(hbA);
            double ef = hbFast.next(he), es = hbSlow.next(he);
            double g = (es + 1e-7) / (ef + 1e-7);
            if (g > 1.0)  g = 1.0;
            if (g < 0.35) g = 0.35;
            double ge = 1.0 + smo * deficit * (g - 1.0);
            xa += (ge - 1.0) * hbA;
            if (b >= 0) xb += (ge - 1.0) * hbB;
        }

        // -- regeneration: harmonics from 3-8 kHz, injected above the corner --
        // Also gain-gated like the smoother: envelopes restart cold on
        // re-engage but the add ramps up from zero, so no click.
        double gEx = reg * exScale * deficit * 1.2;
        if (gEx > 1e-5) {
            double bpA = ex8[0].process(xa - ex3[0].process(xa));
            double bpB = (b >= 0) ? ex8[1].process(xb - ex3[1].process(xb)) : 0.0;
            double ae  = (b >= 0) ? 0.5 * (std::fabs(bpA) + std::fabs(bpB))
                                  : std::fabs(bpA);
            double me  = midEnv.next(ae);
            if (useDuck) {   // duck the exciter on /s/ so dialogue doesn't spit
                double v  = dkLo2.process(xa) - dkLo1.process(xa);   // 1-3 kHz
                double s  = dkHi2.process(xa) - dkHi1.process(xa);   // 5-9 kHz
                double rv = envVoice.next(std::fabs(v));
                double rs = envSib.next(std::fabs(s));
                double rr = rs / (rv + 1e-6);
                gEx *= 1.0 / (1.0 + 3.0 * std::max(0.0, rr - 0.7));
            }
            double nrm = 1.0 / (1.4142 * me + 3e-4);   // normalize, shape, re-link
            xa += injHP[0][1].process(injHP[0][0].process(shapeHF(bpA * nrm))) * me * gEx;
            if (b >= 0)
                xb += injHP[1][1].process(injHP[1][0].process(shapeHF(bpB * nrm))) * me * gEx;
        }

        // -- transient attack boost (masks pre-echo smear) --
        if (useTrans) {
            double d = (b >= 0) ? 0.5 * (std::fabs(dry[a]) + std::fabs(dry[b]))
                                : std::fabs(dry[a]);
            double ef = trFast.next(d), es = trSlow.next(d);
            double onset = (ef - es) / (es + 1e-5);
            if (onset < 0.0) onset = 0.0;
            if (onset > 1.6) onset = 1.6;
            double tg = 1.0 + trn * deficit * 0.8 * onset;
            if (tg > 2.0) tg = 2.0;
            xa *= tg;
            if (b >= 0) xb *= tg;
        }

        x[a] = xa;
        if (b >= 0) x[b] = xb;
    }
};

// ---------------------------------------------------------------- Plugin ----
static const int kNumGroups = 4;
enum { G_FRONT = 0, G_CENTER, G_SIDE, G_BACK };

struct ReGen {
    AEffect effect;
    audioMasterCallback master = nullptr;

    float  params[kNumParams];
    double fs = 44100.0;

    Group groups[kNumGroups];
    int   ctrlCount = 0;

    // Idle gate: after kGateHold of sub-threshold input, signal state is
    // flushed and processing is skipped until the first loud sample (which is
    // always processed -- the wake test runs BEFORE the skip, so onsets after
    // silence are never touched). Transparent in EAPO, big save on silent
    // DAW tracks. The learned corners survive dormancy.
    static constexpr double kGateThresh = 1e-5;   // ~-100 dBFS peak
    static constexpr double kGateHold   = 0.25;   // seconds below thresh to sleep
    int  silentRun = 0;
    bool dormant   = false;

    // control-rate constants (recomputed on sample-rate change)
    double kMS100 = 0, kMS200 = 0;
    double stepUp = 0, stepDown = 0, holdAtt = 0, holdRel = 0;
    double cornerMinLog = 0, cornerMaxLog = 0;

    Smooth smClean, smSmooth, smRegen, smTrans, smMix;

#if defined(_WIN32)
    HWND edContainer = nullptr;
    HWND edSlider[kNumSliders] = {};
    HWND edValue[kNumSliders]  = {};
    HWND edFreeze = nullptr, edStatus = nullptr;
    void openEditor(HWND parent);
    void closeEditor();
    void refreshValue(int i);
    void refreshStatus();
    void resetDefaults();
    void syncFromHost();       // reflect host/automation param moves in the GUI
#endif

    // Tell the host a GUI edit happened so DAWs record automation / mark the
    // project dirty. EAPO's master callback just ignores it.
    void notifyAutomate(int i) {
        if (master) master(&effect, audioMasterAutomate, i, 0, nullptr, params[i]);
    }

    ReGen() {
        setDefaultParams();
        setSampleRate(44100.0);
    }

    void setDefaultParams() {
        params[P_CLEAN]  = 0.7f;
        params[P_SMOOTH] = 0.5f;
        params[P_REGEN]  = 0.6f;
        params[P_TRANS]  = 0.5f;
        params[P_MIX]    = 1.0f;
        params[P_FREEZE] = 0.0f;
    }

    void setSampleRate(double sr) {
        fs = sr;
        kMS100 = 1.0 - std::exp(-1.0 / (0.100 * fs));
        kMS200 = 1.0 - std::exp(-1.0 / (0.200 * fs));

        double blockSec = (double)kCtrl / fs;
        stepUp   = 2.00 * blockSec;    // open fast: real HF proves itself
        stepDown = 0.45 * blockSec;    // close slowly: don't chase dark passages
        holdAtt  = 1.0 - std::exp(-blockSec / 0.080);
        holdRel  = 1.0 - std::exp(-blockSec / 2.5);
        cornerMinLog = std::log2(6000.0);
        cornerMaxLog = std::log2(std::min(20000.0, 0.44 * fs));

        groups[G_FRONT]  = Group{}; groups[G_FRONT].chA  = CH_FL; groups[G_FRONT].chB  = CH_FR;
        groups[G_FRONT].useSmooth = true; groups[G_FRONT].useTrans = true;
        groups[G_CENTER] = Group{}; groups[G_CENTER].chA = CH_FC;
        groups[G_CENTER].useSmooth = true; groups[G_CENTER].useDuck = true;
        groups[G_SIDE]   = Group{}; groups[G_SIDE].chA   = CH_SL; groups[G_SIDE].chB   = CH_SR;
        groups[G_SIDE].exScale = 0.6;
        groups[G_BACK]   = Group{}; groups[G_BACK].chA   = CH_BL; groups[G_BACK].chB   = CH_BR;
        groups[G_BACK].exScale = 0.6;
        for (int g = 0; g < kNumGroups; ++g) groups[g].configure(fs, cornerMaxLog);

        smClean.init(fs, 30.0, params[P_CLEAN]);
        smSmooth.init(fs, 30.0, params[P_SMOOTH]);
        smRegen.init(fs, 30.0, params[P_REGEN]);
        smTrans.init(fs, 30.0, params[P_TRANS]);
        smMix.init(fs, 30.0, params[P_MIX]);
        ctrlCount = 0;
    }

    void resetState() {
        for (int g = 0; g < kNumGroups; ++g) groups[g].resetSignalState();
        ctrlCount = 0;
        silentRun = 0;
        dormant   = false;
        // Auto-clear Freeze when the host restarts the pipeline (effMainsChanged).
        // EAPO loads one instance system-wide, so a corner frozen for an old game
        // would otherwise persist and lowpass whatever runs next. The GUI checkbox
        // resyncs on its timer via syncFromHost().
        params[P_FREEZE] = 0.0f;
    }

    void ctrlUpdate() {
        bool frozen = params[P_FREEZE] > 0.5f;
        for (int g = 0; g < kNumGroups; ++g)
            groups[g].ctrl(fs, frozen, stepUp, stepDown, holdAtt, holdRel,
                           cornerMinLog, cornerMaxLog);
    }

    double cornerHz(int g) const {
        if (g < 0 || g >= kNumGroups) return 0.0;
        return std::exp2(groups[g].cornerLog);
    }

    template <typename T>
    void run(T** in, T** out, VstInt32 n) {
        // Envelope followers decaying into silence are a denormal trap.
        unsigned int csr = _mm_getcsr();
        _mm_setcsr(csr | 0x8040);      // FTZ | DAZ

        const int hang = (int)(kGateHold * fs);
        for (VstInt32 i = 0; i < n; ++i) {
            double x[8], dry[8], peak = 0.0;
            for (int c = 0; c < 8; ++c) {
                dry[c] = x[c] = in[c] ? (double)in[c][i] : 0.0;
                double a = std::fabs(dry[c]);
                if (a > peak) peak = a;
            }

            // -- idle gate --
            if (dormant) {
                if (peak < kGateThresh) {          // stay asleep: passthrough
                    for (int c = 0; c < 8; ++c) if (out[c]) out[c][i] = (T)dry[c];
                    continue;
                }
                dormant = false; silentRun = 0;    // wake ON this sample
                ctrlCount = 0;                     // apply coefficients now
                smClean.snap(params[P_CLEAN]);  smSmooth.snap(params[P_SMOOTH]);
                smRegen.snap(params[P_REGEN]);  smTrans.snap(params[P_TRANS]);
                smMix.snap(params[P_MIX]);
            } else if (peak < kGateThresh) {
                if (++silentRun >= hang) {
                    // Decaying state has flushed to ~0 by now; make it exact.
                    // resetState() keeps the learned corners (and clears the
                    // gate fields, so re-set dormant after).
                    resetState();
                    dormant = true;
                    for (int c = 0; c < 8; ++c) if (out[c]) out[c][i] = (T)dry[c];
                    continue;
                }
            } else silentRun = 0;

            if (ctrlCount-- <= 0) { ctrlUpdate(); ctrlCount = kCtrl - 1; }

            double cln = smClean.next(params[P_CLEAN]);
            double smo = smSmooth.next(params[P_SMOOTH]);
            double reg = smRegen.next(params[P_REGEN]);
            double trn = smTrans.next(params[P_TRANS]);
            double mix = smMix.next(params[P_MIX]);

            for (int g = 0; g < kNumGroups; ++g)
                groups[g].tick(x, dry, kMS100, kMS200, cln, smo, reg, trn);

            for (int c = 0; c < 8; ++c) {
                if (!out[c]) continue;
                if (c == CH_LFE) { out[c][i] = (T)dry[c]; continue; }
                double y = dry[c] + mix * (x[c] - dry[c]);
                out[c][i] = (T)softclip(y);
            }
        }
        _mm_setcsr(csr);
    }
};

// ------------------------------------------------------------- editor GUI ----
#if defined(_WIN32)
static VstRect g_edRect = { 0, 0, 330, 460 };

enum { kResetId = 200, kFreezeId = 201, kStatusTimer = 1 };

static HINSTANCE dllInstance() {
    HMODULE h = nullptr;
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                       GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       (LPCSTR)&g_edRect, &h);
    return (HINSTANCE)h;
}

void ReGen::refreshValue(int i) {
    if (!edValue[i]) return;
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.0f %%", params[i] * 100.0);
    SetWindowTextA(edValue[i], buf);
}

void ReGen::refreshStatus() {
    if (!edStatus) return;
    char buf[96];
    std::snprintf(buf, sizeof buf,
                  "Rolloff   F %.1fk   C %.1fk   S %.1fk   B %.1fk",
                  cornerHz(G_FRONT) / 1000.0, cornerHz(G_CENTER) / 1000.0,
                  cornerHz(G_SIDE)  / 1000.0, cornerHz(G_BACK)   / 1000.0);
    SetWindowTextA(edStatus, buf);
}

static LRESULT CALLBACK EditorWndProc(HWND h, UINT msg, WPARAM wp, LPARAM lp) {
    if (msg == WM_CREATE) {
        CREATESTRUCTA* cs = (CREATESTRUCTA*)lp;
        SetWindowLongPtrA(h, GWLP_USERDATA, (LONG_PTR)cs->lpCreateParams);
        return 0;
    }
    ReGen* p = (ReGen*)GetWindowLongPtrA(h, GWLP_USERDATA);
    if (msg == WM_HSCROLL && p) {
        HWND tb = (HWND)lp;
        for (int i = 0; i < kNumSliders; ++i) {
            if (tb == p->edSlider[i]) {
                int pos = (int)SendMessageA(tb, TBM_GETPOS, 0, 0);
                p->params[i] = (float)(pos / 1000.0);
                p->refreshValue(i);
                p->notifyAutomate(i);
                break;
            }
        }
        return 0;
    }
    if (msg == WM_COMMAND && p) {
        if (LOWORD(wp) == kResetId) { p->resetDefaults(); return 0; }
        if (LOWORD(wp) == kFreezeId) {
            bool on = SendMessageA(p->edFreeze, BM_GETCHECK, 0, 0) == BST_CHECKED;
            p->params[P_FREEZE] = on ? 1.0f : 0.0f;
            p->notifyAutomate(P_FREEZE);
            return 0;
        }
    }
    if (msg == WM_TIMER && p && wp == kStatusTimer) {
        p->refreshStatus();
        p->syncFromHost();     // pick up DAW automation / preset loads
        return 0;
    }
    if (msg == WM_CTLCOLORSTATIC) return (LRESULT)GetSysColorBrush(COLOR_BTNFACE);
    return DefWindowProcA(h, msg, wp, lp);
}

void ReGen::openEditor(HWND parent) {
    if (edContainer) return;
    HINSTANCE inst = dllInstance();

    INITCOMMONCONTROLSEX icc = { sizeof(icc), ICC_BAR_CLASSES };
    InitCommonControlsEx(&icc);

    static const char* kClass = "ReGenEditorWnd";
    WNDCLASSEXA wc;
    if (!GetClassInfoExA(inst, kClass, &wc)) {
        ZeroMemory(&wc, sizeof wc);
        wc.cbSize        = sizeof wc;
        wc.lpfnWndProc   = EditorWndProc;
        wc.hInstance     = inst;
        wc.lpszClassName = kClass;
        wc.hCursor       = LoadCursor(nullptr, IDC_ARROW);
        wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
        RegisterClassExA(&wc);
    }

    edContainer = CreateWindowExA(0, kClass, "", WS_CHILD | WS_VISIBLE,
                                  0, 0, g_edRect.right, g_edRect.bottom,
                                  parent, nullptr, inst, this);
    if (!edContainer) return;

    CreateWindowExA(0, "STATIC", "ReGen  -  lossy-audio reconstruction",
                    WS_CHILD | WS_VISIBLE,
                    16, 8, 300, 18, edContainer, nullptr, inst, nullptr);
    CreateWindowExA(0, "BUTTON", "Defaults",
                    WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                    374, 6, 74, 22, edContainer,
                    (HMENU)(intptr_t)kResetId, inst, nullptr);

    const char* names[kNumSliders] = { "Cleanup", "Smooth", "Regen", "Attack", "Mix" };
    for (int i = 0; i < kNumSliders; ++i) {
        int y = 40 + i * 48;
        CreateWindowExA(0, "STATIC", names[i], WS_CHILD | WS_VISIBLE,
                        16, y, 96, 20, edContainer, nullptr, inst, nullptr);
        HWND tb = CreateWindowExA(0, TRACKBAR_CLASSA, "",
                        WS_CHILD | WS_VISIBLE | TBS_HORZ | TBS_NOTICKS,
                        116, y - 2, 250, 28, edContainer,
                        (HMENU)(intptr_t)(100 + i), inst, nullptr);
        SendMessageA(tb, TBM_SETRANGE, TRUE, MAKELONG(0, 1000));
        SendMessageA(tb, TBM_SETPOS, TRUE, (LPARAM)(params[i] * 1000.0f));
        edSlider[i] = tb;
        edValue[i] = CreateWindowExA(0, "STATIC", "", WS_CHILD | WS_VISIBLE,
                        374, y, 74, 20, edContainer, nullptr, inst, nullptr);
        refreshValue(i);
    }

    int y = 40 + kNumSliders * 48;
    edFreeze = CreateWindowExA(0, "BUTTON", "Freeze",
                    WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
                    16, y, 90, 20, edContainer,
                    (HMENU)(intptr_t)kFreezeId, inst, nullptr);
    SendMessageA(edFreeze, BM_SETCHECK,
                 params[P_FREEZE] > 0.5f ? BST_CHECKED : BST_UNCHECKED, 0);
    edStatus = CreateWindowExA(0, "STATIC", "", WS_CHILD | WS_VISIBLE,
                    116, y + 2, 332, 20, edContainer, nullptr, inst, nullptr);
    refreshStatus();
    SetTimer(edContainer, kStatusTimer, 300, nullptr);
}

void ReGen::closeEditor() {
    if (edContainer) {
        KillTimer(edContainer, kStatusTimer);
        DestroyWindow(edContainer);
        edContainer = nullptr;
    }
    for (int i = 0; i < kNumSliders; ++i) { edSlider[i] = nullptr; edValue[i] = nullptr; }
    edFreeze = nullptr; edStatus = nullptr;
}

void ReGen::resetDefaults() {
    setDefaultParams();
    for (int i = 0; i < kNumSliders; ++i) {
        if (edSlider[i]) SendMessageA(edSlider[i], TBM_SETPOS, TRUE,
                                      (LPARAM)(params[i] * 1000.0f));
        refreshValue(i);
    }
    if (edFreeze) SendMessageA(edFreeze, BM_SETCHECK, BST_UNCHECKED, 0);
    for (int i = 0; i < kNumParams; ++i) notifyAutomate(i);
}

// TBM_SETPOS does not generate WM_HSCROLL, so this can't feedback-loop.
void ReGen::syncFromHost() {
    for (int i = 0; i < kNumSliders; ++i) {
        if (!edSlider[i]) continue;
        int want = (int)std::lround(params[i] * 1000.0f);
        if ((int)SendMessageA(edSlider[i], TBM_GETPOS, 0, 0) != want) {
            SendMessageA(edSlider[i], TBM_SETPOS, TRUE, (LPARAM)want);
            refreshValue(i);
        }
    }
    if (edFreeze) {
        WPARAM want = params[P_FREEZE] > 0.5f ? BST_CHECKED : BST_UNCHECKED;
        if (SendMessageA(edFreeze, BM_GETCHECK, 0, 0) != (LRESULT)want)
            SendMessageA(edFreeze, BM_SETCHECK, want, 0);
    }
}
#endif // _WIN32

// ---------------------------------------------------- host entry helpers ----
static void setParameter(AEffect* e, VstInt32 index, float value) {
    ReGen* p = (ReGen*)e->object;
    if (!p) return;
    if (index >= 0 && index < kNumParams)   // DAW automation can stray past [0,1]
        p->params[index] = value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
}
static float getParameter(AEffect* e, VstInt32 index) {
    ReGen* p = (ReGen*)e->object;
    return (p && index >= 0 && index < kNumParams) ? p->params[index] : 0.0f;
}
static void processReplacing(AEffect* e, float** in, float** out, VstInt32 n) {
    ((ReGen*)e->object)->run<float>(in, out, n);
}
static void processDoubleReplacing(AEffect* e, double** in, double** out, VstInt32 n) {
    ((ReGen*)e->object)->run<double>(in, out, n);
}

// Bounded copy that writes ONLY the needed bytes + terminator (never pads to
// cap). VST2 param strings get only kVstMaxParamStrLen (8) bytes; strncpy's
// zero-fill would overrun them.
static void copyStr(void* dst, const char* s, size_t cap) {
    if (!dst || cap == 0) return;
    char* d = (char*)dst;
    size_t i = 0;
    for (; s[i] && i + 1 < cap; ++i) d[i] = s[i];
    d[i] = 0;
}

enum {
    kMaxParamStr   = 8,    // kVstMaxParamStrLen
    kMaxProgName   = 24,
    kMaxEffectName = 32,
    kMaxVendorStr  = 64,
    kMaxProductStr = 64
};

static VstIntPtr dispatcher(AEffect* e, VstInt32 opcode, VstInt32 index,
                            VstIntPtr value, void* ptr, float opt) {
    ReGen* p = (ReGen*)e->object;
    if (!p) return 0;                 // defensive: never dispatch into a dead instance
    switch (opcode) {
    case effOpen:  return 0;
    case effClose:
        if (p) {
#if defined(_WIN32)
            p->closeEditor();
#endif
            // e == &p->effect, so e dangles the moment p is freed -- do NOT
            // touch it after delete (the old `e->object = nullptr` was a
            // use-after-free write). Hosts must not dispatch after effClose.
            delete p;
        }
        return 0;

    case effSetSampleRate: p->setSampleRate((double)opt); return 0;
    case effSetBlockSize:  return 0;
    case effMainsChanged:  if (value) p->resetState(); return 0;

#if defined(_WIN32)
    // Equalizer APO calls effEditGetRect and dereferences the returned pointer
    // WITHOUT null-checking, so always hand back a valid rect.
    case effEditGetRect:
        if (ptr) *(VstRect**)ptr = &g_edRect;
        return 1;
    case effEditOpen:
        if (p) p->openEditor((HWND)ptr);
        return 1;
    case effEditClose:
        if (p) p->closeEditor();
        return 1;
    case effEditIdle:
        return 0;
#endif

    case effGetParamName:
    case effGetParamLabel:
    case effGetParamDisplay: {
        char buf[32] = {0};
        if (index < 0 || index >= kNumParams) { copyStr(ptr, "", kMaxParamStr); return 0; }
        if (opcode == effGetParamName) {
            // Must fit kVstMaxParamStrLen (8 bytes -> 7 chars).
            const char* names[] = { "Cleanup", "Smooth", "Regen", "Attack", "Mix", "Freeze" };
            copyStr(ptr, names[index], kMaxParamStr);
        } else if (opcode == effGetParamLabel) {
            const char* labels[] = { "%", "%", "%", "%", "%", "" };
            copyStr(ptr, labels[index], kMaxParamStr);
        } else { // display
            if (index == P_FREEZE) {
                copyStr(ptr, p->params[P_FREEZE] > 0.5f ? "On" : "Off", kMaxParamStr);
            } else {
                std::snprintf(buf, sizeof buf, "%.0f", p->params[index] * 100.0);
                copyStr(ptr, buf, kMaxParamStr);
            }
        }
        return 0;
    }

    case effCanBeAutomated: return 1;

    // Private readout for tooling: index 'Roff', value = group 0..3
    // (front/center/side/back) -> detected corner in Hz.
    case effVendorSpecific:
        if (index == REGEN_ROLLOFF_QUERY && p)
            return (VstIntPtr)std::lround(p->cornerHz((int)value));
        return 0;

    case effGetEffectName:    copyStr(ptr, "ReGen", kMaxEffectName); return 1;
    case effGetProductString: copyStr(ptr, "ReGen lossy reconstruction", kMaxProductStr); return 1;
    case effGetVendorString:  copyStr(ptr, "daede", kMaxVendorStr);   return 1;
    case effGetVendorVersion: return 1000;
    case effGetPlugCategory:  return kPlugCategEffect;
    case effGetVstVersion:    return 2400;

    case effCanDo: return 0;

    // Legacy hosts probe this before trusting the plugin.
    case effIdentify_DEPRECATED: return CCONST('N', 'v', 'E', 'f');

    // DAWs enumerate program names for their preset dropdowns.
    case effGetProgramNameIndexed:
        if (index == 0) { copyStr(ptr, "Default", kMaxProgName); return 1; }
        return 0;

    case effGetProgramName: copyStr(ptr, "Default", kMaxProgName); return 0;
    case effSetProgramName: return 0;
    case effGetProgram:     return 0;
    case effSetProgram:     return 0;

    default: return 0;
    }
}

#if defined(_WIN32)
#define VST_EXPORT extern "C" __declspec(dllexport)
#else
#define VST_EXPORT extern "C" __attribute__((visibility("default")))
#endif

VST_EXPORT AEffect* VSTPluginMain(audioMasterCallback audioMaster) {
    ReGen* p = new ReGen();
    AEffect* e = &p->effect;
    std::memset(e, 0, sizeof(AEffect));

    e->magic      = kEffectMagic;
    e->dispatcher = dispatcher;
    e->setParameter = setParameter;
    e->getParameter = getParameter;
    e->processReplacing       = processReplacing;
    e->processDoubleReplacing = processDoubleReplacing;

    e->numPrograms = 1;
    e->numParams   = kNumParams;
    e->numInputs   = 8;              // 7.1 in
    e->numOutputs  = 8;              // 7.1 out (role-aware, in place)
    e->flags       = effFlagsCanReplacing | effFlagsCanDoubleReplacing
                   | effFlagsHasEditor;
    e->uniqueID    = CCONST('R', 'e', 'G', 'n');
    e->version     = 1000;
    e->object      = p;

    p->master = audioMaster;
    return e;
}

#if defined(_WIN32)
// Pre-2.4 hosts GetProcAddress("main"); a C++ function can't be named `main`,
// so export it under that name via an asm label.
VST_EXPORT AEffect* vstLegacyMain(audioMasterCallback audioMaster) __asm__("main");
AEffect* vstLegacyMain(audioMasterCallback audioMaster) {
    return VSTPluginMain(audioMaster);
}
#endif
