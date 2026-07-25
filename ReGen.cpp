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
//             swirl/hash (scaled by Repair)
//   warble    detector, not a repair stage: log-envelope of the top coded
//             octave minus the octave below it, bandpassed 8-40 Hz (codec
//             frame rates), RMS. Shared musical modulation cancels; what
//             remains is band-local bit-allocation flutter. The score slides
//             the whole dull-and-replace boundary DOWN (up to 0.4 oct), so a
//             warbling top octave gets dulled and rebuilt by translation --
//             the translated band inherits the donor's stability, which is
//             the only real un-warble (scaled by Repair)
//   regen     squaring translation: the octave below the corner, squared
//             (sum terms land one octave up, difference terms die in the
//             injection highpass), envelope-normalized, level closed-loop
//             matched to the source's own spectral slope extrapolated one
//             octave -- so the rebuilt band continues the recording rather
//             than shelving flat. Injection stops at the 16.5 kHz anchor
//             (scaled by Repair; Repair 1.0 = slope target)
//   transient zero-lookahead attack boost masks pre-echo smear (scaled by Repair)
//
// THREE deficit factors derived from the corner scale the chain, so pristine
// content gets an automatic light touch -- no preset system needed. They are
// split because the stages repair DIFFERENT defects and are not equally
// audible at a high corner:
//
//   deficit     (cleanup, regen) -- both place their filters RELATIVE to the
//               corner (corner*1.25 and corner*1.08), so when the corner is
//               high they act high, above anything anyone can hear. They are
//               self-limiting and can afford an earlier ramp.
//               0 at >=17.5 kHz, 1 at <=15 kHz.
//
//   audDeficit  (smoother) -- reaches into the AUDIBLE band wherever the corner
//               sits, working from corner*0.55 (10 kHz at a 18 kHz corner). It
//               must not engage until the source is audibly damaged.
//               0 at >=16.5 kHz, 1 at <=14 kHz.
//
//   trnGate     (transient) -- not a severity ramp but a LOSSLESS DETECTOR.
//               Pre-echo is temporal, so the corner cannot rank how badly a
//               codec smeared transients; it can, however, tell lossless from
//               lossy, because lossless pins the servo against its ceiling.
//               Measured as octaves below that ceiling, so it stays correct at
//               44.1 kHz where the ceiling is 19404 Hz, not 20 kHz.
//               0 at the ceiling, 1 at kTrnGateOct below it.
//
// 16.5 kHz is the audibility anchor: a lowpass above it is inaudible on music.
// A 320k MP3 therefore idles cleanup, regen and the smoother rather than
// applying broadband gain to content whose only defect is at 19 kHz.
//
// Regen is additionally skipped outright once corner*1.08 reaches the anchor --
// injecting content nobody can hear is pure cost. See regenAudible.
//
// No channel roles (Windows 7.1: 0 FL 1 FR 2 FC 3 LFE 4 BL 5 BR 6 SL 7 SR):
// every channel gets the identical chain, because the plugin runs ahead of
// any upmixer and positional sources can appear in any channel -- role
// special-casing means positional timbre inconsistency. LFE alone passes
// through untouched. Channels are still grouped into pairs for detection:
// dynamics gains are shared per pair so imaging never wanders.
//
// Params: 0 Repair  1 Restore  2 Freeze(0/1)
//
// Build: see build_mingw.bat.  License: BSD-2-Clause.

#include "vst2_min.h"
#include <cmath>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <vector>
#include <xmmintrin.h>   // MXCSR access for FTZ/DAZ denormal flushing

#if defined(_WIN32)
#include <windows.h>
#include <commctrl.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// One knob. Every damage-proportional stage -- dull, warble detection,
// translation, Air tilt, transient boost -- is a facet of the same repair
// and scales from the single Repair control; the stages' relative
// proportions are design constants, not user homework. Restore stays
// separate because it is a real decision (it costs latency and needs host
// PDC); Freeze is the servo-hold utility checkbox.
static const VstInt32 kNumParams  = 3;
static const int      kNumSliders = 2;                 // Freeze is a checkbox
enum { P_REPAIR = 0, P_RESTORE, P_FREEZE };
static const double kTrnScale = 0.58;   // Attack share of Repair (0.6 -> 0.35)

enum { CH_FL = 0, CH_FR, CH_FC, CH_LFE, CH_BL, CH_BR, CH_SL, CH_SR };

// effVendorSpecific query: index='Roff', value=group 0..3 -> corner in Hz.
#define REGEN_ROLLOFF_QUERY CCONST('R', 'o', 'f', 'f')
// index='Dorm' -> bitmask of groups currently asleep (bit g = group g). Sleeping
// is a pure CPU optimisation with no signal-level signature, so this is the only
// way a test can assert it actually happens rather than merely not breaking.
#define REGEN_DORMANT_QUERY CCONST('D', 'o', 'r', 'm')
// index='Dbg0', value=field -> group-0 stage internals scaled 1e9 (tooling
// diagnostics only): 0 msDon 1 msLow 2 msInj 3 gInj*1e3 4 warble*1e3 5 slide*1e3
#define REGEN_DEBUG_QUERY   CCONST('D', 'b', 'g', '0')

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

// Deficit ramps (see the header). Cleanup and regen place their filters
// relative to the corner and are self-limiting, so they ramp earlier; the
// smoother and the transient boost act in the audible band regardless of the
// corner, so they hold off until a lowpass would actually be audible.
// A lowpass at or above kAudIdleHz is inaudible on music -- that is the anchor.
// Every stage is scaled by TWO things, and keeping them separate is the whole
// design:
//
//   damage      -- is this source damaged at all? Measured as octaves below the
//                  servo's ceiling, NOT as an absolute frequency. The ceiling is
//                  min(20 kHz, 0.44*fs) and so moves with the HOST sample rate:
//                  20000 Hz at 48k (Equalizer APO always), 19404 at 44.1k,
//                  9702 at 22.05k. Absolute thresholds would therefore be
//                  unreachable at low host rates -- offline remastering of a
//                  22 kHz asset would peg every stage to full strength forever,
//                  by construction, whatever the audio was. (Content ORIGIN is
//                  a separate matter: a CD rip resampled to a 48k device keeps
//                  its 22.05 kHz wall, sails past the cap, and reads lossless.)
//                  Lossless pins the servo at the ceiling and scores exactly 0;
//                  every lossy format sits measurably below. A ramp, not a step:
//                  bandwidth can rank pre-echo and hash audibility (their LEVEL
//                  falls as bitrate rises) but it cannot resolve adjacent
//                  bitrates on a knife edge.
//
//                  Pleasant side effect: at fs = 22.05k a full-band asset reads
//                  damage 0, correctly -- regen would inject above corner*1.08
//                  with Nyquist at 11.025 kHz, so there is no room to synthesize
//                  into. The same asset through a 48k device resamples up, lands
//                  at a ~10.4 kHz corner under a 20 kHz ceiling, and gets the
//                  full treatment. "Bandlimited with headroom to repair" and
//                  "fills the available band" fall out without being told apart.
//
//   audibility  -- does this stage's action land where anyone can hear it?
//                  Cleanup lowpasses at corner*1.25 and regen injects above
//                  corner*1.08, so both march upward with the corner and can
//                  end up working entirely above the hearing limit. Weighted by
//                  where that action band actually falls. The smoother (from
//                  corner*0.55) and the transient boost (broadband) always land
//                  in audible territory, so they carry no such weight.
//
// 16.5 kHz is the anchor throughout: a lowpass above it is inaudible on music.
static const double kAudIdleHz  = 16500.0;   // audibility anchor
static const double kAudSpanHz  =  3000.0;   // ramp width below the anchor
static const double kDamageOct  =     0.40;  // octaves below ceiling -> full

// The smoother needs a LATER onset than the rest. It clamps top-octave warble,
// and warble is a bit-starvation artifact -- coefficients flickering on and off
// between frames when the encoder runs out of bits. It does not meaningfully
// exist at high bitrate, so engaging it at a 320k corner costs CPU to chase
// something that is not there.
static const double kSmoothOnsetOct = 0.20;  // octaves below ceiling before it starts

// The transient boost has NO corner-derived factor at all. Pre-echo spread is
// set by the codec's transform block length, not by its bitrate or its lowpass:
// an MP3 short block smears over 1152/384 samples (26.1 / 8.7 ms at 44.1 kHz),
// Vorbis over 2048/256 (46.4 / 5.8 ms), and those lengths do not change when
// you raise the bitrate. Backward masking only covers a few ms -- premasking is
// usually put at 0.5-2 ms, effective under 5 ms, against 10-50 ms forward -- so
// even a correctly switched MP3 short block stays partly exposed. Which is why
// transients fail at 192k and 320k alike, and why no setting of a bandwidth
// ramp can gate this stage correctly. It idles on its own: the onset term goes
// to zero on steady material, exactly as the Crystalizer did.
static const double kTrnSplitHz = 2000.0;   // transient LF/HF crossover

// ...but it must still idle on lossless. The corner cannot rank HOW badly a
// codec smeared transients -- 192k and 320k are not separable that way -- yet
// it answers a much easier question reliably: is this lossless at all? Lossless
// pins the servo hard against its ceiling (measured: exactly cornerMax for both
// 48 kHz native and CD-sourced material), while every lossy format sits
// measurably below it, the nearest being high-quality Ogg at ~0.065 octaves
// down. So the gate is expressed as distance BELOW THE CEILING, not as an
// absolute frequency -- which also keeps it correct at 44.1 kHz, where the
// ceiling is 0.44*fs = 19404 Hz rather than 20 kHz.
//
// It is a gentle ramp, not a step, and the reason is worth stating precisely.
// Block length fixes the DURATION of the smear and does not change with
// bitrate. But audibility is the smear's LEVEL against the masking threshold,
// and level does fall as bitrate rises -- which bandwidth tracks. So the corner
// can legitimately rank pre-echo audibility even though it cannot rank pre-echo
// duration. What it cannot do is resolve adjacent bitrates on a knife edge,
// hence a ramp spanning ~0.4 octaves rather than a cliff. Calibration target:
// 320k MP3 lands near half a dB (a near-transparent format should get a
// near-inaudible correction), 192k clearly engaged, 128k and below full.
static const double kTrnGateOct = 0.40;     // octaves below ceiling -> full

// Idle gate, shared by the plugin-wide gate and the per-group ones.
static const double kGateThresh = 1e-5;     // ~-100 dBFS peak
static const double kGateHold   = 0.25;     // seconds below thresh to sleep

// Warble-detection constants. The detector compares the top coded octave's
// envelope against the donor octave below it in the 8-40 Hz modulation band
// (Vorbis long frames tick at ~11 Hz, MP3 granules at ~38 Hz; musical
// dynamics and noise-floor breathing sit mostly below 8 Hz -- measured with
// tools/warble_meter, which is also this detector's calibration reference).
// The RMS maps to a 0..1 score across the narrowband JND range and slides
// the dull-and-replace boundary down by up to kWarbleMaxOct.
static const double kWarbleHpHz   = 8.0;    // modulation highpass
static const double kWarbleLpHz   = 40.0;   // modulation lowpass
static const double kWarbleRmsSec = 0.5;    // score integration time
static const double kWarbleDb0    = 1.0;    // dB RMS -> score 0 (JND floor)
static const double kWarbleDb1    = 4.0;    // dB RMS -> score 1
static const double kWarbleMaxOct = 0.4;    // max boundary slide, octaves

// ------------------------------------------------------------- tunables ----
// The internal design constants the tuning harness may override at runtime
// via effVendorSpecific 'Tune' (index='Tune', value=id, opt=new value;
// value=-1 resets all). Tooling surface only: shipped configs never set
// these, and the defaults ARE the shipped behavior. Process-global.
enum {
    TN_TRNSCALE = 0,   // Attack share of Repair
    TN_AIRTEMPL,       // Air template slope, dB/oct
    TN_AIRCAP,         // Air max boost, dB
    TN_TMPSLOPE,       // temper dB->factor slope
    TN_TMPFLOOR,       // temper floor
    TN_SLOPECAP,       // translation slope-ratio cap
    TN_REACH,          // injection highpass, fraction of eff corner
    TN_WRBDB0,         // warble score floor, dB RMS
    TN_WRBDB1,         // warble score ceiling, dB RMS
    TN_RSTRATIO,       // Restore attack:pre-window ratio
    TN_NRMREL,         // translation normalizer release, ms
    TN_WRBMAXOCT,      // max warble boundary slide, oct
    TN_RSTMAXDB,       // Restore deepest cut, dB
    TN_RSTSLOWREL,     // Restore detector slow-envelope release, ms
    NTUNE
};
static const double kTuneDefault[NTUNE] = {
    0.58, -2.0, 3.0, 0.25, 0.30, 1.00, 0.85, 1.0, 4.0, 6.0, 30.0, 0.4, 14.0, 250.0
};
// First 12: the 2026-07-25 face-off winner (blind human ABX + verdict vs
// the hand-tuned originals; see DESIGN.md, derby v2) -- except [9]
// TN_RSTRATIO, which the race never exercised (Restore was off) and the
// 2026-07-25 Restore pass set deliberately. Levers 9/12/13 per that pass
// (RESTORE_RESEARCH.md + castanet grid, judge-scored): fire at ~12 dB
// attack:pre ratio, cut 6 dB toward the floor not silence, slow release
// 25 ms so the detector recovers between roll hits.
static double g_tune[NTUNE] = {
    0.20, -0.339357, 2.995877, 0.085655, 0.536270, 0.723904, 0.859073,
    0.864070, 7.138318, 4.0, 94.450482, 0.423601, 6.0, 25.0
};
#define REGEN_TUNE_QUERY CCONST('T', 'u', 'n', 'e')

// Air: correct a darkened surviving band toward what real content looks
// like. The template is the average per-octave HF slope of the corpus's
// full-bandwidth music references above 2 kHz, measured 2026-07-24 with
// tools/warble_meter: -2 dB/oct (the flat electronic reference sits above
// it and correctly receives none). Correction closes the gap between the
// source's measured slope and the template, capped low per the aggression
// lesson, and never cuts -- a source at or above the template gets nothing.
static const double kAirTemplDb = -2.0;     // template slope, dB/oct
static const double kAirCapDb   =  3.0;     // max boost, dB

// ------------------------------------------------------------ role group ----
// A group is one detection unit: a stereo pair or a mono channel. Detection
// runs on the pair's mono sum; time-varying GAINS derived from shared
// envelopes are applied identically to both channels so imaging stays put.
struct Group {
    int  chA = 0, chB = -1;               // chB < 0 -> mono group

    // --- rolloff servo ---
    Biquad refBP;                         // broad 1-4 kHz mid reference
    HP12   probeLo, probeHi;              // straddle the corner estimate
    double msRef = 0, msLo = 0, msHi = 0; // 200 ms mean-square meters
    double msHiF = 0;                     // 20 ms fast meter: up-gate evidence
    double kMS20c = 0;                    // its coefficient (needs fs)
    int    upRun = 0;                     // consecutive alive blocks (debounce)
    double hRef = 0, hLo = 0, hHi = 0;    // gated peak-hold versions
    double wideMS = 0;                    // wideband gate meter
    double cornerLog = 0, appliedLog = -1;
    double damage  = 0;                   // 0 = pristine (servo pinned), 1 = wrecked
    double dSmooth = 0;                   // damage, but starting later (warble)
    double wClean  = 0;                   // audibility of cleanup's action band
    double wRegen  = 0;                   // audibility of regen's injection band

    // --- per-group idle gate ---
    // The plugin-wide gate keys off the peak across all EIGHT channels, so one
    // active channel kept every group running: measured, 6 digitally silent
    // channels cost exactly as much as 6 active ones. In Equalizer APO ReGen
    // sits ahead of any upmixer, so stereo content leaves 3 of 4 groups
    // processing nothing at all. Each group now sleeps on its own pair.
    int    silentRun = 0;
    bool   dormant   = false;
    int    hang      = 0;                  // samples of sub-threshold before sleep

    // --- adaptive processing ---
    Biquad    cleanLP[2];
    Biquad    donHP[2], donLP[2];         // per-channel donor octave [c/2, c]
    Biquad    injHP[2][2];                // 4th-order injection highpass
    Biquad    injLP17[2];                 // synthesis stop above the anchor
    // mono metering path: donor octave + the octave below it, for the slope
    // extrapolation target and the warble detector's donor comparison
    Biquad    donMHP, donMLP, lowMHP, lowMLP;
    EnvAR     envDon, envLow;             // fast envelopes for warble detection
    // Pair-power donor meters for the translation level and normalizer. The
    // mono-sum meters above are fine for RATIOS (slope, warble log-diff:
    // stereo cancellation divides out) but not for ABSOLUTE level -- a wide
    // stereo mix partially cancels in the sum, and a normalizer keyed to it
    // collapses exactly on wide passages (measured: the clean PCM case
    // injected 40 dB under the point-stereo ogg of the same music).
    EnvAR     envDonP;                    // shared pair envelope (normalizer)
    double    msDonP = 0;                 // pair-power donor MS (level target)
    double    msDon = 0, msLow = 0;       // slope meters (200 ms MS, mono sum)
    double    msInj = 0;                  // translated-signal meter (pre-gain)
    double    gInj  = 0;                  // closed-loop injection gain, smoothed
    double    gAir  = 0;                  // Air band boost (linear add gain)
    // warble detector state (control rate). Each band's log-envelope gets its
    // own 8-40 Hz modulation chain; the ratio flutter is their difference and
    // the cross meter gives the correlation SIGN -- the arp discriminator.
    double    tLp8 = 0, tLp40 = 0;        // top coded octave modulation
    double    lLp8 = 0, lLp40 = 0;        // donor-below octave modulation
    double    msT = 0, msL = 0, msX = 0;  // band mod power + cross
    double    msWrb = 0;                  // ratio-flutter power
    double    warble = 0;                 // 0..1 score
    double    slideOct = 0;               // current boundary slide, octaves

    // --- transient restoration, Crystalizer-shaped ---
    // TWO bands with independent detectors and independent gains. Pre-echo is
    // high-frequency dominant (that is where the codec allocates fewest bits),
    // and a single broadband gain "repairing" it was measured putting +0.59 dB
    // onto a 200 Hz burst. The X-Fi Crystalizer this stage descends from ran
    // separate low- and high-frequency energy flux with proportionally weighted
    // per-band boosts, for exactly this reason. Complementary 1st-order split
    // (hf = x - lp) so the bands recombine to unity when the gains are equal.
    OnePoleLP trSplitD;                   // detection split (on the pair's sum)
    OnePoleLP trSplitA[2];                // application split (per channel)
    EnvAR     trFastLo, trSlowLo;
    EnvAR     trFastHi, trSlowHi;

    double aW8 = 0, aW40 = 0, aWms = 0;   // ctrl-rate detector coefficients

    void configure(double fs, double cornerMaxLog) {
        refBP.setBP(fs, 2000.0, 0.8);
        for (int c = 0; c < 2; ++c) injLP17[c].setLP(fs, 17500.0, 0.7071);
        // 1 ms attack: the translation normalizer divides by this envelope,
        // and a slower attack lets musical transients square up faster than
        // the divisor tracks -- measured as huge low-duty spikes standing in
        // for a band. Release 30 ms avoids pumping inside a modulation cycle.
        envDon.init(fs, 1.0, 30.0);   envLow.init(fs, 1.0, 30.0);
        envDonP.init(fs, 1.0, g_tune[TN_NRMREL]);
        kMS20c = 1.0 - std::exp(-1.0 / (0.020 * fs));
        double ctrlRate = fs / kCtrl;
        aW8  = 1.0 - std::exp(-2.0 * M_PI * kWarbleHpHz / ctrlRate);
        aW40 = 1.0 - std::exp(-2.0 * M_PI * kWarbleLpHz / ctrlRate);
        aWms = 1.0 - std::exp(-1.0 / (kWarbleRmsSec * ctrlRate));
        trFastLo.init(fs, 0.5, 40.0); trSlowLo.init(fs, 15.0, 140.0);
        trFastHi.init(fs, 0.5, 40.0); trSlowHi.init(fs, 15.0, 140.0);
        trSplitD.setCutoff(fs, kTrnSplitHz);
        trSplitA[0].setCutoff(fs, kTrnSplitHz);
        trSplitA[1].setCutoff(fs, kTrnSplitHz);
        hang = (int)(kGateHold * fs);
        silentRun = 0; dormant = false;
        cornerLog = cornerMaxLog;
        appliedLog = -1.0;            // force coefficient apply on next ctrl
    }

    void resetSignalState() {         // clears audio state, keeps the learned corner
        refBP.reset(); probeLo.reset(); probeHi.reset();
        msRef = msLo = msHi = msHiF = wideMS = 0.0; upRun = 0;
        for (int c = 0; c < 2; ++c) {
            cleanLP[c].reset(); donHP[c].reset(); donLP[c].reset();
            injHP[c][0].reset(); injHP[c][1].reset(); injLP17[c].reset();
        }
        donMHP.reset(); donMLP.reset(); lowMHP.reset(); lowMLP.reset();
        envDon.reset(); envLow.reset(); envDonP.reset();
        msDon = msLow = msInj = 0.0; msDonP = 0.0; gInj = 0.0; gAir = 0.0;
        tLp8 = tLp40 = lLp8 = lLp40 = 0.0;
        msT = msL = msX = msWrb = 0.0; warble = 0.0; slideOct = 0.0;
        trFastLo.reset(); trSlowLo.reset(); trFastHi.reset(); trSlowHi.reset();
        trSplitD.reset(); trSplitA[0].reset(); trSplitA[1].reset();
    }

    // Control-rate servo step + adaptive coefficient refresh.
    void ctrl(double fs, bool frozen, double stepUp, double stepDown,
              double holdAtt, double holdRel, double cornerMinLog,
              double cornerMaxLog, double smo, double reg) {
        bool gate = wideMS > 3e-7;    // ~-65 dBFS: below this, hold everything
        auto hold = [&](double& h, double e) {
            if (e > h)     h += holdAtt * (e - h);
            else if (gate) h += holdRel * (e - h);
        };
        hold(hRef, msRef); hold(hLo, msLo); hold(hHi, msHi);

        if (!frozen && gate && hRef > 1e-8) {
            double th = hRef * 2.5e-4 + 1e-12;      // -36 dB alive threshold
            // Up-steps take the 20 ms FAST meter, not the peak-hold or even
            // the 200 ms meter: the corner climbs at 2 oct/s, so any verdict
            // that outlives its evidence lets a burst carry the corner
            // through the wall -- 40 dB of decay is 0.9 s at the 200 ms
            // meter, exactly a 6 kHz -> ceiling runaway (measured: damage=0,
            // plugin fully idle, for 15 s of a 45 s brickwalled file after a
            // breakdown-then-drop). At 20 ms the same decay is 90 ms and the
            // overshoot stays ~0.2 oct. The hold stays on the DOWN gate,
            // where surviving bass-only passages is exactly what it is for.
            // ...debounced ~32 ms: a signal stopping dead is a broadband
            // step edge that rings the highpass probe, and a meter fast
            // enough to drop the runaway is fast enough to catch that ring
            // (the old peak-hold's 80 ms attack was silently the debounce).
            // Real content sustains; edges and rings do not.
            if (msHiF > th) {
                if (++upRun >= 24) cornerLog += stepUp; // live content above
            } else {
                upRun = 0;
                if (hLo < th) cornerLog -= stepDown;    // nothing at corner
            }
            if (cornerLog < cornerMinLog) cornerLog = cornerMinLog;
            if (cornerLog > cornerMaxLog) cornerLog = cornerMaxLog;
        }

        double corner = std::exp2(cornerLog);

        // How damaged: distance below the ceiling, in octaves.
        double below = cornerMaxLog - cornerLog;
        damage  = clamp01(below / kDamageOct);
        dSmooth = clamp01((below - kSmoothOnsetOct) / (kDamageOct - kSmoothOnsetOct));

        // Where each stage lands. Above the anchor there is nothing to gain --
        // equal-loudness contours roll off steeply up there, more steeply still
        // at low level, and regen's injection is level-linked -- so work done
        // there is inaudible however much of it we do. Ramps rather than hard
        // switches so two similar sources cannot fall either side of a cliff.
        wClean = clamp01((kAudIdleHz - corner * 1.25) / kAudSpanHz);
        wRegen = clamp01((kAudIdleHz - corner * 1.08) / kAudSpanHz);

        // -- warble detector (ctrl rate; the envelopes run per-sample) --
        // Log-envelope difference between the top coded octave and its donor
        // cancels shared musical modulation; the 8-40 Hz remainder is
        // band-local frame-rate flutter. dSmooth gates it to coded content
        // near a wall, where warble can exist at all.
        // Runs whenever warble CAN exist (dSmooth gate), independent of the
        // Smooth knob: the translation temper below consumes msT even when
        // the slide is disabled. smo scales the slide, not the measurement.
        if (dSmooth > 0.0) {
            double lt = std::log(envDon.v + 1e-9);
            double ll = std::log(envLow.v + 1e-9);
            tLp8 += aW8 * (lt - tLp8);  double thp = lt - tLp8;
            tLp40 += aW40 * (thp - tLp40);
            lLp8 += aW8 * (ll - lLp8);  double lhp = ll - lLp8;
            lLp40 += aW40 * (lhp - lLp40);
            double wbp = tLp40 - lLp40;                  // ratio flutter
            msWrb += aWms * (wbp * wbp - msWrb);
            msT   += aWms * (tLp40 * tLp40 - msT);
            msL   += aWms * (lLp40 * lLp40 - msL);
            msX   += aWms * (tLp40 * lLp40 - msX);
            double db = 8.6859 * std::sqrt(msWrb);       // ln-RMS -> dB RMS
            // Arp discriminator: chiptune arpeggios/PWM trade energy BETWEEN
            // octaves, so their band modulations anti-correlate -- which the
            // ratio amplifies maximally. Codec flutter is uncorrelated
            // (independent per-band bit allocation). Gate the score to zero
            // as normalized correlation approaches -0.5.
            double ncc = msX / std::sqrt(msT * msL + 1e-18);
            double arpGate = clamp01(1.0 + 2.0 * ncc);
            warble = clamp01((db - g_tune[TN_WRBDB0])
                            / (g_tune[TN_WRBDB1] - g_tune[TN_WRBDB0])) * arpGate;
        } else {
            warble = 0.0;
        }
        // The slide is the detector's only output: the whole dull-and-replace
        // boundary (cleanup lowpass, donor band, injection highpass) moves
        // down together so the flutter band gets dulled AND covered by the
        // translated band. Smoothed by the score's own 0.5 s integrator.
        slideOct = smo * warble * dSmooth * g_tune[TN_WRBMAXOCT];

        // -- closed-loop translation level --
        // Target: continue the source's own per-octave slope one octave up.
        // Open-loop in signal terms (the meters are slow), so no instability.
        // Slope capped at 1.0: a flat or rising spectrum is continued flat,
        // never amplified -- the rebuilt octave can carry at most the donor's
        // per-octave energy. Boost mistakes are the unforgiving kind.
        double slope = (msDon + 1e-12) / (msLow + 1e-12);
        if (slope < 0.05) slope = 0.05;
        if (slope > g_tune[TN_SLOPECAP]) slope = g_tune[TN_SLOPECAP];
        // Level from the pair-power meter: absolute, phase-immune. The slope
        // ratio stays mono -- stereo cancellation divides out of a ratio.
        // Tempered by the donor's own 8-40 Hz modulation: squaring DOUBLES
        // dB-domain flutter, so a donor fluttering (or arpeggiating) at 2 dB
        // yields a 4 dB-fluttering band -- audible as warble we created.
        // Full level below ~1 dB donor flutter, backed off above, floor 0.3.
        double donFlutDb = 8.6859 * std::sqrt(msT);
        double temper = clamp01(1.2 - g_tune[TN_TMPSLOPE] * donFlutDb);
        if (temper < g_tune[TN_TMPFLOOR]) temper = g_tune[TN_TMPFLOOR];
        double targetMS = msDonP * slope * temper * (reg * reg);

        // -- Air: close the surviving band's tilt gap toward the template --
        // Self-limiting (gap <= 0 -> nothing) and boost-only; the donor band
        // [c/2, c] is the surviving band's top, where truncation darkening
        // lives. Audibility-weighted by where that band actually sits.
        double slopeDb = 10.0 * std::log10(slope);
        double gapDb = g_tune[TN_AIRTEMPL] - slopeDb;
        if (gapDb < 0.0) gapDb = 0.0;
        if (gapDb > g_tune[TN_AIRCAP]) gapDb = g_tune[TN_AIRCAP];
        double wAir = clamp01((kAudIdleHz - corner) / kAudSpanHz);
        gAir = std::pow(10.0, gapDb * reg * damage * wAir / 20.0) - 1.0;
        double g = std::sqrt(targetMS / (msInj + 1e-12));
        if (g > 6.0) g = 6.0;
        gInj += 0.15 * (g - gInj);

        double effLog = cornerLog - slideOct;
        if (std::fabs(effLog - appliedLog) > 5e-4) {
            appliedLog = effLog;
            double eff = std::exp2(effLog);
            probeLo.set(fs, corner);
            probeHi.set(fs, std::min(corner * 1.30, fs * 0.47));
            // slope/warble meters key off the unslid corner: the slope is a
            // property of the source, not of where we cut
            donMHP.setHP(fs, corner * 0.50, 0.7071);
            donMLP.setLP(fs, corner,        0.7071);
            lowMHP.setHP(fs, corner * 0.25, 0.7071);
            lowMLP.setLP(fs, corner * 0.50, 0.7071);
            int nch = (chB >= 0) ? 2 : 1;
            for (int c = 0; c < nch; ++c) {
                cleanLP[c].setLP(fs, eff * 1.25, 0.7071);
                donHP[c].setHP(fs, eff * 0.50, 0.7071);
                donLP[c].setLP(fs, eff,        0.7071);
                // Injection starts BELOW the corner estimate: the servo's
                // probes read a clean tonal wall up to ~0.25 oct high (skirt
                // leakage; the harness measures a 10k wall as ~11.7k), and an
                // injection placed above that bias leaves a gap between the
                // real wall and the rebuilt band -- measured as a still-dead
                // octave on resampled PCM while ogg filled fine. The 0.23 oct
                // reach-down guarantees coverage; when the estimate is honest
                // the overlap sits in the coded band's dying top octave and
                // the closed-loop level target absorbs the sum.
                injHP[c][0].setHP(fs, eff * g_tune[TN_REACH], 0.5412);
                injHP[c][1].setHP(fs, eff * g_tune[TN_REACH], 1.3066);
            }
        }
    }

    // Per-sample processing. x[] holds the working samples for all 8 channels
    // (only ours are touched); dry[] is the untouched input. kMS100/kMS200 are
    // the meter one-pole coefficients; cln/smo/reg/trn are the smoothed knobs.
    inline void tick(double* x, const double* dry, double kMS100, double kMS200,
                     double cln, double reg, double trn) {
        int a = chA, b = chB;

        // -- per-group idle gate --
        // Leaves x[] untouched, so the group's channels pass through dry. The
        // wake test runs before the skip, so the first audible sample after
        // silence is processed normally. resetSignalState() zeroes the meters,
        // which closes ctrl()'s gate and parks the learned corner.
        {
            double pk = std::fabs(dry[a]);
            if (b >= 0) { double t = std::fabs(dry[b]); if (t > pk) pk = t; }
            if (dormant) {
                if (pk < kGateThresh) return;
                dormant = false; silentRun = 0;
            } else if (pk < kGateThresh) {
                if (++silentRun >= hang) {
                    resetSignalState(); dormant = true; silentRun = 0; return;
                }
            } else silentRun = 0;
        }

        double xa = x[a], xb = (b >= 0) ? x[b] : 0.0;

        // -- detection taps (meters free-run; gating happens in ctrl()) --
        double m = (b >= 0) ? 0.5 * (xa + xb) : xa;
        wideMS += kMS100 * (m * m - wideMS);
        double r  = refBP.process(m);   msRef += kMS200 * (r * r - msRef);
        double lo = probeLo.process(m); msLo  += kMS200 * (lo * lo - msLo);
        double hi = probeHi.process(m); msHi  += kMS200 * (hi * hi - msHi);
        msHiF += kMS20c * (hi * hi - msHiF);

        // -- slope + warble metering (consumed by ctrl()) --
        // Mono-sum meters feed the RATIOS (slope, warble log-diff); the
        // per-channel donor filters feed the phase-immune pair-power level
        // meter and the shared normalizer envelope. Same filters later feed
        // the translation itself, so they run whenever the source is damaged.
        bool dMeter = damage > 0.02;
        double dA = 0.0, dB = 0.0;
        if (dMeter) {
            double dm = donMLP.process(donMHP.process(m));
            double lm = lowMLP.process(lowMHP.process(m));
            msDon += kMS200 * (dm * dm - msDon);
            msLow += kMS200 * (lm * lm - msLow);
            envDon.next(std::fabs(dm));
            envLow.next(std::fabs(lm));
            dA = donLP[0].process(donHP[0].process(xa));
            double pp, ea;
            if (b >= 0) {
                dB = donLP[1].process(donHP[1].process(xb));
                pp = 0.5 * (dA * dA + dB * dB);
                ea = 0.5 * (std::fabs(dA) + std::fabs(dB));
            } else { pp = dA * dA; ea = std::fabs(dA); }
            msDonP += kMS200 * (pp - msDonP);
            envDonP.next(ea);
        }

        // -- cleanup: bury swirl/hash just above the corner --
        // Gated like the stages below rather than run-and-multiply-by-zero: at
        // cEff = 1e-3 the wet mix is a tenth of a percent of a gentle lowpass,
        // orders of magnitude under the ~0.5-1 dB level JND. Filter state goes
        // stale while skipped, but cEff ramps up from zero on re-engage so it
        // is faded in, never switched in.
        double cEff = cln * damage * wClean;
        if (cEff > 1e-3) {
            double lpA = cleanLP[0].process(xa); xa += cEff * (lpA - xa);
            if (b >= 0) { double lpB = cleanLP[1].process(xb); xb += cEff * (lpB - xb); }
        }

        // -- regeneration: squaring translation of the donor octave --
        // x^2 on a band [f1, f2] lands its sum terms on [2*f1, 2*f2] -- an
        // octave-up translation that keeps the donor's character (tonal stays
        // tonal, noise stays noise) and its envelope. Difference terms fall
        // near DC and die in the 4th-order injection highpass. Normalizing by
        // the shared donor envelope keeps the translated band envelope-linear
        // (a raw square tracks env^2 -- expander behavior); absolute level is
        // the closed-loop gInj from ctrl(), aimed at the slope target. The
        // injection lowpass stops synthesis above the audibility anchor.
        // Gain-gated: the add ramps from zero on re-engage, so no click.
        if (dMeter && reg * damage * wRegen > 1e-3) {
            double nrm = 1.0 / (1.4142 * envDonP.v + 3e-4);
            // The normalized square is non-negative and unbounded; soft-cap
            // it at 3x the envelope so a transient cannot turn the band into
            // sparse spikes (smooth cap: no hard-clip splatter, linear-ish
            // below 1 so the translated fine structure survives).
            double nA = dA * dA * nrm;  nA = nA / (1.0 + nA * (1.0 / 3.0));
            double yA = injLP17[0].process(
                        injHP[0][1].process(injHP[0][0].process(nA)));
            double yB = 0.0;
            if (b >= 0) {
                double nB = dB * dB * nrm;  nB = nB / (1.0 + nB * (1.0 / 3.0));
                yB = injLP17[1].process(
                     injHP[1][1].process(injHP[1][0].process(nB)));
            }
            msInj += kMS200 * (((b >= 0) ? 0.5 * (yA * yA + yB * yB)
                                         : yA * yA) - msInj);
            double gAdd = gInj * damage * wRegen;
            xa += yA * gAdd;
            if (b >= 0) xb += yB * gAdd;
        }

        // -- Air: boost-only tilt correction on the surviving band's top --
        // (gAir is fully weighted in ctrl(); dA/dB are the donor band taps)
        if (dMeter && gAir > 1e-3) {
            xa += gAir * dA;
            if (b >= 0) xb += gAir * dB;
        }

        // -- transient restoration: per-band onset boost --
        // Detection runs on the pair's SIGNED sum split into two bands (the
        // split has to happen before rectification or the band information is
        // gone). Dividing by the slow envelope makes onset large when an attack
        // arrives out of a quiet passage -- which is precisely the condition
        // that produces pre-echo ("a sharp attack beginning near the end of a
        // transform block immediately following a region of low energy") and
        // also where backward masking has least to work with.
        if (trn * damage > 1e-3) {
            double dm  = (b >= 0) ? 0.5 * (dry[a] + dry[b]) : dry[a];
            double dLo = trSplitD.process(dm);
            double dHi = dm - dLo;
            double efL = trFastLo.next(std::fabs(dLo));
            double esL = trSlowLo.next(std::fabs(dLo));
            double efH = trFastHi.next(std::fabs(dHi));
            double esH = trSlowHi.next(std::fabs(dHi));

            double onL = (efL - esL) / (esL + 1e-5);
            double onH = (efH - esH) / (esH + 1e-5);
            if (onL < 0.0) onL = 0.0;  if (onL > 1.6) onL = 1.6;
            if (onH < 0.0) onH = 0.0;  if (onH > 1.6) onH = 1.6;

            double tEff = trn * damage;
            double gLo = 1.0 + tEff * 0.8 * onL;  if (gLo > 2.0) gLo = 2.0;
            double gHi = 1.0 + tEff * 0.8 * onH;  if (gHi > 2.0) gHi = 2.0;

            // The application split runs unconditionally so its state stays
            // warm (two one-poles per group), but
            // the recombine is skipped when neither band is lifting: steady
            // material then stays bit-exact, since lo + (x - lo) is not
            // guaranteed to round-trip exactly in floating point.
            double aLo = trSplitA[0].process(xa);
            double bLo = (b >= 0) ? trSplitA[1].process(xb) : 0.0;
            if (gLo > 1.0 || gHi > 1.0) {
                xa = aLo * gLo + (xa - aLo) * gHi;
                if (b >= 0) xb = bLo * gLo + (xb - bLo) * gHi;
            }
        }

        x[a] = xa;
        if (b >= 0) x[b] = xb;
    }
};

// ------------------------------------------------------ lookahead restore ----
// Everything else in this plugin MASKS pre-echo -- it lifts the transient so
// backward masking covers more of the smear. That is all a zero-latency process
// can do, and it is not much: backward masking runs 0.5-2 ms (under 5 ms
// effective) against an MP3 short block's 8.7 ms of spread.
//
// With lookahead the problem inverts. Pre-echo occupies the window BEFORE an
// attack; given enough delay we detect the attack while that window is still
// sitting in the delay line, unemitted, and can attenuate it instead of trying
// to hide it. That is removal of added noise -- correction of the decoded
// signal's temporal envelope -- not psychoacoustic sleight of hand.
//
// What it cannot do is restore detail the quantiser discarded inside the
// transient. It reconstructs the ENVELOPE, not the CONTENT. The envelope is
// what reads as smear, so that is the part worth having.
//
// The false-positive guard comes straight out of the literature: pre-echo needs
// "a sharp attack beginning near the end of a transform block immediately
// following a region of low energy". Requiring the pre-window to sit well below
// the attack keeps this off snare rolls, crescendos and reverb tails, where a
// naive version would punch audible holes.
//
// OFF by default and refused outright under Equalizer APO, which does no plugin
// delay compensation -- 32 ms of uncompensated latency would wreck A/V sync.
static const double kRestoreLookMs  = 32.0;    // > MP3 long block (26.1 ms @44.1k)
static const double kRestoreWinMs   = 26.0;    // suppression window
static const double kRestoreRampMs  =  2.0;    // attack-edge ramp, no step at the hit
static const double kRestoreSplitHz = 700.0;   // cut floor: below this, pre-echo
                                               // is not separable from signal and
                                               // cuts do audible harm
// ratio / max-cut / slow-release live in g_tune (TN_RSTRATIO, TN_RSTMAXDB,
// TN_RSTSLOWREL)

struct RestoreStage {
    std::vector<float> dl[8];             // every channel: latency must be uniform
    std::vector<float> gring;             // scheduled HF gain, 1 = untouched
    std::vector<float> efring;            // fast-envelope history for the width scan
    int len = 0, L = 0, W = 0, ramp = 1, wpos = 0, hold = 0, holdLen = 0;
    OnePoleLP splitD, splitA[2];          // hf = x - lp
    EnvAR envFast, envSlow;
    bool ready = false;

    int configure(double fs) {
        L = (int)(kRestoreLookMs * 0.001 * fs);
        W = (int)(kRestoreWinMs  * 0.001 * fs);
        ramp = (int)(kRestoreRampMs * 0.001 * fs); if (ramp < 1) ramp = 1;
        if (W > L - 8) W = L - 8;
        if (W < 2 * ramp + 2) { ready = false; return 0; }
        len = L + 8;
        for (int c = 0; c < 8; ++c) dl[c].assign(len, 0.0f);
        gring.assign(len, 1.0f);
        efring.assign(len, 0.0f);
        splitD.setCutoff(fs, kRestoreSplitHz);
        splitA[0].setCutoff(fs, kRestoreSplitHz);
        splitA[1].setCutoff(fs, kRestoreSplitHz);
        envFast.init(fs, 0.3, 20.0);
        envSlow.init(fs, 25.0, g_tune[TN_RSTSLOWREL]);
        holdLen = (int)(0.005 * fs);
        wpos = 0; hold = 0; ready = true;
        return L;
    }

    void reset() {
        for (int c = 0; c < 8; ++c) std::fill(dl[c].begin(), dl[c].end(), 0.0f);
        std::fill(gring.begin(), gring.end(), 1.0f);
        std::fill(efring.begin(), efring.end(), 0.0f);
        splitD.reset(); splitA[0].reset(); splitA[1].reset();
        envFast.reset(); envSlow.reset();
        wpos = 0; hold = 0;
    }

    // Paint the attenuation profile back over the window preceding the attack.
    // W < L guarantees none of it has been emitted yet.
    //
    // Width estimation first: walk back through the fast-envelope history and
    // stop where the pre-window stops being quiet -- that is the previous
    // hit's decay (real signal), not codec noise, and cutting it punches
    // audible holes in rolls. Only the genuinely quiet valley is treated.
    // Then the cut is dB-linear: deepest just before the attack, fading to
    // nothing at the far (early) edge -- pre-echo grows toward the hit.
    // min() so overlapping hits deepen rather than overwrite each other.
    void schedule(double cutDb, double atk) {
        int Weff = W;
        double thr = atk * 0.25;              // -12 dB of this attack
        // skip the attack's own rise (inside the ramp guard) -- the envelope
        // needs ~0.3 ms to fire, so the last samples before wpos are the hit
        for (int k = ramp + 1; k <= W; ++k) {
            int idx = wpos - k; while (idx < 0) idx += len;
            if (efring[idx] > thr) { Weff = k - 1; break; }
        }
        if (Weff < 2 * ramp + 2) return;      // no usable valley (dense roll)
        for (int k = 1; k <= Weff; ++k) {
            int idx = wpos - k; while (idx < 0) idx += len;
            double d = cutDb * (double)(Weff - k) / (Weff - ramp);
            if (k <= ramp) d *= (double)k / ramp;    // no step at the attack
            if (d > cutDb) d = cutDb;
            double f = std::pow(10.0, -d / 20.0);
            if (f < gring[idx]) gring[idx] = (float)f;
        }
    }

    // Replaces x[]/dry[] with the delayed signal, pre-echo suppressed on the
    // fronts. dry[] carries the delayed-but-untouched reference so Mix still
    // means what it says.
    void tick(double* x, double* dry, double depth) {
        double dm = 0.5 * (dry[CH_FL] + dry[CH_FR]);
        double hf = dm - splitD.process(dm);
        double a  = std::fabs(hf);
        double ef = envFast.next(a), es = envSlow.next(a);
        efring[wpos] = (float)ef;

        if (hold > 0) --hold;
        else if (es > 1e-7 && ef > es * g_tune[TN_RSTRATIO]) {
            // Sharper attack out of a quieter window -> deeper cut, capped.
            double excess = ef / (es * g_tune[TN_RSTRATIO]) - 1.0;
            schedule(g_tune[TN_RSTMAXDB] * depth * clamp01(excess), ef);
            hold = holdLen;
        }

        for (int c = 0; c < 8; ++c) dl[c][wpos] = (float)dry[c];

        int r = wpos - L; if (r < 0) r += len;
        double g = gring[r];
        gring[r] = 1.0f;                       // consume
        for (int c = 0; c < 8; ++c) dry[c] = x[c] = dl[c][r];
        if (g < 1.0f) {
            for (int i = 0; i < 2; ++i) {
                int c = (i == 0) ? CH_FL : CH_FR;
                double lo = splitA[i].process(x[c]);
                x[c] = lo + (x[c] - lo) * g;   // high band only
            }
        }
        if (++wpos >= len) wpos = 0;
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
    RestoreStage restore;
    // Latency is FIXED whenever the host can compensate, rather than following
    // the knob: a lookahead that appears and disappears mid-session makes the
    // host re-negotiate delay compensation and jumps the stream. The knob sets
    // depth only. Refused entirely under a host that identifies as Equalizer
    // APO, which does no PDC at all.
    bool restoreAllowed = false;
    int   ctrlCount = 0;

    // Plugin-wide idle gate: after kGateHold of sub-threshold input on EVERY
    // channel, signal state is flushed and processing is skipped until the
    // first loud sample (which is always processed -- the wake test runs BEFORE
    // the skip, so onsets after silence are never touched). Groups additionally
    // sleep individually; see Group::dormant.
    int  silentRun = 0;
    bool dormant   = false;

    // control-rate constants (recomputed on sample-rate change)
    double kMS100 = 0, kMS200 = 0;
    double stepUp = 0, stepDown = 0, holdAtt = 0, holdRel = 0;
    double cornerMinLog = 0, cornerMaxLog = 0;

    Smooth smRepair, smRestore;

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
        params[P_REPAIR]  = 0.6f;         // the calibrated default
        params[P_RESTORE] = 0.0f;         // off: costs latency, needs host PDC
        params[P_FREEZE]  = 0.0f;
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
        groups[G_CENTER] = Group{}; groups[G_CENTER].chA = CH_FC;
        groups[G_SIDE]   = Group{}; groups[G_SIDE].chA   = CH_SL; groups[G_SIDE].chB   = CH_SR;
        groups[G_BACK]   = Group{}; groups[G_BACK].chA   = CH_BL; groups[G_BACK].chB   = CH_BR;
        for (int g = 0; g < kNumGroups; ++g) groups[g].configure(fs, cornerMaxLog);

        smRepair.init(fs, 30.0, params[P_REPAIR]);
        smRestore.init(fs, 30.0, params[P_RESTORE]);
        int lat = restoreAllowed ? restore.configure(fs) : 0;
        if (effect.initialDelay != lat) {
            effect.initialDelay = lat;
            if (master) master(&effect, audioMasterIOChanged, 0, 0, nullptr, 0.0f);
        }
        ctrlCount = 0;
    }

    void resetState() {   // signal state only: also called by the silence gate
        for (int g = 0; g < kNumGroups; ++g) groups[g].resetSignalState();
        ctrlCount = 0;
        silentRun = 0;
        dormant   = false;
    }

    // Pipeline restart (effMainsChanged on). Two extras beyond the signal-state
    // flush, and both would be WRONG in resetState() itself, because the
    // silence gate calls that mid-stream:
    //   - Freeze auto-clears. EAPO loads one instance system-wide, so a corner
    //     frozen for an old game would otherwise persist and lowpass whatever
    //     runs next -- but a paused game going digitally silent must not
    //     uncheck the user's Freeze, hence not in resetState(). The GUI
    //     checkbox resyncs on its timer via syncFromHost().
    //   - The Restore delay line empties. Across a transport stop/start, up to
    //     32 ms of stale pre-stop audio would otherwise replay on resume. The
    //     silence gate must NOT do this: it sleeps mid-stream, where the line
    //     has to keep turning or the stream would jump on wake.
    void mainsRestart() {
        resetState();
        restore.reset();
        params[P_FREEZE] = 0.0f;
    }

    void ctrlUpdate() {
        bool frozen = params[P_FREEZE] > 0.5f;
        for (int g = 0; g < kNumGroups; ++g) {
            if (groups[g].dormant) continue;   // meters are zeroed; corner parked
            groups[g].ctrl(fs, frozen, stepUp, stepDown, holdAtt, holdRel,
                           cornerMinLog, cornerMaxLog,
                           params[P_REPAIR], params[P_REPAIR]);
        }
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
            for (int c = 0; c < 8; ++c)
                dry[c] = x[c] = in[c] ? (double)in[c][i] : 0.0;

            // -- lookahead pre-echo suppression --
            // Runs FIRST and before the idle gate, because it rewrites x[] and
            // dry[] with the delayed signal: everything downstream, including
            // the gate's peak test and the groups' dry[] reference, has to see
            // the same time base. The delay line must keep turning even while
            // the plugin idles or the stream would jump on wake.
            if (restore.ready) restore.tick(x, dry, smRestore.next(params[P_RESTORE]));

            for (int c = 0; c < 8; ++c) {
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
                smRepair.snap(params[P_REPAIR]);
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

            double rep = smRepair.next(params[P_REPAIR]);

            for (int g = 0; g < kNumGroups; ++g)
                groups[g].tick(x, dry, kMS100, kMS200, rep, rep, g_tune[TN_TRNSCALE] * rep);

            for (int c = 0; c < 8; ++c) {
                if (!out[c]) continue;
                if (c == CH_LFE) { out[c][i] = (T)dry[c]; continue; }
                out[c][i] = (T)softclip(x[c]);
            }
        }
        _mm_setcsr(csr);
    }
};

// ------------------------------------------------------------- editor GUI ----
#if defined(_WIN32)
static VstRect g_edRect = { 0, 0, 268, 460 };

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
    wc.cbSize = sizeof wc;               // GetClassInfoExA requires this preset
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

    const char* names[kNumSliders] = { "Repair", "Restore" };
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
    case effMainsChanged:  if (value) p->mainsRestart(); return 0;

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
            const char* names[] = { "Repair", "Restore", "Freeze" };
            copyStr(ptr, names[index], kMaxParamStr);
        } else if (opcode == effGetParamLabel) {
            const char* labels[] = { "%", "%", "" };
            copyStr(ptr, labels[index], kMaxParamStr);
        } else { // display
            if (index == P_FREEZE) {
                copyStr(ptr, p->params[P_FREEZE] > 0.5f ? "On" : "Off", kMaxParamStr);
            } else if (index == P_RESTORE && !p->restoreAllowed) {
                copyStr(ptr, "n/a", kMaxParamStr);   // no PDC in this host
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
        if (index == REGEN_DORMANT_QUERY && p) {
            VstIntPtr m = 0;
            for (int g = 0; g < kNumGroups; ++g) if (p->groups[g].dormant) m |= (1 << g);
            return m;
        }
        if (index == REGEN_TUNE_QUERY && p) {
            if ((int)value < 0)
                for (int k = 0; k < NTUNE; ++k) g_tune[k] = kTuneDefault[k];
            else if ((int)value < NTUNE) g_tune[(int)value] = (double)opt;
            for (int g = 0; g < kNumGroups; ++g) {
                p->groups[g].envDonP.init(p->fs, 1.0, g_tune[TN_NRMREL]);
                p->groups[g].appliedLog = -1.0;
            }
            if (p->restore.ready)
                p->restore.envSlow.init(p->fs, 25.0, g_tune[TN_RSTSLOWREL]);
            return 1;
        }
        if (index == REGEN_DEBUG_QUERY && p) {
            const Group& g = p->groups[0];
            double v = 0;
            switch ((int)value) {
                case 0: v = g.msDon;         break;
                case 1: v = g.msLow;         break;
                case 2: v = g.msInj;         break;
                case 3: v = g.gInj * 1e-6;   break;
                case 4: v = g.warble * 1e-6; break;
                case 5: v = g.slideOct * 1e-6; break;
                case 6: v = g.wRegen * 1e-6;   break;
                case 7: v = g.damage * 1e-6;   break;
                case 8: v = std::exp2(g.cornerLog) * 1e-3; break;
                case 9: v = g.msDonP;        break;
            }
            return (VstIntPtr)std::llround(v * 1e9);
        }
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

    // Restore needs the host to compensate 32 ms of latency. Equalizer APO does
    // not, so it is refused there outright. The test is deliberately asymmetric:
    // refuse only on a POSITIVE identification, because an unidentified host
    // (a minimal offline harness, say) is one the user drove deliberately, and
    // Restore still defaults to off there.
    if (audioMaster) {
        char host[80] = {0};
        audioMaster(e, audioMasterGetProductString, 0, 0, host, 0.0f);
        host[sizeof(host) - 1] = 0;
        bool isEapo = false;
        for (char* s = host; *s; ++s)                       // case-insensitive find
            if ((s[0] | 32) == 'e' && _strnicmp(s, "equalizer", 9) == 0) isEapo = true;
        p->restoreAllowed = !isEapo;
    }
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
