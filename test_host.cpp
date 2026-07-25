// test_host.cpp -- minimal VST2 host to sanity-check ReGen.dll.
// Loads the DLL, instantiates the effect, and verifies:
//   - param strings never overrun kVstMaxParamStrLen (Equalizer APO gives 8 bytes)
//   - the editor survives EAPO's garbage-rect effEditGetRect call
//   - the rolloff servo converges DOWN on bandlimited content and back UP on
//     full-bandwidth content (read via the private effVendorSpecific query)
//   - cleanup attenuates content above the detected corner
//   - regeneration synthesizes new energy above the corner (2nd harmonic of a
//     source-band tone), and none with the knob at zero
//   - transient boost lifts burst onsets
//   - Mix=0 is bit-transparent and LFE always passes through untouched
//
// Build (MinGW): g++ -O2 -o test_host.exe test_host.cpp
#include "vst2_min.h"
#include <windows.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define REGEN_ROLLOFF_QUERY CCONST('R', 'o', 'f', 'f')
#define REGEN_DORMANT_QUERY CCONST('D', 'o', 'r', 'm')

// This harness exists to emulate Equalizer APO, so it says so -- which also
// exercises the path where Restore refuses to add latency a host cannot
// compensate. VSTCALLBACK_daw is the opposite case.
static VstIntPtr VSTCALLBACK_master(AEffect*, VstInt32 op, VstInt32, VstIntPtr,
                                    void* ptr, float) {
    if (op == audioMasterGetProductString && ptr) {
        std::strcpy((char*)ptr, "Equalizer APO"); return 1;
    }
    return 0;
}
static VstIntPtr VSTCALLBACK_daw(AEffect*, VstInt32 op, VstInt32, VstIntPtr,
                                 void* ptr, float) {
    if (op == audioMasterGetProductString && ptr) {
        std::strcpy((char*)ptr, "TestDAW"); return 1;
    }
    return 0;
}
typedef AEffect* (*EntryProc)(audioMasterCallback);

// Goertzel magnitude of frequency f in buffer.
static double goertzel(const std::vector<float>& x, size_t from, size_t to,
                       double fs, double f) {
    double w = 2.0 * M_PI * f / fs, cw = std::cos(w), coeff = 2.0 * cw;
    double s0 = 0, s1 = 0, s2 = 0;
    for (size_t i = from; i < to; ++i) { s0 = x[i] + coeff * s1 - s2; s2 = s1; s1 = s0; }
    double re = s1 - s2 * cw, im = s2 * std::sin(w);
    return std::sqrt(re * re + im * im) / (double)(to - from);
}

static const double fs = 48000.0;
static const int    kCh = 8;

struct Buffers {
    std::vector<float> in[kCh], out[kCh];
    int n;
    explicit Buffers(int frames) : n(frames) {
        for (int c = 0; c < kCh; ++c) { in[c].assign(n, 0.0f); out[c].assign(n, 0.0f); }
    }
};

static void process(AEffect* fx, Buffers& b) {
    for (int off = 0; off < b.n; off += 512) {
        int n = (off + 512 <= b.n) ? 512 : (b.n - off);
        float* bi[kCh]; float* bo[kCh];
        for (int c = 0; c < kCh; ++c) { bi[c] = b.in[c].data() + off; bo[c] = b.out[c].data() + off; }
        fx->processReplacing(fx, bi, bo, n);
    }
}

static void setParams(AEffect* fx, float repair, float restore, float freeze) {
    fx->setParameter(fx, 0, repair); fx->setParameter(fx, 1, restore);
    fx->setParameter(fx, 2, freeze);
}

static long corner(AEffect* fx, int group) {
    return (long)fx->dispatcher(fx, effVendorSpecific, REGEN_ROLLOFF_QUERY,
                                group, nullptr, 0);
}

// A virgin instance. The gated peak-hold meters (hRef/hLo/hHi) deliberately
// survive resetSignalState() -- they are what keeps silence from dragging the
// corner around -- so once full-band content has pumped hHi up, a later test
// cannot retrain the corner downward in any reasonable time. Tests that need a
// known servo state must therefore start from a fresh effect, not inherit one.
static AEffect* freshFx(EntryProc entry) {
    AEffect* fx = entry(VSTCALLBACK_master);
    fx->dispatcher(fx, effOpen, 0, 0, nullptr, 0);
    fx->dispatcher(fx, effSetSampleRate, 0, 0, nullptr, (float)fs);
    fx->dispatcher(fx, effSetBlockSize, 0, 512, nullptr, 0);
    fx->dispatcher(fx, effMainsChanged, 0, 1, nullptr, 0);
    return fx;
}

// Fill fronts (+ scaled surrounds, center, LFE) with a multitone. `walled`
// content stops at 10 kHz; full-band content extends to 19 kHz.
static void fillSignal(Buffers& b, bool walled) {
    const double f10[] = { 500, 1200, 2400, 3000, 4500, 6000, 8000, 10000 };
    const double fHF[] = { 12500, 16000, 19000 };
    for (int i = 0; i < b.n; ++i) {
        double t = i / fs, v = 0.0;
        for (double f : f10) v += 0.10 * std::sin(2.0 * M_PI * f * t);
        if (!walled) for (double f : fHF) v += 0.10 * std::sin(2.0 * M_PI * f * t);
        float s = (float)v;
        b.in[0][i] = s;          b.in[1][i] = s;                    // FL FR
        b.in[2][i] = 0.7f * s;                                      // FC
        b.in[3][i] = 0.4f * (float)std::sin(2.0 * M_PI * 40.0 * t); // LFE
        b.in[4][i] = b.in[5][i] = 0.5f * s;                         // BL BR
        b.in[6][i] = b.in[7][i] = 0.5f * s;                         // SL SR
    }
}

static bool finite8(const Buffers& b) {
    for (int c = 0; c < kCh; ++c)
        for (float v : b.out[c]) if (!std::isfinite(v)) return false;
    return true;
}

int main() {
    bool ok = true;
    HMODULE dll = LoadLibraryA("ReGen.dll");
    if (!dll) { printf("FAIL: cannot load ReGen.dll (err %lu)\n", GetLastError()); return 1; }
    EntryProc entry = (EntryProc)GetProcAddress(dll, "VSTPluginMain");
    if (!entry) { printf("FAIL: no VSTPluginMain export\n"); return 1; }

    AEffect* fx = entry(VSTCALLBACK_master);
    if (!fx || fx->magic != kEffectMagic) { printf("FAIL: bad AEffect\n"); return 1; }
    printf("OK: loaded. numParams=%d in=%d out=%d flags=0x%x\n",
           fx->numParams, fx->numInputs, fx->numOutputs, fx->flags);

    // --- buffer-safety: EAPO allocates only 8 bytes for param strings ---
    {
        bool safe = true;
        struct { char buf[8]; char canary[8]; } g;
        VstInt32 ops[] = { effGetParamName, effGetParamLabel, effGetParamDisplay };
        for (VstInt32 op : ops) {
            for (VstInt32 pi = 0; pi < fx->numParams; ++pi) {
                std::memset(&g, 0xAB, sizeof g);
                fx->dispatcher(fx, op, pi, 0, g.buf, 0);
                for (int k = 0; k < 8; ++k)
                    if ((unsigned char)g.canary[k] != 0xAB) safe = false;
                bool term = false;
                for (int k = 0; k < 8; ++k) if (g.buf[k] == 0) term = true;
                if (!term) safe = false;
            }
        }
        printf("bufsafe: %s\n", safe ? "no overflow, all terminated" : "OVERFLOW/UNTERMINATED");
        if (!safe) ok = false;
    }

    // --- DAW-compat: identify, indexed program name, param clamping ---
    {
        long id = (long)fx->dispatcher(fx, effIdentify_DEPRECATED, 0, 0, nullptr, 0);
        char prog[24] = {0};
        long pr = (long)fx->dispatcher(fx, effGetProgramNameIndexed, 0, 0, prog, 0);
        fx->setParameter(fx, 0, 1.5f);
        float clamped = fx->getParameter(fx, 0);
        printf("daw  : identify=0x%08lx prog0='%s'(ret %ld) clamp(1.5)=%.2f\n",
               id, prog, pr, clamped);
        if (id != CCONST('N', 'v', 'E', 'f')) { printf("FAIL: effIdentify\n"); ok = false; }
        if (pr != 1 || std::strcmp(prog, "Default") != 0) {
            printf("FAIL: effGetProgramNameIndexed\n"); ok = false;
        }
        if (clamped != 1.0f) { printf("FAIL: setParameter not clamped\n"); ok = false; }
    }

    // --- editor: replicate EAPO's startEditing(), garbage rect included ---
    {
        HWND parent = CreateWindowExA(0, "STATIC", "", WS_OVERLAPPEDWINDOW,
                                      0, 0, 500, 400, nullptr, nullptr,
                                      GetModuleHandle(nullptr), nullptr);
        VstRect* rect = (VstRect*)(intptr_t)0x1;
        fx->dispatcher(fx, effEditGetRect, 0, 0, &rect, 0);
        fx->dispatcher(fx, effEditOpen, 0, 0, parent, 0);
        rect = (VstRect*)(intptr_t)0x1;
        fx->dispatcher(fx, effEditGetRect, 0, 0, &rect, 0);
        if (rect == (VstRect*)(intptr_t)0x1 || !rect) {
            printf("FAIL: effEditGetRect did not set the rect pointer\n"); return 1;
        }
        printf("editor: rect=%dx%d\n", rect->right - rect->left, rect->bottom - rect->top);
        MSG m; int pumped = 0;
        while (pumped++ < 40 && PeekMessageA(&m, nullptr, 0, 0, PM_REMOVE)) {
            TranslateMessage(&m); DispatchMessageA(&m);
        }
        fx->dispatcher(fx, effEditClose, 0, 0, nullptr, 0);
        if (parent) DestroyWindow(parent);
        printf("editor: opened + closed, no crash\n");
    }

    fx->dispatcher(fx, effSetSampleRate, 0, 0, nullptr, (float)fs);
    fx->dispatcher(fx, effSetBlockSize, 0, 512, nullptr, 0);
    fx->dispatcher(fx, effMainsChanged, 0, 1, nullptr, 0);

    // --- transparency: mix=0 must be bit-exact; LFE always passes ---
    {
        Buffers b(2 * (int)fs);
        fillSignal(b, true);
        setParams(fx, 0, 0, 0);
        process(fx, b);
        if (!finite8(b)) { printf("FAIL: non-finite output\n"); ok = false; }
        double maxd = 0, lfed = 0;
        for (int i = b.n / 2; i < b.n; ++i) {
            maxd = std::max(maxd, (double)std::fabs(b.out[0][i] - b.in[0][i]));
            lfed = std::max(lfed, (double)std::fabs(b.out[3][i] - b.in[3][i]));
        }
        printf("dry  : max|out-in| FL=%.6f LFE=%.6f (expect ~0)\n", maxd, lfed);
        if (maxd > 1e-4 || lfed > 0.0) { printf("FAIL: mix=0 not transparent\n"); ok = false; }
    }

    // --- servo: train on 10 kHz-walled content; corner must come down ---
    long cFront = 0, cSide = 0;
    {
        Buffers b(8 * (int)fs);
        fillSignal(b, true);
        setParams(fx, 0, 0, 0);
        process(fx, b);
        cFront = corner(fx, 0); cSide = corner(fx, 2);
        printf("servo: walled-at-10k -> corner front=%ld side=%ld back=%ld center=%ld\n",
               cFront, cSide, corner(fx, 3), corner(fx, 1));
        // 12th-order probes resolve a pure-tone edge ~0.5 oct high, so accept
        // anything clearly below full bandwidth but above the wall itself.
        if (cFront < 10000 || cFront > 17000) { printf("FAIL: front corner off\n"); ok = false; }
        if (cSide  < 10000 || cSide  > 17000) { printf("FAIL: side corner off\n");  ok = false; }
    }

    // --- regeneration: squaring translation puts donor-band sum terms one
    //     octave up. With the corner near 12k the donor holds the 8k and 10k
    //     tones, so 2x8k lands at 16k (and 8+10 at 18k dies in the synthesis
    //     stop). Level is closed-loop matched to the slope target, so the
    //     translated tone must appear but stay below the donor tone itself. ---
    {
        Buffers b(2 * (int)fs);
        fillSignal(b, true);
        setParams(fx, 0, 0, 1);          // repair off, detector frozen
        process(fx, b);
        double off16 = goertzel(b.out[0], b.n / 2, b.n, fs, 16000.0);
        setParams(fx, 1, 0, 1);          // repair full = slope target
        process(fx, b);
        double on16 = goertzel(b.out[0], b.n / 2, b.n, fs, 16000.0);
        double src8 = goertzel(b.in[0],  b.n / 2, b.n, fs, 8000.0);
        printf("regen: 16k off=%.6f on=%.6f (src 8k=%.4f)\n", off16, on16, src8);
        if (on16 < 3.0 * off16 + 1e-4) { printf("FAIL: no regenerated highs\n"); ok = false; }
        // On a FLAT multitone the slope target grants the full donor energy
        // and the injection filters leave ~one surviving line to carry it, so
        // parity with a source tone is the design level, not runaway. The
        // bound guards the runaway class (normalizer/servo faults land 3-6x).
        if (on16 > 1.2 * src8) { printf("FAIL: regen level implausibly hot\n"); ok = false; }
    }

    // --- cleanup: an 18 kHz tone (above the learned corner) gets attenuated ---
    {
        Buffers b(2 * (int)fs);
        fillSignal(b, true);
        for (int i = 0; i < b.n; ++i) {
            float t18 = 0.08f * (float)std::sin(2.0 * M_PI * 18600.0 * i / fs);
            b.in[0][i] += t18; b.in[1][i] += t18;
        }
        setParams(fx, 0, 0, 1);          // frozen, repair off
        process(fx, b);
        double raw18 = goertzel(b.out[0], b.n / 2, b.n, fs, 18600.0);
        setParams(fx, 1, 0, 1);          // repair full
        process(fx, b);
        double cln18 = goertzel(b.out[0], b.n / 2, b.n, fs, 18600.0);
        printf("clean: 18k raw=%.5f cleaned=%.5f\n", raw18, cln18);
        if (cln18 > 0.85 * raw18) { printf("FAIL: cleanup not attenuating\n"); ok = false; }
    }

    // --- transient: burst onsets get lifted (detector still frozen-lossy) ---
    {
        Buffers b(2 * (int)fs);
        for (int i = 0; i < b.n; ++i) {
            int ph = i % 12000;                    // 30 ms burst every 250 ms
            float v = (ph < 1440) ? 0.25f * (float)std::sin(2.0 * M_PI * 2000.0 * i / fs) : 0.0f;
            b.in[0][i] = b.in[1][i] = v;
        }
        setParams(fx, 1, 0, 1);
        process(fx, b);
        float inPk = 0, outPk = 0;
        for (int i = b.n / 2; i < b.n; ++i) {
            inPk  = std::max(inPk,  std::fabs(b.in[0][i]));
            outPk = std::max(outPk, std::fabs(b.out[0][i]));
        }
        printf("trans: burst peak in=%.3f out=%.3f\n", inPk, outPk);
        if (outPk < 1.05f * inPk) { printf("FAIL: no attack boost\n"); ok = false; }
    }

    // --- servo recovery: full-band content must pull the corner back up ---
    {
        setParams(fx, 0, 0, 0);           // unfreeze
        Buffers b(6 * (int)fs);
        fillSignal(b, false);
        process(fx, b);
        long c = corner(fx, 0);
        printf("servo: full-band -> corner front=%ld (expect >=17000)\n", c);
        if (c < 17000) { printf("FAIL: corner did not recover\n"); ok = false; }
        if (!finite8(b)) { printf("FAIL: non-finite output\n"); ok = false; }
    }

    // --- idle gate: long silence -> exact-zero passthrough, corner retained,
    //     wake processes the very first audible sample ---
    {
        setParams(fx, 1, 0, 1);           // frozen so corner is provable
        long before = corner(fx, 0);
        Buffers b(2 * (int)fs);                    // 2 s digital silence
        process(fx, b);
        long after = corner(fx, 0);
        bool zero = true;                          // decaying tails allowed early;
        for (int c = 0; c < kCh; ++c)              // gate must be exact 0 by 1 s
            for (int i = b.n / 2; i < b.n; ++i)
                if (b.out[c][i] != 0.0f) zero = false;
        printf("gate : silence zero-out=%s corner %ld -> %ld\n",
               zero ? "yes" : "NO", before, after);
        if (!zero) { printf("FAIL: gate not exact-zero in silence\n"); ok = false; }
        if (after != before) { printf("FAIL: corner changed across dormancy\n"); ok = false; }

        // Freeze must survive dormancy. Only a pipeline restart auto-clears it;
        // a paused game going digitally silent must not uncheck the user's own
        // setting mid-session. The corner check above cannot catch this on its
        // own -- a dormant group runs no servo, so a silently cleared Freeze
        // still leaves the corner parked, and the regression hides until audio
        // returns and the corner walks off.
        float frz = fx->getParameter(fx, 2);
        printf("gate : Freeze across dormancy = %.0f (expect 1)\n", frz);
        if (frz < 0.5f) { printf("FAIL: silence gate cleared Freeze\n"); ok = false; }

        Buffers b2((int)fs);                       // tone starts at sample 1000
        for (int i = 1000; i < b2.n; ++i) {
            float v = 0.2f * (float)std::sin(2.0 * M_PI * 1000.0 * (i - 1000) / fs);
            b2.in[0][i] = b2.in[1][i] = v;
        }
        process(fx, b2);
        int firstOut = -1;
        for (int i = 0; i < b2.n; ++i)
            if (std::fabs(b2.out[0][i]) > 1e-4f) { firstOut = i; break; }
        printf("gate : wake first-output sample %d (tone starts 1000)\n", firstOut);
        if (firstOut < 1000 || firstOut > 1010) {
            printf("FAIL: wake not sample-accurate\n"); ok = false;
        }
        if (!finite8(b2)) { printf("FAIL: non-finite output\n"); ok = false; }
    }

    // --- per-group idle gate: a group with no input sleeps on its own pair,
    //     keeps its learned corner parked while other groups move, and wakes
    //     sample-accurately. Regression guard for the plugin-wide gate being
    //     the only one (which made silent channels cost full price). ---
    {
        AEffect* gx = freshFx(entry);
        setParams(gx, 0.6f, 0, 0);
        Buffers train(8 * (int)fs);                 // train ALL groups down
        fillSignal(train, true);                    // (down-slew is deliberately slow)
        process(gx, train);
        long frontTrained = corner(gx, 0);
        long sideTrained = corner(gx, 2), backTrained = corner(gx, 3);

        // Fronts now get FULL-BAND content while the surrounds get nothing.
        // The front corner must climb; the sleeping groups must not budge.
        Buffers fo(2 * (int)fs);
        const double fAll[] = { 500, 2400, 6000, 10000, 12500, 16000, 19000 };
        for (int i = 0; i < fo.n; ++i) {
            double t = i / fs, v = 0.0;
            for (double f : fAll) v += 0.10 * std::sin(2.0 * M_PI * f * t);
            fo.in[0][i] = fo.in[1][i] = (float)v;   // FL/FR only; 2..7 stay zero
        }
        process(gx, fo);
        long frontMoved = corner(gx, 0);
        long sideParked = corner(gx, 2), backParked = corner(gx, 3);

        // Tails from the training pass decay for the first 250 ms (the hold),
        // as with the plugin-wide gate; require exact zero once asleep.
        bool quiet = true;
        for (int c = 4; c < kCh; ++c)
            for (int i = fo.n / 2; i < fo.n; ++i)
                if (fo.out[c][i] != 0.0f) quiet = false;

        long dorm = (long)gx->dispatcher(gx, effVendorSpecific,
                                         REGEN_DORMANT_QUERY, 0, nullptr, 0);
        printf("group: front %ld->%ld while side %ld->%ld back %ld->%ld (parked), "
               "dormant mask=0x%lx\n",
               frontTrained, frontMoved, sideTrained, sideParked,
               backTrained, backParked, dorm);
        // bit0 front (must be awake), bits 1..3 center/side/back (must sleep)
        if (dorm != 0xE) {
            printf("FAIL: expected groups 1-3 asleep and front awake (0xE)\n");
            ok = false;
        }
        if (frontMoved < 17000) {
            printf("FAIL: front group stopped tracking\n"); ok = false;
        }
        if (sideTrained > 17000) {
            printf("FAIL: training left nothing to prove (side already at ceiling)\n");
            ok = false;
        }
        if (sideParked != sideTrained || backParked != backTrained) {
            printf("FAIL: dormant group's corner drifted\n"); ok = false;
        }
        if (!quiet) { printf("FAIL: silent group did not pass through clean\n"); ok = false; }

        // Wake the sides mid-stream; fronts stay active throughout.
        Buffers wk((int)fs);
        for (int i = 0; i < wk.n; ++i) {
            double t = i / fs;
            float v = (float)(0.2 * std::sin(2.0 * M_PI * 1000.0 * t));
            wk.in[0][i] = wk.in[1][i] = v;
            if (i >= 1000)
                wk.in[6][i] = wk.in[7][i] =
                    (float)(0.2 * std::sin(2.0 * M_PI * 1000.0 * (i - 1000) / fs));
        }
        process(gx, wk);
        int firstSide = -1;
        for (int i = 0; i < wk.n; ++i)
            if (std::fabs(wk.out[6][i]) > 1e-4f) { firstSide = i; break; }
        printf("group: side wake first-output sample %d (tone starts 1000)\n", firstSide);
        if (firstSide < 1000 || firstSide > 1010) {
            printf("FAIL: per-group wake not sample-accurate\n"); ok = false;
        }
        if (!finite8(wk)) { printf("FAIL: non-finite output\n"); ok = false; }
        gx->dispatcher(gx, effClose, 0, 0, nullptr, 0);
    }

    // --- transient band separation: the boost must stay in the band that
    //     actually has the onset. A single broadband detector/gain fails this
    //     (measured +0.25 dB on an HF transient over bass, vs +0.31 dB on the
    //     steady bass it was supposed to leave alone). ---
    {
        AEffect* bx = freshFx(entry);
        Buffers train(8 * (int)fs);                 // corner down, then freeze
        fillSignal(train, true);
        setParams(bx, 1, 0, 0);            // Attack alone, servo live
        process(bx, train);
        long cTrain = corner(bx, 0);
        setParams(bx, 1, 0, 1);            // freeze: damage now provable
        if (cTrain > 17000) {
            printf("FAIL: corner never came down; Attack would be idle\n"); ok = false;
        }

        struct BandCase { const char* name; double burst, steady; };
        BandCase bc[] = { { "LF burst / steady HF", 200.0, 8000.0 },
                          { "HF burst / steady LF", 8000.0, 200.0 } };
        for (const BandCase& c : bc) {
            Buffers b(3 * (int)fs);
            for (int i = 0; i < b.n; ++i) {
                double t = i / fs, ph = std::fmod(t, 0.5);
                double env = (ph < 0.10) ? std::exp(-ph * 30.0) : 0.0;
                float v = (float)(0.18 * env * std::sin(2.0 * M_PI * c.burst * t)
                                + 0.10 * std::sin(2.0 * M_PI * c.steady * t));
                b.in[0][i] = b.in[1][i] = v;
            }
            process(bx, b);
            size_t o0 = (size_t)(2.5 * fs), o1 = o0 + (size_t)(0.006 * fs);
            double gB = 20.0 * std::log10(goertzel(b.out[0], o0, o1, fs, c.burst) /
                                         (goertzel(b.in[0], o0, o1, fs, c.burst) + 1e-15));
            double gS = 20.0 * std::log10(goertzel(b.out[0], o0, o1, fs, c.steady) /
                                         (goertzel(b.in[0], o0, o1, fs, c.steady) + 1e-15));
            printf("bands: %-22s burst %+5.2f dB  steady %+5.2f dB\n", c.name, gB, gS);
            // Margin threshold is calibrated to the shipping tune vector; the
            // 2026-07-25 face-off winner runs less attack share (TN_TRNSCALE
            // 0.58 -> 0.20), which legitimately narrows the LF-case margin.
            if (gB - gS < 0.75) {
                printf("FAIL: boost not confined to the onset's band\n"); ok = false;
            }
            if (!finite8(b)) { printf("FAIL: non-finite output\n"); ok = false; }
        }
        bx->dispatcher(bx, effClose, 0, 0, nullptr, 0);
    }

    // --- lookahead Restore: refused where the host cannot compensate, and in
    //     a host that can, it delays cleanly and pulls down the quiet window
    //     ahead of a sharp attack -- where pre-echo lives. ---
    {
        if (fx->initialDelay != 0) {
            printf("FAIL: latency added under a host that reports Equalizer APO\n");
            ok = false;
        }
        AEffect* rx = entry(VSTCALLBACK_daw);
        rx->dispatcher(rx, effOpen, 0, 0, nullptr, 0);
        rx->dispatcher(rx, effSetSampleRate, 0, 0, nullptr, (float)fs);
        rx->dispatcher(rx, effSetBlockSize, 0, 512, nullptr, 0);
        rx->dispatcher(rx, effMainsChanged, 0, 1, nullptr, 0);
        int lat = rx->initialDelay;
        printf("restore: eapo latency=%d  daw latency=%d (expect ~%d)\n",
               fx->initialDelay, lat, (int)(0.032 * fs));
        if (lat < (int)(0.030 * fs) || lat > (int)(0.034 * fs)) {
            printf("FAIL: lookahead latency not reported correctly\n"); ok = false;
        }

        // Restore at 0 must be a pure delay: everything else idles on lossless.
        // Set the knobs BEFORE the mains cycle so the 30 ms parameter smoothers
        // initialise at zero instead of gliding down from their old values.
        setParams(rx, 0, 0, 0);
        rx->setParameter(rx, 1, 0.0f);
        rx->dispatcher(rx, effMainsChanged, 0, 0, nullptr, 0);
        rx->dispatcher(rx, effMainsChanged, 0, 1, nullptr, 0);
        Buffers d(1 * (int)fs);
        for (int i = 0; i < d.n; ++i) {
            float v = (float)(0.2 * std::sin(2.0 * M_PI * 700.0 * i / fs));
            d.in[0][i] = d.in[1][i] = v;
        }
        process(rx, d);
        // Skip the first 0.3 s: the 30 ms knob smoothers glide down from their
        // construction defaults, so the head of the buffer is genuinely filtered.
        double worst = 0.0;
        for (int i = (int)(0.3 * fs); i + lat < d.n; ++i)
            worst = std::max(worst, (double)std::fabs(d.out[0][i + lat] - d.in[0][i]));
        printf("restore: off -> pure delay, max|out[n+L]-in[n]| = %.3e\n", worst);
        if (worst > 1e-6) { printf("FAIL: Restore=0 is not a clean delay\n"); ok = false; }

        // A quiet HF bed (standing in for pre-echo) interrupted by loud clicks.
        // The 20 ms before each click must come down when Restore is up.
        auto preEchoRms = [&](float depth) {
            rx->dispatcher(rx, effMainsChanged, 0, 0, nullptr, 0);
            rx->dispatcher(rx, effMainsChanged, 0, 1, nullptr, 0);
            setParams(rx, 0, 0, 0);
            rx->setParameter(rx, 1, depth);
            Buffers b(3 * (int)fs);
            unsigned rs = 4242u;
            for (int i = 0; i < b.n; ++i) {
                rs = rs * 1664525u + 1013904223u;
                double nz = ((double)(int)(rs >> 1) / 1073741824.0 - 1.0) * 0.004;
                nz -= 0.5 * nz;                       // crude HF tilt
                double v = nz;
                int ph = i % (int)(0.5 * fs);
                if (ph < 200) v += 0.5 * std::exp(-ph / 40.0);   // click
                b.in[0][i] = b.in[1][i] = (float)v;
            }
            process(rx, b);
            // window: 20 ms .. 2 ms before the click at t = 2.0 s, delay-shifted
            size_t c0 = (size_t)(2.0 * fs) + lat;
            size_t w0 = c0 - (size_t)(0.020 * fs), w1 = c0 - (size_t)(0.002 * fs);
            double acc = 0.0;
            for (size_t i = w0; i < w1; ++i) acc += b.out[0][i] * (double)b.out[0][i];
            return std::sqrt(acc / (double)(w1 - w0));
        };
        double rOff = preEchoRms(0.0f), rOn = preEchoRms(1.0f);
        double cut = 20.0 * std::log10((rOn + 1e-12) / (rOff + 1e-12));
        printf("restore: pre-click window %.2e -> %.2e  (%+.1f dB)\n", rOff, rOn, cut);
        if (cut > -3.0) {
            printf("FAIL: Restore did not attenuate the pre-attack window\n"); ok = false;
        }
        // Dormancy WITH the delay line running. The gate measures peak on the
        // delayed signal and the silence gate deliberately leaves the delay
        // line alone, so sleeping mid-stream must not swallow or shift audio:
        // the wake lands exactly one latency after the input returns.
        //
        // This needs a genuinely empty delay line to mean anything, which is
        // what effMainsChanged now guarantees -- it clears the line, so the
        // tail of the preceding click test cannot drain into the head of this
        // buffer. (That leak is what left this check unasserted before: it
        // reported sample 0, and the fault was a stale line surviving the
        // transport restart, in the plugin rather than the harness.)
        {
            setParams(rx, 0, 0, 0);
            rx->setParameter(rx, 1, 1.0f);          // Restore fully up
            rx->dispatcher(rx, effMainsChanged, 0, 0, nullptr, 0);
            rx->dispatcher(rx, effMainsChanged, 0, 1, nullptr, 0);
            Buffers s(2 * (int)fs);                 // 1 s silence, then a tone
            int t0 = (int)fs;
            for (int i = t0; i < s.n; ++i) {
                float v = (float)(0.25 * std::sin(2.0 * M_PI * 900.0 * (i - t0) / fs));
                s.in[0][i] = s.in[1][i] = v;
            }
            process(rx, s);
            int first = -1;
            for (int i = 0; i < s.n; ++i)
                if (std::fabs(s.out[0][i]) > 1e-4f) { first = i; break; }
            long dm = (long)rx->dispatcher(rx, effVendorSpecific,
                                           REGEN_DORMANT_QUERY, 0, nullptr, 0);
            printf("restore: wake-from-sleep sample %d (expect %d = %d+L), dorm=0x%lx\n",
                   first, t0 + lat, t0, dm);
            // Slack upward only: the tone starts at a zero crossing, so the
            // first sample at t0+L is exactly 0 and the threshold is crossed a
            // sample or two later. Anything EARLIER than t0+L is stale audio.
            if (first < t0 + lat || first > t0 + lat + 12) {
                printf("FAIL: wake-from-sleep not one latency after input\n"); ok = false;
            }
            if (!finite8(s)) { printf("FAIL: non-finite output\n"); ok = false; }
        }
        rx->dispatcher(rx, effClose, 0, 0, nullptr, 0);
    }

    fx->dispatcher(fx, effClose, 0, 0, nullptr, 0);
    FreeLibrary(dll);
    printf(ok ? "\nALL CHECKS PASSED\n" : "\nCHECKS FAILED\n");
    return ok ? 0 : 1;
}
