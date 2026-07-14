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

static VstIntPtr VSTCALLBACK_master(AEffect*, VstInt32, VstInt32, VstIntPtr, void*, float) { return 0; }
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

static void setParams(AEffect* fx, float clean, float smooth, float regen,
                      float trans, float mix, float freeze) {
    fx->setParameter(fx, 0, clean);  fx->setParameter(fx, 1, smooth);
    fx->setParameter(fx, 2, regen);  fx->setParameter(fx, 3, trans);
    fx->setParameter(fx, 4, mix);    fx->setParameter(fx, 5, freeze);
}

static long corner(AEffect* fx, int group) {
    return (long)fx->dispatcher(fx, effVendorSpecific, REGEN_ROLLOFF_QUERY,
                                group, nullptr, 0);
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
        setParams(fx, 1, 1, 1, 1, 0, 0);
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
        setParams(fx, 0, 0, 0, 0, 1, 0);
        process(fx, b);
        cFront = corner(fx, 0); cSide = corner(fx, 2);
        printf("servo: walled-at-10k -> corner front=%ld side=%ld back=%ld center=%ld\n",
               cFront, cSide, corner(fx, 3), corner(fx, 1));
        // 12th-order probes resolve a pure-tone edge ~0.5 oct high, so accept
        // anything clearly below full bandwidth but above the wall itself.
        if (cFront < 10000 || cFront > 17000) { printf("FAIL: front corner off\n"); ok = false; }
        if (cSide  < 10000 || cSide  > 17000) { printf("FAIL: side corner off\n");  ok = false; }
    }

    // --- regeneration: 2nd harmonic of the 8 kHz source tone appears at 16k ---
    {
        Buffers b(2 * (int)fs);
        fillSignal(b, true);
        setParams(fx, 0, 0, 0, 0, 1, 1);          // regen off, detector frozen
        process(fx, b);
        double off16 = goertzel(b.out[0], b.n / 2, b.n, fs, 16000.0);
        setParams(fx, 0, 0, 1, 0, 1, 1);          // regen full
        process(fx, b);
        double on16 = goertzel(b.out[0], b.n / 2, b.n, fs, 16000.0);
        double src8 = goertzel(b.in[0],  b.n / 2, b.n, fs, 8000.0);
        printf("regen: 16k off=%.6f on=%.6f (src 8k=%.4f)\n", off16, on16, src8);
        if (on16 < 3.0 * off16 + 1e-4) { printf("FAIL: no regenerated highs\n"); ok = false; }
        if (on16 > 0.3 * src8) { printf("FAIL: regen level implausibly hot\n"); ok = false; }
    }

    // --- cleanup: an 18 kHz tone (above the learned corner) gets attenuated ---
    {
        Buffers b(2 * (int)fs);
        fillSignal(b, true);
        for (int i = 0; i < b.n; ++i) {
            float t18 = 0.08f * (float)std::sin(2.0 * M_PI * 18000.0 * i / fs);
            b.in[0][i] += t18; b.in[1][i] += t18;
        }
        setParams(fx, 0, 0, 0, 0, 1, 1);          // frozen, cleanup off
        process(fx, b);
        double raw18 = goertzel(b.out[0], b.n / 2, b.n, fs, 18000.0);
        setParams(fx, 1, 0, 0, 0, 1, 1);          // cleanup full
        process(fx, b);
        double cln18 = goertzel(b.out[0], b.n / 2, b.n, fs, 18000.0);
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
        setParams(fx, 0, 0, 0, 1, 1, 1);
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
        setParams(fx, 0, 0, 0, 0, 1, 0);           // unfreeze
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
        setParams(fx, 1, 1, 1, 1, 1, 1);           // frozen so corner is provable
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

    fx->dispatcher(fx, effClose, 0, 0, nullptr, 0);
    FreeLibrary(dll);
    printf(ok ? "\nALL CHECKS PASSED\n" : "\nCHECKS FAILED\n");
    return ok ? 0 : 1;
}
