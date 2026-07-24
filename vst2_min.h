// vst2_min.h
// Minimal, clean-room VST 2.4 interface definitions -- just enough to build an
// audio-effect plugin. No Steinberg SDK required. These are the ABI-level struct
// layouts and opcode numbers a VST2 host expects; nothing proprietary here.
//
// Public-domain / do-whatever. Matches the VST 2.4 binary interface.
#ifndef VST2_MIN_H
#define VST2_MIN_H

#include <stdint.h>

typedef int32_t   VstInt32;
typedef intptr_t  VstIntPtr;   // pointer-sized: 64-bit on x64 builds

struct AEffect;

// Host -> plugin callback type.
typedef VstIntPtr (*audioMasterCallback)(AEffect* effect, VstInt32 opcode,
                                          VstInt32 index, VstIntPtr value,
                                          void* ptr, float opt);

// Plugin function pointer types.
typedef VstIntPtr (*AEffectDispatcherProc)(AEffect* effect, VstInt32 opcode,
                                           VstInt32 index, VstIntPtr value,
                                           void* ptr, float opt);
typedef void  (*AEffectProcessProc)(AEffect* effect, float** inputs,
                                    float** outputs, VstInt32 sampleFrames);
typedef void  (*AEffectProcessDoubleProc)(AEffect* effect, double** inputs,
                                          double** outputs, VstInt32 sampleFrames);
typedef void  (*AEffectSetParameterProc)(AEffect* effect, VstInt32 index, float parameter);
typedef float (*AEffectGetParameterProc)(AEffect* effect, VstInt32 index);

struct AEffect {
    VstInt32 magic;                       // kEffectMagic ('VstP')
    AEffectDispatcherProc    dispatcher;
    AEffectProcessProc       process;     // legacy (accumulating); unused here
    AEffectSetParameterProc  setParameter;
    AEffectGetParameterProc  getParameter;

    VstInt32 numPrograms;
    VstInt32 numParams;
    VstInt32 numInputs;
    VstInt32 numOutputs;
    VstInt32 flags;

    VstIntPtr resvd1;
    VstIntPtr resvd2;

    VstInt32 initialDelay;

    VstInt32 realQualities;               // deprecated
    VstInt32 offQualities;                // deprecated
    float    ioRatio;                     // deprecated

    void* object;                         // we stash our plugin instance here
    void* user;

    VstInt32 uniqueID;
    VstInt32 version;

    AEffectProcessProc        processReplacing;
    AEffectProcessDoubleProc  processDoubleReplacing;

    char future[56];
};

// Editor rectangle the host reads via effEditGetRect. Fields are 16-bit and in
// this exact order (top, left, bottom, right) per the VST2 ABI.
struct VstRect {
    int16_t top;
    int16_t left;
    int16_t bottom;
    int16_t right;
};

// ---- magic / category ----
#define CCONST(a, b, c, d) \
    ((((VstInt32)(a)) << 24) | (((VstInt32)(b)) << 16) | \
     (((VstInt32)(c)) << 8)  |  ((VstInt32)(d)))

#define kEffectMagic      CCONST('V', 's', 't', 'P')
#define kVstLangEnglish   1
#define kPlugCategEffect  1

// ---- effect flags ----
enum {
    effFlagsHasEditor          = 1 << 0,
    effFlagsCanReplacing       = 1 << 4,
    effFlagsProgramChunks      = 1 << 5,
    effFlagsIsSynth            = 1 << 8,
    effFlagsNoSoundInStop      = 1 << 9,
    effFlagsCanDoubleReplacing = 1 << 12
};

// ---- dispatcher opcodes (host -> plugin) ----
enum {
    effOpen = 0,
    effClose,
    effSetProgram,
    effGetProgram,
    effSetProgramName,
    effGetProgramName,
    effGetParamLabel,
    effGetParamDisplay,
    effGetParamName,
    effGetVu_DEPRECATED,        // 9
    effSetSampleRate,           // 10
    effSetBlockSize,            // 11
    effMainsChanged,            // 12
    effEditGetRect,             // 13
    effEditOpen,                // 14
    effEditClose,               // 15
    effEditDraw_DEPRECATED,
    effEditMouse_DEPRECATED,
    effEditKey_DEPRECATED,
    effEditIdle,                // 19
    effEditTop_DEPRECATED,
    effEditSleep_DEPRECATED,
    effIdentify_DEPRECATED,
    effGetChunk,                // 23
    effSetChunk,                // 24
    effProcessEvents = 25,
    effCanBeAutomated = 26,
    effGetProgramNameIndexed = 29,
    effGetPlugCategory = 35,
    effGetEffectName = 45,
    effGetVendorString = 47,
    effGetProductString = 48,
    effGetVendorVersion = 49,
    effVendorSpecific = 50,     // plugin-private queries (ReGen: rolloff readout)
    effCanDo = 51,
    effGetVstVersion = 58
};

// ---- audioMaster opcodes (plugin -> host) ----
enum {
    audioMasterAutomate = 0,    // index=param, opt=value: host records automation
    audioMasterVersion  = 1,
    audioMasterIOChanged     = 13,  // latency/IO changed: host must re-negotiate
    audioMasterGetProductString = 33 // ptr=char[64]: host's own name
};

#endif // VST2_MIN_H
