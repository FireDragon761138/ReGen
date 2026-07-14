# ReGen — lossy-audio reconstruction for Equalizer APO

A 64-bit VST2 plugin that cleans up and partially reconstructs lossy/legacy
game audio: low-sample-rate ADPCM sound effects, 128 kbps MP3 soundtracks,
early Ogg/XMA streams. 8-in/8-out, channel-role aware (standard Windows 7.1),
zero latency, everything IIR.

License: BSD-2-Clause. No Steinberg SDK — built against the clean-room
`vst2_min.h` interface definitions.

## How it works

The spine is a per-channel-group **rolloff servo**: two steep (12th-order IIR
highpass) probes straddle a running estimate of where legitimate content dies.
If the upper probe still sees energy, the corner slews up fast; if even the
lower probe is dead, it slews down slowly. Gated peak-hold meters keep silence
and bass-only passages from dragging the estimate around. A 22 kHz-resampled
asset, a 16 kHz-brick-walled MP3, and full-bandwidth content all settle at
different corners — and everything downstream keys off that one measurement:

- **Cleanup** — adaptive 12 dB/oct lowpass placed just above the corner buries
  codec swirl and quantization hash. Gentle slope so it reads as smoothness.
- **Smooth** — envelope-ratio gain on the top octave clamps warbly level
  flutter (attenuate-only, so stable content is untouched).
- **Regen** — the 3–8 kHz band is soft-saturated to generate harmonics, then
  highpassed *above the detected corner* and mixed back in, level-linked to
  the real midrange envelope so quiet passages don't hiss.
- **Attack** — zero-lookahead transient boost masks codec pre-echo smear on
  gunshots / footsteps / hi-hats.

A *deficit* factor derived from the corner (0 at ~19.5 kHz, 1 at ≤16 kHz)
scales the whole chain, so pristine content automatically gets a light touch —
there is no preset system to manage.

## Channel roles (Windows 7.1 order: FL FR FC LFE BL BR SL SR)

| Role | Treatment |
|---|---|
| FL/FR | full chain |
| FC | speech-tuned: no transient shaper; a sibilance detector ducks the exciter on /s/ so dialogue doesn't spit |
| SL/SR, BL/BR | cleanup + lighter regeneration (they mostly carry ambience) |
| LFE | untouched passthrough |

Detection and time-varying gains are shared per pair (FL+FR, SL+SR, BL+BR) so
imaging never wanders; only static per-channel filtering differs.

## Parameters

| Param | Default | What it does |
|---|---|---|
| Cleanup | 70 % | dry/filtered mix of the adaptive lowpass |
| Smooth | 50 % | top-octave flutter clamping |
| Regen | 60 % | synthesized-highs level |
| Attack | 50 % | transient onset boost (up to +6 dB) |
| Mix | 100 % | global dry/wet — the safety valve |
| Freeze | Off | locks the detector — **leave off** unless you know why (see below) |

The editor's status line shows the live detected rolloff per group
(Front / Center / Side / Back, in kHz) — watch it settle for a few seconds
after audio starts. **Defaults** restores everything.

The defaults target the common late-90s/2000s PC case: a compressed soundtrack
(128k MP3, early Ogg/XMA) mixed with a 22 kHz ADPCM effects bank. Because the
detector parks at the widest source and the deficit factor saturates at or below
16 kHz, the same settings serve a pure-ADPCM game and an MP3-scored one — the
servo just places the filters differently. Modern/lossless content needs no
change: the deficit factor idles the whole chain (including Attack) as the corner
approaches full bandwidth.

**Freeze — leave it off.** Freeze locks the detector at wherever the corner
happens to be. Equalizer APO loads *one* instance for the whole system, not per
application, so a corner frozen for an old game persists after you quit it and
will lowpass whatever you launch next, with no servo left to recover. Only use
Freeze for a single session with genuinely uniform assets, and clear it when
you're done. (The plugin also auto-clears Freeze whenever the host restarts the
audio pipeline, so a device/format change won't strand a stale corner.)

## Equalizer APO setup

Add to your config (full path, or relative to the EAPO `VSTPlugins` folder):

```
VSTPlugin: Library "C:\work\ReGen\ReGen.dll"
```

Chain order: ReGen runs **first** (repair the source before anything else),
ahead of PsychoBass / SonicStorm / SonicStormHP.

## DAW use

ReGen also loads in ordinary VST2 DAW hosts (exports both `VSTPluginMain` and
the legacy `main`): GUI edits are reported via `audioMasterAutomate` so
automation records, host automation/preset loads are reflected back into the
GUI, out-of-range automation values are clamped, and state saves through the
normal parameter mechanism (no chunks needed).

**Idle gate:** after 250 ms below ~-100 dBFS the plugin flushes its (already
decayed) state and sleeps — silent DAW tracks cost almost nothing. The wake
check runs *before* the skip each sample, so the first audible sample after
silence is processed normally: onsets are never clipped or delayed, and the
learned rolloff corners survive dormancy. In EAPO gameplay audio is never
digitally silent, so behavior there is unchanged (menus/pauses now idle for
free).

## Build & test

Run `build_mingw.bat` (needs WinLibs MinGW-w64 g++ on PATH). It produces
`ReGen.dll` plus `test_host.exe`, a mini-host that replicates Equalizer APO's
quirks (8-byte param strings, garbage-rect `effEditGetRect`) and verifies the
servo, cleanup, regeneration, transient boost, and Mix=0 transparency:

```
test_host.exe
```

Optional: `build_mingw.bat avx2` produces `ReGen-AVX2.dll`, built for
x86-64-v3 (AVX2+FMA, Haswell/Ryzen or newer — ~2013+). FMA fuses the
multiply-adds in the filter chains for a measured ~25 % lower CPU cost. It
will crash on older CPUs, so the generic build stays the default; load one or
the other, not both.

CPU (measured, 8 channels @ 48 kHz): ~0.9 % of one core fully active on lossy
content, ~0.6 % on quiet-but-not-silent content, ~0 when the idle gate engages
(digital silence > 250 ms). Latency: 0 samples. Denormals are flushed
(FTZ/DAZ) — envelope followers decaying to silence are a classic denormal
trap.

## Tooling hook

`effVendorSpecific` with index `'Roff'` and value 0–3 (front/center/side/back)
returns that group's detected corner in Hz — used by the test host, handy for
scripted verification.
