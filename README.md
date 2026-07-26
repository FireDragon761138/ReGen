# ReGen — lossy-audio reconstruction for Equalizer APO

A 64-bit VST2-compatible plugin that cleans up and partially reconstructs lossy/legacy
game audio: low-rate Vorbis/MP3 streams and low-sample-rate (22/11 kHz-class)
assets. 8-in/8-out, everything IIR, zero latency under Equalizer APO. (In a
DAW the optional lookahead **Restore** stage reserves a fixed 32 ms whether or
not you turn it up — see below. EAPO refuses that stage outright and stays at
0 samples.)

License: BSD-2-Clause. No Steinberg SDK — built against the clean-room
`vst2_min.h` interface definitions.

## How it works

The spine is a per-channel-group **rolloff servo**: two steep (12th-order IIR
highpass) probes straddle a running estimate of where legitimate content dies.
Upward moves require live evidence on a fast meter (debounced so signal edges
and filter rings don't count); downward moves are slow and peak-hold-guarded so
silence and bass-only passages don't drag the estimate. Everything downstream
keys off that one measurement:

- **Cleanup** — adaptive 12 dB/oct lowpass just above the corner buries codec
  swirl and quantization hash.
- **Warble detection** — not a repair stage: the top coded octave's envelope is
  compared against the octave below it in the 8–40 Hz modulation band (codec
  frame rates). Detected flutter slides the whole dull-and-replace boundary
  down (up to 0.4 oct), so an unstable top octave is dulled and rebuilt by
  translation instead of clamped. A correlation-sign gate keeps band-local
  musical modulation (arpeggios, PWM) from triggering it.
- **Regen** — squaring translation: the octave below the corner, squared (sum
  terms land one octave up; difference terms die in the injection highpass),
  envelope-normalized, and level-matched closed-loop to the source's own
  spectral slope extrapolated one octave. Level is tempered when the donor
  octave itself flutters. Synthesis stops at the 16.5 kHz audibility anchor.
- **Air** — boost-only tilt correction of the surviving band's top toward a
  built-in neutral template (−2 dB/oct); self-limiting, capped 3 dB, nothing
  on sources already at or above the template.
- **Attack** — two-band transient restoration: independent onset detectors and
  gains below and above 2 kHz, active only on damaged sources.

Every stage is scaled by two independent factors:

- **`damage`** — *is this source damaged at all?* Octaves below the servo's
  ceiling (`min(20 kHz, 0.44 × fs)`, host-rate-relative). Lossless pins the
  servo at the ceiling and scores exactly **0** — bit-exact passthrough,
  verified `max |out−in| = 0` below the clip knee. Full at 0.40 oct below.
- **audibility weights** — *does the stage's action land where anyone can hear
  it?* Action bands march with the corner and are weighted by where they
  actually fall, ramping to zero at the 16.5 kHz anchor.

| source class | behavior |
|---|---|
| lossless / full-bandwidth | bit-exact passthrough (all stages idle) |
| well-encoded moderate compression (128k-class MP3) | light touch: transient restoration, sub-JND elsewhere |
| low-rate Vorbis / 22 kHz-class assets | full chain: dull, rebuild, tilt-correct |
| 11 kHz-class assets | one octave rebuilt above the wall |

## Channels (Windows 7.1 order: FL FR FC LFE BL BR SL SR)

Every channel gets the identical chain; LFE passes through untouched. Channels
are grouped into pairs (FL+FR, SL+SR, BL+BR) plus mono FC for detection:
time-varying gains are shared per pair so imaging never wanders.

## Parameters

| Param | Default | What it does |
|---|---|---|
| Repair | 60 % | the one knob: scales dull, warble-driven replacement, translation level (100 % = slope target), Air, and transient restoration together |
| Restore | **0 %** | lookahead pre-echo suppression — costs 32 ms latency, **refused under Equalizer APO** |
| Freeze | Off | locks the detector — **leave off** unless you know why (see below) |

The editor's status line shows the live detected rolloff per group
(Front / Center / Side / Back, in kHz). **Defaults** restores everything.

**Freeze — leave it off.** Freeze locks the detector at wherever the corner
happens to be. Equalizer APO loads *one* instance for the whole system, so a
corner frozen for one source persists and will lowpass whatever plays next.
The plugin auto-clears Freeze whenever the host restarts the audio pipeline.

## Equalizer APO setup

Add to your config (full path, or relative to the EAPO `VSTPlugins` folder):

```
VSTPlugin: Library "C:\work\ReGen\ReGen.dll"
```

Chain order: ReGen runs **first** (repair the source before anything else),
ahead of PsychoBass / SonicStorm / SonicStormHP.

## Restore — removing pre-echo, not masking it

Attack can only *mask* pre-echo (lift the transient so backward masking covers
more smear). With 32 ms of lookahead the attack is detected while the
pre-window is still in the delay line, so the smear is **attenuated** before
it is emitted. Envelope correction only: detail the quantizer discarded stays
gone.

It fires only on a sharp attack out of a quiet window (the literature's
pre-echo precondition), keeping it off rolls, crescendos, and reverb tails.
Suppression is high-band only (above 3 kHz), edge-ramped. With Restore at 0
the plugin is a bit-exact pure delay (`max |out[n+L] − in[n]| = 0`).

Off by default; refused under Equalizer APO (no plugin delay compensation
there). Latency is fixed whenever the host permits it, so knob moves never
force a mid-stream PDC renegotiation. Natural home: offline remaster work.

## DAW use

Loads in ordinary VST2 hosts (exports `VSTPluginMain` and legacy `main`):
GUI edits report via `audioMasterAutomate`, host automation reflects back into
the GUI, out-of-range values clamp, state saves through plain parameters.

**Idle gate:** after 250 ms below ~−100 dBFS the plugin flushes state and
sleeps; each channel group also sleeps on its own pair, so a stereo source in
a 7.1 stream runs one group instead of four. The wake check runs before the
skip, so the first audible sample after silence is processed normally and
learned corners survive dormancy.

## Build & test

Run `build_mingw.bat` (needs WinLibs MinGW-w64 g++ on PATH). It produces
`ReGen.dll` plus `test_host.exe`, a mini-host that replicates Equalizer APO's
quirks (8-byte param strings, garbage-rect `effEditGetRect`) and verifies the
servo, translation, cleanup, transient boost, dormancy, Restore, and
idle-transparency:

```
test_host.exe
```

Optional: `build_mingw.bat avx2` produces `ReGen-AVX2.dll` (x86-64-v3:
AVX2+FMA, ~2013+ CPUs; lower CPU on the filter chains). It will crash on older
CPUs, so the generic build stays the default; load one or the other, not both.

Latency: 0 samples under EAPO. Denormals are flushed (FTZ/DAZ).

## Tooling hooks

`effVendorSpecific` queries, used by the harness and the corpus tools in
`tools/` (offline processing, corpus generation, perceptual metering):

| index | returns |
|---|---|
| `'Roff'`, value 0–3 | that group's detected corner in Hz |
| `'Dorm'` | bitmask of dormant groups |
| `'Dbg0'`, value 0–9 | group-0 stage internals (diagnostics) |

SPDX-License-Identifier: GPL-2.0-only

ReGen — adaptive reconstruction for lossy and band-limited game audio
Copyright (C) 2026 <FireDragon761138>

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License, version 2,
as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, see <https://www.gnu.org/licenses/>.
