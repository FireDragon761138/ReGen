# ReGen — lossy-audio reconstruction for Equalizer APO

A 64-bit VST2 plugin that cleans up and partially reconstructs lossy/legacy
game audio: low-sample-rate ADPCM sound effects, 128 kbps MP3 soundtracks,
early Ogg/XMA streams. 8-in/8-out, channel-role aware (standard Windows 7.1),
everything IIR, zero latency under Equalizer APO. (In a DAW the optional
lookahead **Restore** stage reserves a fixed 32 ms whether or not you turn it
up — see below. EAPO refuses that stage outright and stays at 0 samples.)

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
- **Regen** — **restores** the band the codec destroyed. The surviving octave
  below the wall is squared, which translates it up an octave, then highpassed
  into the empty region. Squaring rather than saturating matters: it keeps tonal
  content tonal and noise-like content noisy, where a tanh's odd harmonics pile
  intermodulation into dense material and read as fizz. The level comes from
  extrapolating the source's *own* spectral slope across the gap, so the
  synthesized band continues the recording instead of sitting on top of it as a
  flat ledge — and it is scaled by the surviving band's envelope, so it breathes
  with the music rather than becoming a static hiss bed.
- **Air** — tilt on the band that *survived*, hinged below the wall.
  Truncation doesn't only remove the top, it darkens the balance of everything
  left behind, and that is a separate defect needing a separate fix. This one
  invents nothing: it lifts content that is genuinely there.
- **Attack** — two-band transient restoration. Independent onset detectors and
  independent gains below and above 2 kHz, in the lineage of the X-Fi
  Crystalizer (which ran separate low- and high-frequency energy flux with
  proportionally weighted per-band boosts).

Every stage is scaled by two independent things, and keeping them separate is
the whole design — no preset system to manage:

- **`damage`** — *is this source damaged at all?* Measured as **octaves below
  the servo's ceiling**, not as an absolute frequency. Lossless pins the servo
  at the ceiling and scores exactly **0**; full at 0.40 octaves below. A ramp,
  not a step: bandwidth can rank how *audible* codec damage is — the artifacts'
  level falls as bitrate rises — but it cannot resolve adjacent bitrates on a
  knife edge.

  The ceiling is `min(20 kHz, 0.44 × fs)`, so it moves with the **host** sample
  rate: 20000 Hz at 48k (Equalizer APO, always), 19404 at 44.1k, 9702 at
  22.05k. Absolute thresholds would be unreachable at low host rates — offline
  remastering of a 22 kHz asset would peg every stage to full strength forever,
  by construction. Content *origin* is a separate matter: a CD rip resampled to
  a 48k device keeps its 22.05 kHz wall, clears the cap, and reads as lossless.
- **audibility weights** — *does this stage's action land where anyone can hear
  it?* Cleanup lowpasses at `corner × 1.25` and regen injects above
  `corner × 1.08`, so both march upward with the corner and can end up working
  entirely above the hearing limit. Each is weighted by where its action band
  actually falls, ramping to zero at the **16.5 kHz** anchor.

The smoother is the exception: its band (from `corner × 0.55`) is always
audible, but warble is a *bit-starvation* artifact that doesn't exist at high
bitrate, so it starts later — 0.20 octaves below the ceiling rather than at it.

| source | corner | damage | Cleanup | Regen | Air | Smooth | Attack |
|---|---|---|---|---|---|---|---|
| lossless (48k or CD) | 20000 Hz | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | **0.00** |
| high-quality Ogg | 19114 Hz | 0.16 | 0.00 | 0.00 | 0.16 | 0.00 | 0.16 |
| 320k MP3 | 17772 Hz | 0.43 | 0.00 | 0.00 | 0.43 | 0.00 | 0.43 |
| 192k MP3 | 16767 Hz | 0.64 | 0.00 | 0.00 | 0.64 | 0.27 | 0.64 |
| 128k MP3 | 14843 Hz | 1.00 | 0.00 | 0.45 | 1.00 | 1.00 | 1.00 |
| 22 kHz ADPCM | 10423 Hz | 1.00 | **1.00** | **1.00** | **1.00** | 1.00 | 1.00 |
| retro mix (128k + ADPCM) | 14435 Hz | 1.00 | 0.00 | 0.58 | 1.00 | 1.00 | 1.00 |

Air and Regen carry one further factor not shown, because it depends on content
rather than on the corner: **wall pressure**. It asks whether energy is actually
pressed up against the corner — meaning the encoder truncated something — or
whether the music simply rolled off on its own well below it. A hard brickwall
measures ~8 dB above the servo's own alive threshold and reads 1.00; content
pinned at the 6 kHz corner floor reads near zero and neither stage fires. It is
a coarse, slow term; moment-to-moment restraint comes from the source-band
envelope, which is exact.

Cleanup and regen concentrating on the *narrow-band* sources is not a
regression — it is `ReGen-retro-defaults.md`'s own diagnosis, now enforced in
code. That document already noted that with the corner parked at an MP3's
16 kHz, cleanup sits "around 20 kHz … so the effects' own 11–16 kHz codec hash
is never buried," and regen injects "above ~17 kHz, where it does almost
nothing audible." Both stages were doing measurable, inaudible work at full CPU
price. Now they don't.

Attack deserves a note of its own, because pre-echo is not a bandwidth defect.
Its **duration** is set by the codec's transform block length
— MP3 1152/384 samples (26.1 / 8.7 ms at 44.1 kHz), Vorbis 2048/256 (46.4 /
5.8 ms) — and those do not shorten when you raise the bitrate. Backward masking
covers only a few ms (premasking is usually put at 0.5–2 ms, effective under
5 ms, against 10–50 ms forward), so even a correctly switched MP3 short block
stays partly exposed. That is why transients fail at 192k as readily as at 128k.

But audibility is the smear's **level** against the masking threshold, and level
*does* fall as bitrate rises — which bandwidth tracks. So the corner can rank
pre-echo audibility even though it cannot rank pre-echo duration, which is why
`damage` is a legitimate driver for it and why the ramp is 0.4 octaves wide
rather than a cliff. A near-transparent format gets a near-inaudible correction:
320k measures **+0.57 dB**, under the ~0.5–1 dB level JND.

**Good content is left alone.** On lossless ReGen is bit-exact *including on
transients* — verified `max |out−in| = 0` over 25 s of burst-laden material
below the clip knee. This is deliberately **not** full Crystalizer emulation,
which enhanced everything unconditionally.

## Channels (Windows 7.1 order: FL FR FC LFE BL BR SL SR)

**Every channel gets the identical chain.** There is deliberately no role
differentiation — no speech-tuned centre, no scaled-down surrounds.

Two reasons. ReGen runs *first* under Equalizer APO, ahead of any upmixer, so
what it actually sees is stereo or positional-mono game audio and role logic
would almost never fire. And where it did fire it would be actively harmful: in
positional audio any sound can be in any channel, so treating surrounds more
gently makes a source **change character as it pans front to back**. That is
positional timbre inconsistency — an artifact the plugin would be introducing,
which is worse than leaving the channel alone.

LFE is the one exception, and it is passed through on physical grounds rather
than role ones: it carries nothing above ~120 Hz, and every repair stage here
acts above 6 kHz, so processing it is provably a no-op.

Detection and time-varying gains are shared per pair (FL+FR, SL+SR, BL+BR) so
imaging never wanders, and each pair sleeps independently when idle.

## Parameters

| Param | Default | What it does |
|---|---|---|
| Cleanup | 70 % | dry/filtered mix of the adaptive lowpass |
| Smooth | 50 % | top-octave flutter clamping |
| Regen | 60 % | level of the restored (synthesized) band above the wall |
| Air | 40 % | tilt on the surviving band, up to +10 dB, hinged at `corner × 0.55` |
| Attack | 35 % | two-band transient restoration (up to +6 dB) |
| Mix | 100 % | global dry/wet — the safety valve |
| Restore | **0 %** | lookahead pre-echo suppression — see below. Costs 32 ms of latency, so it is **refused under Equalizer APO** and reads `n/a` there |
| Freeze | Off | locks the detector — **leave off** unless you know why (see below) |

The editor's status line shows the detected rolloff per group
(Front / Center / Side / Back, in kHz). **This is only live in a DAW** — Equalizer
APO's Configuration Editor instantiates a separate copy of the plugin to draw
the GUI, so under EAPO that line never sees your audio and stays at its startup
value. **Defaults** restores everything.

The defaults target the common late-90s/2000s PC case: a compressed soundtrack
(128k MP3, early Ogg/XMA) mixed with a 22 kHz ADPCM effects bank. Because the
detector parks at the widest source and cleanup/regen saturate at or below
15 kHz, the same settings serve a pure-ADPCM game and an MP3-scored one — the
servo just places the filters differently. A measured mixed stream (128k music +
22 kHz ADPCM effects) parks at ~14.4 kHz: cleanup/regen at full, `audDeficit`
0.83. Modern/lossless content needs no change: the repair stages idle as the
corner approaches full bandwidth, verified `max |out−in| = 0` over 25 s of steady
material below the clip knee. Attack keeps working on transients — see above.

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

## Restore — actually removing pre-echo, not masking it

Every other stage **masks** pre-echo: Attack lifts the transient so backward
masking covers more of the smear. That is all a zero-latency process can do, and
it isn't much — backward masking runs 0.5–2 ms against an MP3 short block's
8.7 ms of spread.

Lookahead inverts the problem. Pre-echo occupies the window *before* an attack;
with 32 ms of delay the attack is detected while that window is still sitting in
the delay line, unemitted, so it can be **attenuated** rather than hidden. That
is removal of added noise — correction of the decoded signal's temporal
envelope. Measured on a quiet high-frequency bed interrupted by clicks, the
20 ms ahead of each click drops **6.2 dB**.

It reconstructs the *envelope*, not the *content*: detail the quantiser threw
away inside the transient is gone for good. The envelope is what reads as smear,
so that is the part worth having.

The false-positive guard is the literature's own precondition — pre-echo needs a
sharp attack "immediately following a region of low energy" — so the stage only
fires when the pre-window sits ~16 dB under the attack. That keeps it off snare
rolls, crescendos and reverb tails, where a naive version would punch holes.
Suppression is high-band only (above 3 kHz) and ramped at both edges, so there
is no step at the transient itself.

**Off by default, and refused outright under Equalizer APO**, which does no
plugin delay compensation — 32 ms of uncompensated latency would wreck A/V sync.
The host check is deliberately asymmetric: it refuses only on a *positive*
identification, so an unidentified host (a minimal offline harness, say) still
works, and Restore is off there until you ask for it. Latency is fixed whenever
it is permitted rather than following the knob, so turning the knob never makes
the host re-negotiate compensation mid-stream. With Restore at 0 the plugin is a
bit-exact pure delay — verified, `max |out[n+L] − in[n]| = 0`.

This is aimed at *offline* work — the natural home is a remaster pipeline, where
latency is free and the sources are exactly the damaged legacy assets it exists
for.

## DAW use

ReGen also loads in ordinary VST2 DAW hosts (exports both `VSTPluginMain` and
the legacy `main`): GUI edits are reported via `audioMasterAutomate` so
automation records, host automation/preset loads are reflected back into the
GUI, out-of-range automation values are clamped, and state saves through the
normal parameter mechanism (no chunks needed).

**Idle gate:** after 250 ms below ~-100 dBFS the plugin flushes its (already
decayed) state and sleeps — silent DAW tracks cost almost nothing. Each role
group *also* sleeps on its own pair, so a stereo source in a 7.1 stream runs one
group instead of four. The wake check runs *before* the skip each sample, so the
first audible sample after silence is processed normally: onsets are never
clipped or delayed, and the learned rolloff corners survive dormancy. In EAPO
gameplay audio is never digitally silent, so behavior there is unchanged
(menus/pauses now idle for free).

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

CPU (measured, 48 kHz, stereo content in a 7.1 stream — the Equalizer APO case,
where ReGen sits ahead of any upmixer so 6 channels are silent):

| source | before | now |
|---|---|---|
| lossless | 87.4 ns/frame (0.42 %) | **40.0 ns (0.19 %)** |
| 320k MP3 | 124.9 ns (0.60 %) | **49.5 ns (0.24 %)** |
| 192k MP3 | 124.8 ns (0.60 %) | **57.2 ns (0.27 %)** |
| 128k MP3 | 125.6 ns (0.60 %) | **81.8 ns (0.39 %)** |
| 22 kHz ADPCM | 124.8 ns (0.60 %) | **85.1 ns (0.41 %)** |

Most of that is the per-group idle gate: the plugin-wide gate keys off the peak
across all eight channels, so one active channel used to keep every group
running — measured, six digitally silent channels cost exactly as much as six
active ones. Groups now sleep on their own pair. Latency: 0 samples. Denormals are flushed
(FTZ/DAZ) — envelope followers decaying to silence are a classic denormal
trap.

## Tooling hook

`effVendorSpecific` with index `'Roff'` and value 0–3 (front/center/side/back)
returns that group's detected corner in Hz — used by the test host, handy for
scripted verification.

Three further queries return the restoration stage's internal terms as
*value × 1000*, so it can be tuned against measurements instead of inferred from
the output: `'Prss'` (wall pressure), `'Slop'` (extrapolated level ratio for the
restored band) and `'Tonl'` (tonality, 1 = tonal, 0 = noise-like). `'Dorm'`
returns a bitmask of sleeping groups.

**Note:** these are the only way to observe the detector. Equalizer APO's
Configuration Editor instantiates a *separate* copy of the plugin to draw its
GUI, so the editor's rolloff status line never sees your audio and sits at its
startup value — it is only live in a DAW.
