# ReGen — proposed "retro gaming" defaults

> **Partly superseded.** This document analyses the single `deficit` factor
> `clamp01((19500 − corner) / 3500)`. That factor is now two — `deficit` for
> cleanup/regen, `audDeficit` for the smoother — and **Attack is driven by
> neither**: pre-echo spread is set by the codec's transform block length, not
> its bitrate or its lowpass, so no bandwidth ramp gates it correctly. See the
> README. The Attack gain-law floor this document recommends removing was
> removed. Its 50 % alternative default was *not* adopted, because that branch
> assumed Attack would self-disable on content that doesn't need it, and the
> corner-independent design gives that up deliberately; the conservative 35 %
> is what ships. The retro conclusions still hold: a measured mixed stream
> parks at ~14.4 kHz, where cleanup/regen are at full strength.

A single set of shipping defaults intended to cover a PC library spanning roughly
1998–2008: 22 kHz / ADPCM effect banks, 128 kbps MP3 soundtracks, early Ogg and
XMA streams, and the frequent case of *both at once in the same stream*.

## The design assumption these defaults are built on

The mixed stream is not an edge case. From about 1999 to 2006 it was close to
standard PC practice: music was a small number of long assets, so a perceptual
codec paid for itself and a decode thread was affordable; effects were hundreds
of short files needing low-latency triggering, so they stayed as cheap ADPCM or
22 kHz PCM. Miles, FMOD and Bink all made bolting MP3 music onto an existing
22 kHz effects bank trivial, and console ports inherited the split by default —
PS1/PS2-lineage effects, fresh compressed soundtrack for the PC build. THPS3,
GTA III / Vice City / San Andreas, the late Sierra and LucasArts catalog, the
Need for Speed and Colin McRae games of the era are all this shape.

The consequence for ReGen's servo is direct. Both sources land in the same
channel group, the upper probe sees the music's content, `stepUp` fires, and the
corner parks at the **union** — i.e. at the MP3's rolloff, around 16 kHz. So on a
retro library the plugin spends most of its time sitting at an MP3 corner
regardless of what the effects are doing. Defaults should be chosen for the
corner the plugin will actually occupy, not for the pure-22 kHz case.

The good news is that this costs less than it sounds. `deficit` is
`clamp01((19500 − corner) / 3500)`, so it saturates at 1.0 anywhere at or below
16 kHz. An 11 kHz ADPCM corner and a 16 kHz MP3 corner drive the chain at
*identical* strength. The difference between them is only where the filters land,
and the servo places those automatically. One knob setting genuinely serves both.

## Proposed defaults

| Param | Current | Proposed | Rationale |
|---|---|---|---|
| Cleanup | 70 % | **70 %** | unchanged |
| Smooth | 50 % | **50 %** | unchanged |
| Regen | 50 % | **60 %** | the union corner is the MP3 corner; center the default there |
| Attack | 35 % | **35 %** (or 50 %, *conditional* — see below) | unchanged unless the gain law changes |
| Mix | 100 % | **100 %** | unchanged |
| Freeze | Off | **Off** | must stay off; see the footgun below |

### Regen 50 → 60

Low risk, clear benefit. If the corner sits at the MP3's rolloff most of the
time, 60 % is the right center of gravity for the content the plugin will most
often be treating. On the pure-22 kHz games it costs nothing: deficit is
saturated at 1.0 in both cases, so an ADPCM asset simply receives a little more
synthesized top — the direction you want anyway. This is the one default worth
moving unconditionally.

### Attack stays at 35 % — unless the gain law is fixed first

The README's suggested MP3 starting point raises Attack to 50 %. I would not
promote that to a default as the code currently stands, for two reasons.

First, transient boost exists to mask **pre-echo**, and pre-echo is a
transform-codec artifact — MDCT block smearing. ADPCM is a time-domain
predictor and has no pre-echo to mask. On the 22 kHz half of the library, Attack
isn't repairing anything; it's a taste effect.

Second, and more seriously, Attack is the only stage that never idles out:

```c
tg = 1.0 + trn * (0.4 + 0.6 * deficit) * 0.8 * onset;
```

Cleanup, Smooth and Regen all multiply straight by `deficit` and go to zero as
the corner reaches 19.5 kHz. Attack retains 40 % of its authority on
full-bandwidth modern content. Raising its default therefore increases
unrequested transient shaping on audio that has nothing wrong with it, which
quietly contradicts the README's claim that pristine content automatically gets a
light touch and that there is no preset system to manage.

**Recommended change:** remove the floor —

```c
tg = 1.0 + trn * deficit * 0.8 * onset;          // or (0.15 + 0.85 * deficit)
```

With the floor gone, Attack self-disables on content that doesn't need it, and a
**50 %** default becomes defensible and should ship alongside Regen 60 %. Until
then, 35 % is the correct conservative value.

## Freeze must default off — and is a footgun in EAPO

Equalizer APO loads the plugin once for the entire system, not per application.
A frozen corner therefore persists across application switches. Freeze at 11 kHz
for an old game, quit, launch something modern, and you are now lowpassing real
content with a stale corner and no servo to recover. Freeze is for a single
session with genuinely uniform assets, and it wants explicitly unfreezing
afterwards. For a library spanning both eras, the free-running detector is the
entire point of the design.

This is worth stating in the README more forcefully than the current wording
("for games with uniform assets") does. Consider also auto-clearing Freeze on
`resume`, or on any `setSampleRate` / `resetState` where the host changes.

## What these defaults deliberately do *not* fix

In a genuinely mixed stream, the effects are under-served. Cleanup is placed at
`corner × 1.25` — around 20 kHz when the corner has learned the MP3's 16 kHz — so
the effects' own 11–16 kHz codec hash is never buried. Regeneration injects above
`corner × 1.08`, i.e. above ~17 kHz, where it does almost nothing audible for a
source that dies at 11 kHz.

No knob setting repairs this, because the two sources share a channel group and a
per-sample IIR chain on a summed signal cannot treat them differently. Better
*detection* is cheap — the servo runs at control rate (every 64 samples, ~750 Hz)
and another probe pair is on the order of 0.1 % of a core — but detection alone
buys nothing without separation, and separation means either spectral processing
(which destroys the zero-latency, all-IIR premise) or transient/steady-state
gating (cheap, but crude, and it will misfire: music has transients, ambience
loops are steady).

The one cheap structural improvement available is to **decouple which corner each
stage keys off**. Cleanup should keep tracking the union corner — you never want
to lowpass real content, so following the widest source is correct there. But the
regeneration highpass could key off a second, lower, slower-tracking corner
estimate — the one the effects actually occupy — so that injection lands in the
11–16 kHz gap where the effects are dead rather than above 17 kHz where nothing
is listening. Cost: one additional probe pair and a slew-limited estimate.

The trade is real and not free: during music-only passages the plugin would then
be injecting synthesized harmonics on top of genuine MP3 content in that band,
and since regen is level-linked to the 3–8 kHz envelope, music keeps it active.
Whether that reads as air or as hash is an empirical question. The current
union-tracking behavior is the conservative choice and should remain the default
until that experiment is run.

## Summary of what to change

Ship **Regen at 60 %** and leave everything else as-is. Strengthen the Freeze
warning in the README. If the Attack gain-law floor is removed, promote Attack to
**50 %** at the same time; otherwise leave it at 35 %.
