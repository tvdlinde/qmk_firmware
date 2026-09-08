# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

This is Tim's fork of QMK Firmware. The vast majority of the tree is upstream
QMK (thousands of community keyboards); the part Tim actually maintains is his
three personal keymaps, all named **`kiruna`**:

- `keyboards/preonic/keymaps/kiruna/` — build target `preonic/rev3 -km kiruna`
- `keyboards/sofle/keymaps/kiruna/` — build target `sofle/rev1 -km kiruna`
- `keyboards/zsa/moonlander/keymaps/kiruna/` — build target `zsa/moonlander -km kiruna`.
  Originally exported from ZSA's Oryx configurator against ZSA's QMK fork; it's
  built here against mainline instead (see notes below), which needed a couple
  of keycode renames (legacy RGBLIGHT keycodes → `RM_*` RGB Matrix keycodes,
  `KC_MS_BTN1/2` → `MS_BTN1/2`) and dropping the `keymap.json` `"modules":
  ["zsa/defaults"]` reference (that community module only exists in ZSA's fork).
  Re-exporting from Oryx later will likely need the same fixups reapplied.

Remotes: `origin` = `git@github.com:tvdlinde/qmk_firmware.git` (Tim's fork),
`upstream` = `https://github.com/qmk/qmk_firmware`.

Branches:
- `personal-keymaps` — Tim's work branch, tracks `origin/personal-keymaps`. All
  keymap work happens here.
- `master` — mirrors `upstream/master` only, holds no local work. It's kept
  fast-forward and merged into `personal-keymaps` periodically (merge, not
  rebase).

## The `/qmk` skill

There is a project skill at `~/.claude/skills/qmk/SKILL.md` (invoke as `/qmk`)
that automates the whole upstream-sync workflow: fetch upstream → fast-forward
`master` → merge into `personal-keymaps` → sync submodules → compile the
keymaps → push to `origin` if builds pass. As of 2026-09-08 it only compiles
Preonic + Sofle; extend it to also compile the Moonlander `kiruna` keymap next
time it's touched. It stops for confirmation on
conflicts, build failures, or a dirty tree. A SessionStart hook
(`.claude/settings.local.json`) checks on session start whether
`personal-keymaps` is behind `upstream/master` and will surface that in
context — offer to run `/qmk` when it does, but don't build or push without
Tim confirming first.

## Common commands

Compile a keymap (qmk CLI, already installed):
```
qmk compile -kb preonic/rev3 -km kiruna
qmk compile -kb sofle/rev1 -km kiruna
qmk compile -kb zsa/moonlander -km kiruna
```
Or with make directly: `make preonic/rev3:kiruna` / `make sofle/rev1:kiruna` /
`make zsa/moonlander:kiruna`. A successful build ends with a `Linking: ...
[OK]` line and produces a `.bin`/`.hex` in the repo root (gitignored — don't
add these to commits).

Flash (only when Tim asks to actually flash a board — this is a hardware
action, confirm first): `qmk flash -kb preonic/rev3 -km kiruna` /
`qmk flash -kb sofle/rev1 -km kiruna` / `qmk flash -kb zsa/moonlander -km
kiruna`. **Always state the board's bootloader-entry method (e.g. reset
key/combo) before flashing**, per Tim's standing preference.

Lint/format a keyboard or keymap: `qmk lint -kb <kb>`, `qmk format-json -i <path>`.

Unit tests (C++ test framework under `tests/`, see `docs/unit_testing.md`):
```
make test:all          # every test
make test:<test_name>  # a single named test, e.g. make test:tap_hold
```
Python tooling tests: `qmk pytest` (or `nose2` — config in `nose2.cfg`).

None of Tim's `kiruna` keymaps currently has custom unit tests; the
verification loop for keymap changes is "does it compile," done via the
`qmk compile` commands above.

## Keymap architecture (`kiruna`)

All three keymaps are hand-rolled `keymap.c` + `config.h` + `rules.mk` (no
`info.json`/data-driven layout at the keymap level — matrix layout comes from
the keyboard, not the keymap). Shared conventions across Preonic and Sofle
(the two boards Tim built up together; Moonlander is a newer, separately
Oryx-exported keymap that follows its own conventions — see below):

- **Layers**: `_MAIN` (base, columnar-ish letter rearrangement, not QWERTY)
  plus momentary layers `_BLU` (symbols/numpad), `_GRN` (nav/editing),
  `_RED` (a shifted/alternate letter layer). Layer changes are shown via
  RGB underglow color in `layer_state_set_user` when `RGBLIGHT_ENABLE`.
- **Tap-hold tuning**: `PERMISSIVE_HOLD`, `CHORDAL_HOLD`, and `FLOW_TAP_TERM
  150` are set in `config.h` — during a fast typing streak, tap-hold keys
  resolve as taps (QMK's Flow Tap feature). `TAPPING_TERM` is 200ms.
- **Home-pinky mod-taps**: the GUI home-pinky keys (`LGUI_T` on
  backspace/quote) are ordinary mod-taps with a shorter per-key tapping term
  (180ms) via `get_tapping_term()`. The backspace one is special-cased in
  `process_record_user()` so its *tap* sends Ctrl+Backspace (delete word)
  while its *hold* still resolves as GUI through QMK's normal tap-hold engine.
- **Tap dances**: `TD_1`–`TD_5`, mostly for Belgian-French dead-key/accent
  input (e.g. `RALT(KC_QUOTE)`, `RALT(KC_GRAVE)` variants) via
  `ACTION_TAP_DANCE_FN_ADVANCED`, plus two simple `ACTION_TAP_DANCE_DOUBLE`.
- **Encoder**: layer-dependent behavior in `encoder_update_user()` — volume by
  default, virtual desktop switch / workspace scroll / undo-redo depending on
  which momentary layer is held.
- Sofle additionally carries RGB matrix/OLED config groundwork
  (`RGB_MATRIX_ENABLE`/`OLED_ENABLE` guards exist in `config.h` but both
  default `off` in `rules.mk`) and `MASTER_LEFT`/custom split-specific
  defines — most of that block is dormant scaffolding, not active behavior.

When editing a keymap, change the matching board's `keymap.c`/`config.h`
directly; there's no shared userspace module between the boards (no `users/`
code is used by any of them).

### Moonlander (`kiruna`) — different tap-hold setup

Unlike Preonic/Sofle, the Moonlander keymap does **not** use Flow Tap — its
`config.h` never defines `FLOW_TAP_TERM`. Instead it uses
`HOLD_ON_OTHER_KEY_PRESS_PER_KEY` (originally plain `HOLD_ON_OTHER_KEY_PRESS`)
plus `CHORDAL_HOLD` with a `chordal_hold_layout` that marks every thumb key
`'*'` (exempt from the opposite-hands rule). Layers are numbered (`0`-`3`,
via `LT(2, KC_SPACE)` etc.), not named like `_MAIN`/`_GRN`/`_BLU`/`_RED`, and
tap/hold on several keys (`DUAL_FUNC_0`-`DUAL_FUNC_5`) is handled by hand in
`process_record_user()` via `record->tap.count` instead of relying on QMK's
built-in `LT()`/`MT()` resolution.

The two thumb-row `LT(2, KC_SPACE)` / `LT(1, KC_SPACE)` keys had the
Moonlander analogue of the Preonic/Sofle Flow Tap bug: Space is the
highest-frequency key on the board, so on fast/rolling typing the next key is
often pressed before Space is released — with plain `HOLD_ON_OTHER_KEY_PRESS`
that reliably resolved as a layer hold instead of a tap, dropping the space
and reinterpreting the following keystroke on layer 1/2. Fixed
(2026-09-08) by switching to `HOLD_ON_OTHER_KEY_PRESS_PER_KEY` and adding
`get_hold_on_other_key_press()` in `keymap.c` returning `false` for just
those two keys, so they fall back to the default tap/hold decision (tapping
term / release order) instead of resolving hold the instant another key is
pressed. Same failure *symptom* as the Flow Tap bug (fast typing corrupts the
next keystroke via a spurious layer hold) but the opposite mechanism (false
hold, not false tap) — Moonlander has no Flow Tap to be swept into.

## Repo-wide notes (only relevant outside `keyboards/kiruna` work)

- `.github/copilot-instructions.md` is upstream QMK's PR-review checklist for
  Copilot (branch targeting, `info.json` schema, license headers, etc.) —
  scoped to `keyboards/**`. It governs upstream contribution PRs, not Tim's
  own keymap commits, but is useful if touching any other keyboard's files
  (e.g. bisecting an upstream regression).
- Standard QMK build system (`make`, `qmk` CLI) applies repo-wide; see
  `docs/` for full documentation (offline copy of docs.qmk.fm).
