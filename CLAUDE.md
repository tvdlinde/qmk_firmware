# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

This is Tim's fork of QMK Firmware. The vast majority of the tree is upstream
QMK (thousands of community keyboards); the part Tim actually maintains is his
two personal keymaps, both named **`kiruna`**:

- `keyboards/preonic/keymaps/kiruna/` — build target `preonic/rev3 -km kiruna`
- `keyboards/sofle/keymaps/kiruna/` — build target `sofle/rev1 -km kiruna`

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
`master` → merge into `personal-keymaps` → sync submodules → compile both
keymaps → push to `origin` if builds pass. It stops for confirmation on
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
```
Or with make directly: `make preonic/rev3:kiruna` / `make sofle/rev1:kiruna`.
A successful build ends with a `Linking: ... [OK]` line and produces a
`.bin`/`.hex` in the repo root (gitignored — don't add these to commits).

Flash (only when Tim asks to actually flash a board — this is a hardware
action, confirm first): `qmk flash -kb preonic/rev3 -km kiruna` /
`qmk flash -kb sofle/rev1 -km kiruna`. **Always state the board's
bootloader-entry method (e.g. reset key/combo) before flashing**, per Tim's
standing preference.

Lint/format a keyboard or keymap: `qmk lint -kb <kb>`, `qmk format-json -i <path>`.

Unit tests (C++ test framework under `tests/`, see `docs/unit_testing.md`):
```
make test:all          # every test
make test:<test_name>  # a single named test, e.g. make test:tap_hold
```
Python tooling tests: `qmk pytest` (or `nose2` — config in `nose2.cfg`).

Neither of Tim's `kiruna` keymaps currently has custom unit tests; the
verification loop for keymap changes is "does it compile," done via the two
`qmk compile` commands above.

## Keymap architecture (`kiruna`)

Both keymaps are hand-rolled `keymap.c` + `config.h` + `rules.mk` (no
`info.json`/data-driven layout at the keymap level — matrix layout comes from
the keyboard, not the keymap). Shared conventions across both:

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
directly; there's no shared userspace module between the two boards (no
`users/` code is used by either).

## Repo-wide notes (only relevant outside `keyboards/kiruna` work)

- `.github/copilot-instructions.md` is upstream QMK's PR-review checklist for
  Copilot (branch targeting, `info.json` schema, license headers, etc.) —
  scoped to `keyboards/**`. It governs upstream contribution PRs, not Tim's
  own keymap commits, but is useful if touching any other keyboard's files
  (e.g. bisecting an upstream regression).
- Standard QMK build system (`make`, `qmk` CLI) applies repo-wide; see
  `docs/` for full documentation (offline copy of docs.qmk.fm).
