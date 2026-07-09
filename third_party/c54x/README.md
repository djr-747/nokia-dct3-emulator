# third_party/c54x — TMS320C54x DSP core (GPL-2.0-or-later)

## Provenance

`calypso_c54x.{c,h}` is vendored from **github.com/bbaranoff/qemu**
(`hw/arm/calypso/calypso_c54x.{c,h}`), license **GPL-2.0-or-later** (SPDX
headers in the sources). It has been substantially extended in this repo
(decode fixes, PC histogram/halt/snapshot instrumentation, OVLY/warm-boot
modelling — see `docs/5110-DSP-CANONICAL-STATE.md` and the c54x memory docs);
those extensions are contributions under the same GPL license. `inc/` carries
the 3-symbol stub surface that replaces the QEMU PCB/debug environment.

The DCT3 glue in this directory (`dct3_dsp54.{c,h}` HPI bridge,
`mad2_dsp_c54x.c` DspOps backend, `stubs.c`) was written for this project, but
lives HERE deliberately so the entire GPL-linked surface is one directory.

## The licensing boundary (policy, 2026-06-13)

**Everything in this directory is native-only developer/analysis tooling. It
is NEVER linked into the shipped product.**

- The web/wasm build (`make` → `web/dct3.wasm`) excludes this directory at the
  Makefile level — `APP_SRCS` globs `src/` only. The wasm DSP backend is
  always the project-original HLE responder (`src/mad2/dsp_default.c`).
- Native dev binaries (`boot_trace`, `gui`, `test_mad2`, `dsp_replay`) DO link
  the core; as combined works they are GPL-2.0-or-later. They are analysis
  tools, not distributed product.
- `src/` must not `#include` anything from this directory. The single seam is
  the link-time symbol `extern const DspOps mad2_dsp_c54x` declared in
  `src/models/model.h` and referenced (native-only, behind
  `#ifndef __EMSCRIPTEN__`) by `src/models/5110/profile.c`. Keep it that way:
  new cosim features grow inside this directory behind the DspOps vtable, not
  as includes in `src/`.

Standalone runners (PC-histogram spike, integration smoke test, cipher probe)
live in `spike/dsp54/` and link the core from here.
