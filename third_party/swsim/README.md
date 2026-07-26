# swSIM / swICC — vendored software-SIM backend

An **all-software GSM SIM card**, vendored into this repo to give the network-camp
work a deterministic, spec-complete SIM with no real card, no serial bridge, and no
card-presence state to vary a run.

- **swICC** (`third_party/swicc/`) — ISO/IEC 7816 smartcard core: T=0 I/O FSM, APDU
  parse/dispatch, ATR/PPS, a BER-TLV filesystem (`swiccfs`) built from JSON.
- **swSIM** (`third_party/swsim/`) — the 2G SIM application on top of swICC: GSM 11.11
  EF tree, SELECT/GET RESPONSE/READ/STATUS, PIN/CHV, RUN GSM ALGORITHM (Milenage), and
  the SAT/proactive scaffolding (left disabled here).

Upstream (both by Tomasz Lisowski, both **BSD-3-Clause**, © 2024):
- swICC — https://github.com/tomasz-lisowski/swicc
- swSIM — https://github.com/tomasz-lisowski/swsim

`third_party/swicc/src/cJSON.{c,h}` is bundled by upstream swICC and is **MIT**
(© 2009-2017 Dave Gamble and cJSON contributors). See `third_party/swicc/LICENSE`.

This is a **source snapshot**, not a submodule (recovered/re-vendored from the prior
`dc7a706` working integration). It carries the local modifications below.

## Local modifications (do NOT lose these on any re-vendor)

1. **`src/apduh.c` (~line 1712) — TERMINAL PROFILE under GSM CLA.** Pre-UICC 2G phones
   (Nokia 3310) send `TERMINAL PROFILE` as `A0 10`; upstream only wired the ETSI `80 10`
   class, so `A0 10` fell through to `6D00` and the phone looped forever at SIM init.
   The patch also accepts CLA `0xA0` (the handler ignores the profile body anyway).
   Marked in-source with `LOCAL PATCH`.

2. **`src/apduh.c` — GSM READ RECORD (`A0 B2`).** Upstream swSIM's proprietary/GSM
   demux (`sim_apduh_demux`) wired READ BINARY (`A0 B0`) but never READ RECORD
   (`A0 B2`), so record-structured EFs (notably EF_ACM `6F39`, a cyclic file)
   answered `A0 B2` with `6D00` (INS not supported) and the Nokia 3310 looped
   forever in its Advice-of-Charge/ACM read. Added `apduh_gsm_rcrd_read` (mirrors
   the ISO interindustry `apduh_rcrd_read` in swICC but uses the GSM immediate-data
   `90 00` convention and supports P1=`00` "current record" absolute/current mode,
   which the firmware relies on) plus a `case 0xB2` in the GSM CLA switch. Marked
   in-source with `LOCAL PATCH`.

3. **`src/gsm.c` — GSM SELECT FCP record-length fix (upstream bug).** In
   `gsm_select_res`, the record-EF branch correctly computes the record length
   (`rcrd_length = rcrd_size`) but upstream then **unconditionally clobbered it
   to 0** (`rcrd_length = 0;`) immediately before writing FCP byte 14. So every
   linear-fixed / cyclic EF's GSM SELECT response reported **record length 0**.
   The Nokia 3310's record-EF / SIM-storage init reads that FCP to learn the
   record geometry of the phonebook (EF_ADN `6F3A`), SMS (EF_SMS `6F3C`), MSISDN
   (`6F40`), … — with record length 0 it cannot build its record cache, so the
   SIM never reached full storage readiness and storage features reported "SIM
   not ready". Fix: initialise `rcrd_length` at declaration and remove the
   clobber, so the real record length is emitted (byte 14). Confirmed at the
   FCP level (`SELECT 6F3A → …01 1E`, rec-len 0x1E=30). Marked `LOCAL PATCH`.

4. **`src/apduh.c` — GSM UPDATE RECORD (`A0 DC`).** Upstream swSIM's GSM (CLA
   `A0`) demux wired UPDATE BINARY (`A0 D6`) but never UPDATE RECORD (`A0 DC`),
   so writing a record EF — adding a contact to EF_ADN (`6F3A`) — fell through to
   `6D00` (INS not supported). Added `apduh_gsm_rcrd_update` (mirrors the record
   ADDRESSING of `apduh_gsm_rcrd_read` — P1 record number 1-based/0=current, P2
   mode 0x04 absolute/current / 0x02 next / 0x03 prev — and the T=0 data-in /
   procedure handshake of `apduh_3gpp_bin_update`), writing the incoming record
   into the swICC disk record buffer returned by `swicc_disk_file_rcrd` (a
   writable pointer into `file->data`, the same store UPDATE BINARY writes
   through), then `90 00`. Wired `case 0xDC` in the GSM CLA switch. Verified by
   round-trip: `SELECT 6F3A → UPDATE RECORD 1 (9000) → READ RECORD 1` reads back
   the written contact. PIN/CHV disabled in this fixture, so EF_ADN writes are
   permitted like UPDATE BINARY. Marked `LOCAL PATCH`.

5. **`src/fs/va.c` — SELECT-by-FID resolves current-DF-first (duplicate file-id
   collision).** `swicc_va_select_file_id` went straight to a disk-GLOBAL id LUT
   (`swicc_disk_lutid_lookup`), which ignores the selected DF. When the same
   file-id exists under several DFs — e.g. **EF_LOCI `6F7E` under DF_GSM `7F20`,
   DF_DCS1800 `7F21`, and `7F40`** — the global LUT collapses them to one entry, so
   the phone (which does `SELECT 7F20` → `SELECT 6F7E`) was served the wrong DF's
   copy (whichever the LUT kept), regardless of the selected DF. Fix: search the
   immediate children of the current DF (`fs->va.cur_df` in `fs->va.cur_tree`) by
   file-id first (GSM 11.11 / ISO 7816-4 order), falling back to the global lookup
   only when not found there (unchanged for MF / parent / cross-DF selects). Added a
   small `va_selid_curdf_cb`/`va_selid_curdf_ud_st` helper above the function.
   Verified: editing only `7F20/6F7E` while leaving `7F21`/`7F40` all-FF now serves
   `7F20`'s copy (before the fix it served the unedited copy). Marked `LOCAL PATCH`.

6. **`sim_swsim.c` — added (not upstream).** The in-process bridge between this repo's
   MAD2 SIM model and swICC. `swsim_backend_apdu()` drives swICC's T=0 io FSM directly,
   synchronously, in the same process (the 3310 firmware is the reader, swICC is the
   card, this file is the wire) — so a boot is fully deterministic. Card identity/FS
   comes from `gsm.json` (override with `SWSIM_FS=<path>`). SAT/proactive is left OFF
   (`app_default_enable = false`) so the card never pushes a proactive command.

   It also carries the **card-persistence API** the browser build uses to keep a
   visitor's SIM (contacts, messages, LOCI/Kc/FPLMN) across page loads:

   - `swsim_backend_snapshot(&buf, &len)` — `swicc_disk_save()` the LIVE mounted disk
     to the staging path, then hand the bytes back. Pure read of the FS: it never
     touches the T=0 FSM or the current selection, so it is safe at any point in a run.
   - `swsim_backend_restore(buf, len)` — stage the bytes and re-enter `swsim_init()`
     with `path_json = NULL`, which makes it **load** the `.swiccfs` instead of parsing
     `gsm.json`. Must run before the firmware's first APDU. Any failure leaves the card
     un-inited so `swsim_ensure()` rebuilds it from the JSON — a corrupt or stale
     snapshot costs the saved data, never the boot.
   - `swsim_backend_write_ef(df, fid, data, len)` — SELECT-path + UPDATE BINARY on the
     live card. The host uses it to re-apply EF_SPN (`7F20`/`6F46`) after a restore:
     the operator name is a page setting, so it must beat the snapshot's copy.
   - `swsim_backend_writes()` — successful `DC`/`D6`/`32` count, so the host only
     auto-saves when the card actually changed this run.

   The snapshot format is swICC's own FS image, **not** a hand-rolled diff: it
   round-trips through the loader `swsim_init` already uses, and its 16-byte magic
   encodes format + machine endianness, so a foreign blob is rejected by the library.
   Staging path is `SWSIM_FS_PATH` (`/tmp/dct3_swsim.swiccfs`) — MEMFS on the web, so
   the bytes only leave via the returned pointer.

7. **`gsm.json` — the card fixture.** The SIM filesystem + identity the camp targets
   (Telstra 505-01 IMSI, EFs). Consumed once at `swsim_init`.

8. **Build — native** (`Makefile`, `trace`/`gui` targets). Linked into
   `build/dct3_boot_trace` + `build/dct3_boot_trace_gui`, at `-std=gnu11` (swICC needs
   C11 `static_assert`) with `-DSWSIM_BUILD`. The socket transport `src/net.c` is
   **excluded** — the card is driven in-process, never over a socket. The unit-test /
   `make guard` builds do **not** compile these objects and never reference the
   `swsim_backend_apdu` symbol, so they stay byte-identical.

9. **Build — wasm/web** (`Makefile`, default `all`/`serve` target → `web/dct3.js` +
   `web/dct3.wasm`). swSIM is now the **web SIM** too. The swICC/swSIM TUs are compiled
   to wasm objects (`SWSIM_WASM_OBJS`) at `-std=gnu11` with `-DSWSIM_BUILD` + `SWSIM_INC`
   and linked into the module; the app TUs get `-DSWSIM_BUILD` via `CFLAGS` so the
   `mad2_sim.c` intercept is live. Two wasm-only wrinkles:
   - **`-include endian.h`** on the swICC TUs — `fs/disk.c` calls `htobe16()` without
     including `<endian.h>`; glibc pulls it in transitively but emscripten's musl libc
     does not, so it is force-included (`SWSIM_WASM_CFLAGS`).
   - **Browser filesystem.** There is no host FS in the browser, so the shim's
     `fopen("third_party/swsim/gsm.json")` and `swsim_init(..., "/tmp/dct3_swsim.swiccfs")`
     must resolve in MEMFS. `gsm.json` is **preloaded** into MEMFS at
     `/third_party/swsim/gsm.json` (the shim's default path) via `SWSIM_PRELOAD`
     (`--preload-file`; this emcc makes `--preload`/`--embed` mutually exclusive and the
     firmware already uses `--preload-file`, so both land in `web/dct3.data`). The
     swiccfs save target `/tmp/…` is emscripten-MEMFS-writable (`/tmp` exists by
     default). Verified in a node run of the wasm: `[swsim] card ready`, and a
     `SELECT 6F3A → UPDATE RECORD 1 → READ RECORD 1` round-trip reads the written bytes
     back (persistence works in the browser build).

## How it is wired in

`src/mad2/mad2_sim.c` (`sim_compute_apdu`) routes the assembled T=0 command through
`swsim_backend_apdu()` instead of the hand-rolled synthetic EF table, under
`#ifdef SWSIM_BUILD`. Selection at runtime:

- **default (native):** ON iff the faithful **ROM6NEW** DSP engine is active (the
  real-network camp target). OFF otherwise — so the legacy-DSP guard boots stay
  byte-identical.
- **default (web, `#ifdef __EMSCRIPTEN__`):** ON — swSIM is the browser's persistent,
  spec-complete SIM (UPDATE RECORD sticks, unlike the synthetic table's no-op). The wasm
  runs the HLE DSP, not ROM6NEW, so the native default never fires there.
- `SWSIM=0` — force the synthetic EF table (A/B against swSIM). On the web, settable via
  `Module.ENV` before boot (getenv is honoured under emscripten when the JS host sets it).
- `SWSIM=1` — force swSIM anywhere (e.g. a legacy boot).

## Fixed: EF_ACM READ RECORD boot loop (was the reason swSIM stayed opt-in)

swSIM previously stalled the 3310 boot in an **infinite `READ RECORD` loop on EF_ACM
(`6F39`)** — `A0 B2 00 04 03` repeated ~1050× and the MMI never drew standby. Root cause:
upstream swSIM's GSM (CLA `A0`) demux wired READ BINARY (`A0 B0`) but **never READ RECORD
(`A0 B2`)**, so every record read fell through to `6D00` (INS not supported); the
firmware's Advice-of-Charge/ACM path retried forever. (Confirmed by response-level trace:
`A0 B2 00 04 03 -> SW 6D00`, whereas the synthetic table returns `00 00 00 / 90 00`.)
Fixed by local modification (2) above — the GSM READ RECORD handler. With it, `A0 B2 00 04
03 -> 00 00 00 / 90 00`, the B2 count drops from ~1050 to ~5, the netmonitor/standby draws
(LCD dark% ~14.5, was ~2.3), and camp-status still reaches `0x50`. swSIM remains opt-in
only because the synthetic table is what the camp work was tuned against, not because of
this loop.

## Re-vendoring / updating

Re-copy `include/` + `src/` from each upstream, then **re-apply the code patches:
(1) TERMINAL PROFILE + (2) GSM READ RECORD + (4) GSM UPDATE RECORD in `apduh.c`,
(3) the `gsm.c` GSM SELECT FCP record-length fix, and (5) the `fs/va.c`
current-DF-first SELECT-by-FID fix** — and keep the added files (6)
`sim_swsim.c` + (7) `gsm.json`. Confirm with:

```
make guard                                  # byte-identical legacy boots + unit suites
ROM6NEW=1 RAMWATCH=0x110F96 ./build/dct3_boot_trace \
    "firmware/Factory Reset 3310 NR1 v5.79.fls" 46000000   # [swsim] card ready + camp 0x50
```
