# nokix — Linux-native runner for NokiX .nrx scripts

Replaces NokiX-on-wine.  A small (~1000 LOC) C program that links libregina
and provides NokiX's host-command surface (`getbyte`, `getword`, `getlong`,
`getdata`, `setbyte`, `setword`, `setlong`, `setdata`, `checkbounds`, `find`,
`findfunc`, `findbl`, `getbl`, `setbl`, `findb`, `getb`, `setb`, `findldr`,
`getldr`, `create`, `runatend`, `fail`) plus REXX functions (`getenv`,
`setenv`, `swapenv`, `reserve_ram`, `dand`, `dor`, `dxor`, `dshr`, `dshl`).

Regina autoloads NokiX's REXX macros (`LOCATE.rx`, `CREATE.rx`-style files in
`ref/NokiX-scripts/.../macros/`) — they implement `locate(...)`, `add_5e0_id`,
`add_action`, etc.  We only provide the C-side primitives those macros call.

## Build + run

```
make                       # produces ./nokix, links -lregina
M=../../ref/NokiX-scripts/NokiX/scripts/scripts/macros
REGINA_MACROS=$M ./nokix [-v] -m 3310 firmware.fls path/to/script.nrx [args...]
```

The patched flash is written to `<input>.patched.fls` (override with
`-o OUT.fls`).  Partial saves happen even on `fail` so you can inspect.

## What works today

- Read-only dumpers (`dump_text.nrx`, etc.) — small surface, proven end-to-end.
- Patch bodies through the **create + setbyte/setword/setbl/setlong** stage —
  `anonymous_access.nrx` writes 48 bytes across the SIM hook, action bytes,
  and ui_processor table.  Hook code lands in the auto-detected free-flash
  pool (240 KB of 0xFF at 0x393EC0 on 3310 v5.79).
- BL/B/LDR Thumb instruction codecs are real implementations
  (`thumb_bl_decode/encode`, `thumb_b_decode/encode`, `thumb_ldr_pc_decode`).

## What's incomplete

- **NokiX's REXX-engine quirks**.  NokiX shipped a forked Regina that treats
  bare function-call statements as "set rc = return value" rather than
  sending the value to ADDRESS.  We shim it for pure numeric values, but
  more elaborate cases may misfire.
- **C-side-only locators**.  Some objects (e.g. `ui_task`) have no
  `LOCATE_*.rx` definition — NokiX's runtime resolves them in C.  We
  pre-cache the known ones for 3310 v5.79 (see `func/UI_TASK` etc. in
  `main()`).  Other firmwares need their own seed values.
- **PPM string injection** (`add_ppm_strings`) — fails inside ADD_PPM_STRINGS.rx
  line 333 with an arithmetic-on-empty error.  Suggests a missing primitive
  in the PPM management path; haven't traced it yet.
- **`link`** is a script-local label, not a primitive — should "just work"
  once the rest does.
- Only one model seeded (3310).  Adding 8850/7110 etc. needs their task-map
  addresses + firmware version strings hardcoded in `main()`.

## Architecture notes

- All flash access is big-endian word-invariant (`dct3-big-endian` memory) —
  byte at file offset A is the MSB of the 32-bit word at CPU addr (base + A).
  Flash base defaults to 0x200000 (DCT3), override with `-b`.
- `create` is a simple bump allocator over the longest 0xFF run in the
  loaded flash (auto-detected at startup).  NokiX presumably has smarter
  free-space tracking with multiple section pools; we don't.
- `runatend` queues `(script, label, arg)` triples that are executed after
  the main script returns, before the save.  Used by `ADD_5E0_ID.rx :THE_END`
  to write the accumulated dispatcher table back to flash.
- `find ... "patt" "mask"` passes patt/mask as variable NAMES — NokiX fetches
  their binary values via `RexxVariablePool` (RXSHV_FETCH).  Same for
  `setdata`.

