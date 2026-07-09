# dbgcon — debug console for DCT3 firmware patches

Print from your injected firmware code onto the host console while it runs **under the DCT3
emulator**. On real hardware it is inert, so the same patch is safe to flash to a phone.

## What to hand a patch developer

Just **one file**: [`dbgcon.h`](dbgcon.h). It is freestanding C — no libc, no system
headers, no build system. Drop it next to your patch source and `#include "dbgcon.h"`.

They also need **an emulator build that has the port** — any current build of this repo
(`build/dct3_boot_trace`, the SDL GUI, or the web build). The port is always on; nothing to
enable. On real hardware the address is undecoded, so nothing happens there.

That's it. The other files here are host-side tests you do NOT ship:
`dbgcon_selftest.c` / `dbgcon_stripcheck.c` (run with `make dbgcon-selftest`).

## Use it

```c
#include "dbgcon.h"

void my_patch(unsigned regval) {
    dbgcon_puts("patch hit"); dbgcon_nl();        // [dbgcon] patch hit
    dbgcon_kv("regval=", regval);                 // [dbgcon] regval=0x0000CAFE
    dbgcon_hexport(regval);                        // [dbgcon] pc=<pc> val=... (dec)
}
```

Full API + the port map + a hand-assembled (no-compiler) stub are in
[`../../docs/dbgcon.md`](../../docs/dbgcon.md).

## Getting your code to run

`dbgcon.h` gives you the *API*; you still need your code executing inside the firmware. Two
routes, both covered in:

1. **C overlay** — compile your patch (incl. `dbgcon.h`) with the NokiX SDK ARM toolchain
   (e.g. the NokiX SDK) and inject it. Cleanest.
2. **Hand-assembled stub** — no compiler: graft a few instructions with the NokiX runner
   (`tools/nokix/`: `CREATE` the stub into free flash, redirect a call with `FINDBL`/`SETBL`).

## Production flash

For a shipping / real-hardware image, strip everything at compile time:

```
cc ... -DDBGCON_ENABLE=0
```

Every `dbgcon_*()` call — and its string literals — vanish from the image (zero bytes, zero
cycles). Verify: `strings <out> | grep <your debug text>` → no match. (Caveat: a stripped
call does not evaluate its arguments, so don't put a load-bearing side effect inside one.)
Left compiled in (the default), the calls self-silence on real hardware at run time anyway.
