// identity_provision — in-emulator EEPROM re-provisioning ("reset") service.
//
// Writes a complete, self-consistent factory identity into the phone's EEPROM: the FLASH-ID
// (FAID) record, both SIMlock parts and the IMEI record, all derived from the model's pinned
// MSID (ModelProfile.identity) via the Calcul codec in services/dct3_calcul.c.
//
// HOW it writes is the point. It does NOT poke EEPROM offsets — we do not know where each
// build keeps its records, and guessing would be exactly the kind of shim this project
// refuses. Instead it acts as a VIRTUAL SERVICE TOOL on the phone's own MBUS: it builds the
// real B8 / BA / B6 (selector 0x4F) frames a Windows service tool sends, feeds them into the
// MBUS RX FIFO through the same mbus_rx_push path the host serial bridge uses, and lets the
// FIRMWARE's own service handlers decode them and write their own EEPROM. The records
// therefore land wherever that build actually keeps them, and they land through the code path
// a real re-provisioning would use. The frames are byte-identical in form to the captured
// NokTool traffic in re/mbus-captures/ (decoder: re/mbus-captures/mbusdec.py).
//
// Because the writes go through the firmware, the result is an ordinary EEPROM change: the
// existing NVRAM persistence (web localStorage / IndexedDB, keyed per firmware) saves and
// exports it like any other, and it survives a reload. Nothing here is a boot-time patch.
//
// Preconditions: the phone must be booted far enough to be running its MBUS service handlers
// (i.e. past startup, in normal mode). A run that gets no reply times out and reports it
// rather than half-writing.
#ifndef DCT3_IDENTITY_PROVISION_H
#define DCT3_IDENTITY_PROVISION_H

#include <stdint.h>

struct Mad2;

enum {
    IDPROV_IDLE    = 0,
    IDPROV_RUNNING = 1,
    IDPROV_DONE    = 2,
    IDPROV_FAILED  = 3,
};

// Read the identity back WITHOUT writing: runs the session prologue, then service command
// 0x66. Use it to ask an image "do you still carry a provisioned identity?" — e.g. after
// merging the EEPROM back into a .fls. Same status/state reporting as a write run.
int identity_provision_verify(struct Mad2* m);

// Shared implementation; verify_only != 0 skips every write.
int identity_provision_run(struct Mad2* m, int verify_only);

// Begin a re-provisioning run. Returns 1 if started, 0 if the model carries no pinned
// identity or a run is already in flight (see identity_provision_status for why).
int identity_provision_start(struct Mad2* m);

// Per-step pump. Called from the shared platform tick (mad2_timers_tick); a single flag
// test when idle, so it costs nothing on every boot that never asks for it.
void identity_provision_tick(struct Mad2* m);

// IDPROV_* above.
int identity_provision_state(void);

// Human-readable progress/result line ("writing IMEI record", "timed out waiting for B9",
// "wrote FAID + SIMlock + IMEI"). Never NULL.
const char* identity_provision_status(void);

// Records written so far this run (0..3), for a progress indicator.
int identity_provision_progress(void);

#endif // DCT3_IDENTITY_PROVISION_H
