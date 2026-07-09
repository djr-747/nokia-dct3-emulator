/* Spike stub for calypso_full_pcb.h — replaces QEMU's PCB/lock surface with
 * no-ops so calypso_c54x.c builds standalone for the DCT3 DSP spike. */
#ifndef HW_ARM_CALYPSO_FULL_PCB_H
#define HW_ARM_CALYPSO_FULL_PCB_H

typedef int QemuMutex;
static inline void qemu_mutex_lock(QemuMutex *m)   { (void)m; }
static inline void qemu_mutex_unlock(QemuMutex *m) { (void)m; }

extern QemuMutex calypso_pcb_daram_lock;
extern QemuMutex calypso_pcb_api_ram_lock;

#endif
