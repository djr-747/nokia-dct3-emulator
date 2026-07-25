// eeprom_provision — external-EEPROM factory-provisioning service.
//
// A separate service (NOT part of the MAD2 platform) that the external-EEPROM load path
// (src/mad2/ext_eeprom.c) invokes once, immediately after the raw 24Cxx blob is loaded into
// m->i2c_eeprom. It applies everything a factory-calibrated phone would already carry so the
// firmware's own startup checks resolve organically:
//   - the per-revision integrity checksums the boot self-test validates (tune/calibration),
//   - the persisted DSP-fault latch a factory phone ships cleared, and
//   - FAID / factory-identity provisioning (ModelProfile.eeprom_faid): a coherent IMEI +
//     security-code + derived security state plus the two identity checksums, so the firmware's
//     identity comparison passes (no spurious "Security code" on boot) and the service-present
//     bit used by startup stays set (network registration can proceed).
//
// Every external-EEPROM model is routed through here; each opts into the pieces it needs via
// its ModelProfile fields. All of it is idempotent and EE5110_RAW=1 opts the whole service out
// (checksum-fault A/B). FAID RE + algorithm for the 3210: Gareth Davidson (bitplane),
// github.com/bitplane/nokia-dct3-re.
#ifndef DCT3_EEPROM_PROVISION_H
#define DCT3_EEPROM_PROVISION_H

struct Mad2;

// Provision m->i2c_eeprom in place. Called from ext_eeprom.c after the blob is loaded.
void eeprom_provision(struct Mad2* m);

#endif // DCT3_EEPROM_PROVISION_H
