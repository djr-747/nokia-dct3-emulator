/* Copyright (c) 2013-2014 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef MACROS_H
#define MACROS_H

#include <mgba-util/common.h>

// LOCAL CHANGE (nokia-dct3-emu): the DCT3/MAD2 ARM7TDMI runs BIG-ENDIAN, so the
// CPU's instruction fetch (which reads activeRegion through these macros, bypassing
// the memory vtable) must be big-endian. Upstream mGBA targets little-endian ARM.
// On a little-endian host the *BE variants byte-swap; the flash image is kept
// byte-for-byte as dumped. See third_party/mgba-arm/README.md.
#define LOAD_64 LOAD_64BE
#define LOAD_32 LOAD_32BE
#define LOAD_16 LOAD_16BE
#define STORE_64 STORE_64BE
#define STORE_32 STORE_32BE
#define STORE_16 STORE_16BE

#endif
