// SPDX-License-Identifier: proprietary
//
// Emits the `bl_fwinfo_t` record the isc-fs/stm32-can-bootloader requires
// at a fixed flash offset to consider the application valid. Without this
// record, the bootloader rejects JUMP with `NO_VALID_APP` even though
// FLASH_WRITE + FLASH_VERIFY have succeeded.
//
// Layout is taken verbatim from
// isc-fs/stm32-can-bootloader/Core/Inc/bl_fwinfo.h. The record is pinned
// at `BL_APP_BASE + 0x400` (= 0x08020400) by the `.firmware_info` section
// declared in STM32H733XG_FLASH.ld.
//
// All version fields here are placeholders for the bench bring-up; a
// follow-up commit should hook these into the build system so they get
// populated from CMake (semver / git hash / timestamp) automatically.

#include "ams_config.hpp"

#include <cstdint>

extern "C" {

typedef struct {
    uint32_t magic;
    uint32_t record_version;
    uint32_t fw_version_major;
    uint32_t fw_version_minor;
    uint32_t fw_version_patch;
    uint32_t mcu_id;
    uint8_t  git_hash[8];
    uint64_t build_timestamp;
    char     product_name[16];
    uint32_t reserved[2];
} bl_fwinfo_t;

const bl_fwinfo_t __firmware_info
    __attribute__((section(".firmware_info"), used)) = {
    // BL_FWINFO_MAGIC -- contract source of truth is
     //   isc-fs/stm32-can-bootloader/Core/Inc/bl_fwinfo.h
     // If the BL bumps this constant we MUST follow in lockstep, otherwise
     // the loader rejects JUMP with NO_VALID_APP and the unit silently
     // refuses to boot after a successful flash. Keep the literal hex in
     // sync verbatim; we can't #include the BL header from here because
     // the BL repo is not a submodule of this firmware tree (see #118 for
     // the long-term fix that pulls the struct + magic from CMake).
     .magic            = 0xF14F1B00U,
    .record_version   = 0x00010000U,        // record schema 1.0
    .fw_version_major = 0,
    .fw_version_minor = 1,
    .fw_version_patch = 0,
    .mcu_id           = 0x00000450U,        // STM32H7x3
    .git_hash         = {0,0,0,0,0,0,0,0},
    .build_timestamp  = 0,
    .product_name     = "IFS08-CE-AMS",
    // reserved[0]: AMS node ID on the stm32-can-bootloader multi-node
    // bus. Pit-tool reads this at flash time and refuses to flash if
    // it doesn't match the BL it just discovered (see AmsNodeId
    // commentary in ams_config.hpp). reserved[1] still free.
    .reserved         = { ams::config::AmsNodeId, 0 },
};

// Pit-diag (#247) accessors. Lets AcuCanTask surface firmware identity
// on the wire without re-declaring the bl_fwinfo_t shape in a header
// (avoids a maintenance hazard if the BL contract evolves -- only this
// TU needs to track it). Each getter returns a single self-contained
// scalar; the pit_diag encoder packs them into one frame.
uint8_t ams_fw_version_major(void) {
    return (uint8_t)__firmware_info.fw_version_major;
}
uint8_t ams_fw_version_minor(void) {
    return (uint8_t)__firmware_info.fw_version_minor;
}
uint8_t ams_fw_version_patch(void) {
    return (uint8_t)__firmware_info.fw_version_patch;
}
uint8_t ams_bl_node_id(void) {
    return (uint8_t)__firmware_info.reserved[0];
}
const uint8_t* ams_git_hash(void) {
    return __firmware_info.git_hash;
}

}  // extern "C"
