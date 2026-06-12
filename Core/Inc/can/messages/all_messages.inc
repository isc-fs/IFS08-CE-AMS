/* SPDX-License-Identifier: proprietary */
/*
 * The message registry. can_codecs.hpp #includes THIS file once per
 * expansion pass (struct / encode / decode / descriptor), so adding a
 * message to the DSL is a single line here plus its .def. Each .def is
 * expanded under whatever CAN_MSG / FIELD_* macros the including pass
 * has defined.
 *
 * Phase 2a scope: the three AMS telemetry frames (0x4A0/4A1/4A2).
 */

#include "ams_status.def"
#include "ams_pack.def"
#include "ams_temps.def"
