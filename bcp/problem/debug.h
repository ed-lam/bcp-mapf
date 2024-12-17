/*
This file is part of BCP-MAPF.

BCP-MAPF is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

BCP-MAPF is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with BCP-MAPF.  If not, see <https://www.gnu.org/licenses/>.

Author: Edward Lam <ed@ed-lam.com>
*/

#ifndef MAPF_DEBUG_H
#define MAPF_DEBUG_H

#include "trufflehog/Debug.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "scip/scip.h"
#pragma GCC diagnostic pop

#define scip_assert(statement, ...) do { \
    const auto error = statement; \
    release_assert(error == SCIP_OKAY, "SCIP error {}", static_cast<int>(error)); \
} while (false)

#define not_yet_implemented() do { \
    err("Reached unimplemented section in function \"{}\"", __PRETTY_FUNCTION__); \
} while (false)

#define not_yet_checked() do { \
    err("Reached unchecked section in function \"{}\"", __PRETTY_FUNCTION__); \
} while (false)

#define unreachable() do { \
    err("Reached unreachable section in function \"{}\"", __PRETTY_FUNCTION__); \
} while (false)

#endif
