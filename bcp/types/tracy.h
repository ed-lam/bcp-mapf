#pragma once

#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#else
#define ZoneScoped(...)
#define ZoneScopedC(...)
#define ZoneScopedNC(...)
#define ZoneValue(...)
#endif
