#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <scip/clock.h>
#include <scip/scip.h>
#include <scip/struct_scip.h>
#include <scip/struct_set.h>
#include <scip/struct_stat.h>
#pragma GCC diagnostic pop

inline SCIP_Real get_time_remaining(SCIP* scip)
{
    return scip->set->limit_time - SCIPgetSolvingTime(scip);
}

// Float get_time_remaining(SCIP* scip)
// {
//     SCIP_Real time_limit;
//     scip_assert(SCIPgetRealParam(scip, "limits/time", &time_limit));
//     const auto current_time = SCIPgetSolvingTime(scip);
//     const auto time_remaining = time_limit - current_time;
//     return time_remaining;
// }
