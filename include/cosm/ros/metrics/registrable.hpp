/**
 * \file registrable.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/creatable_collector_set.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ros, metrics, registrable);

extern rmetrics::creatable_collector_set kStandard;

extern rmetrics::creatable_collector_set kWithNBlockClusters;

NS_END(registrable, metrics, ros, cosm);
