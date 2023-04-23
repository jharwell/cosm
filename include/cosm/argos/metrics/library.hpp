/**
 * \file library.hpp
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
namespace cosm::argos::metrics {

/**
 * \class library
 * \ingroup argos metrics
 *
 * \brief Contains all the ARGOS-specific collectors that can be registered.
 *
 * \note Collector sets MUST be member variables to avoid static global variable
 * initialization ordering issue, because they depend on \ref cmspecs, which are
 * also global variables. See COSM#189.
 */
class library {
 public:
  const rmetrics::creatable_collector_set kStandard;
  const rmetrics::creatable_collector_set kWithArenaDims2D;
  const rmetrics::creatable_collector_set kWithArenaDims3D;

  const rmetrics::creatable_collector_set kWithNBlockClusters;
  const rmetrics::creatable_collector_set kKinematics;

  library(void);
};

} /* namespace cosm::argos::metrics */
