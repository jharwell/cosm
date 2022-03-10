/**
 * \file location_metrics.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/math/vector2.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, metrics, caches);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class location_metrics
 * \ingroup arena metrics caches
 *
 * \brief Defines the metrics to be collected from a \ref arena_cache regarding
 * its location in the arena.
 *
 * Metrics are collected every timestep.
 */
class RCPPSW_EXPORT location_metrics : public virtual rmetrics::base_metrics {
 public:
  location_metrics(void) = default;

  /**
   * \brief Should return the discrete location of the cache.
   */
  virtual rcppsw::math::vector2z location(void) const = 0;
};

NS_END(caches, metrics, arena, cosm);
