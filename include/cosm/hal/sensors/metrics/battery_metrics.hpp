/**
 * \file battery_metrics.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class battery_metrics
 * \ingroup hal sensors metrics
 *
 * \brief Data to gather from robot battery sensors.
 */
class battery_metrics : public rmetrics::base_metrics {
 public:
  battery_metrics(void) = default;
  ~battery_metrics(void) = default;

  /**
   * \brief Get the % battery remaining as a number [0.1].
   */
  virtual double percent_remaining(void) const = 0;
};

NS_END(metrics, sensors, hal, cosm);
