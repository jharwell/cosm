/**
 * \file nest_acq_metrics.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
namespace cosm::spatial::strategy::nest_acq {
class base_nest_acq;
} /* namespace cosm::spatial::strategy::nest_acq */

NS_START(cosm, spatial, strategy, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class nest_acq_metrics
 * \ingroup spatial strategy metrics
 *
 * \brief Metrics to be collected from \base_nest_acq and derived classes.
 */
class nest_acq_metrics : virtual rmetrics::base_metrics {
 public:
  nest_acq_metrics(void) = default;
  ~nest_acq_metrics(void) override = default;

  /* Not move/copy constructable/assignable by default */
  nest_acq_metrics(const nest_acq_metrics&) = delete;
  nest_acq_metrics& operator=(const nest_acq_metrics&) = delete;
  nest_acq_metrics(nest_acq_metrics&&) = delete;
  nest_acq_metrics& operator=(nest_acq_metrics&&) = delete;

  /**
   * \brief Return the current nest acquisition strategy.
   */
  virtual const cssnest_acq::base_nest_acq* nest_acq_strategy(void) const = 0;
};

NS_END(metrics, strategy, spatial, cosm);

