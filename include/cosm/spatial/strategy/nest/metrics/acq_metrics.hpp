/**
 * \file acq_metrics.hpp
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
namespace cosm::spatial::strategy::nest::acq {
class base_acq;
} /* namespace cosm::spatial::strategy::acq */

NS_START(cosm, spatial, strategy, nest, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class acq_metrics
 * \ingroup spatial strategy nest metrics
 *
 * \brief Metrics to be collected from \ref base_acq and derived classes.
 */
class acq_metrics : virtual rmetrics::base_metrics {
 public:
  acq_metrics(void) = default;
  ~acq_metrics(void) override = default;

  /* Not move/copy constructable/assignable by default */
  acq_metrics(const acq_metrics&) = delete;
  acq_metrics& operator=(const acq_metrics&) = delete;
  acq_metrics(acq_metrics&&) = delete;
  acq_metrics& operator=(acq_metrics&&) = delete;

  /**
   * \brief Return the current nest acquisition strategy.
   */
  virtual const cssnest::acq::base_acq* nest_acq_strategy(void) const = 0;
};

NS_END(metrics, nest, strategy, spatial, cosm);
