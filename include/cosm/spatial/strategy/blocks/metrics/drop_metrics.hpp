/**
 * \file drop_metrics.hpp
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
namespace cosm::spatial::strategy::blocks::drop {
class base_drop;
} /* namespace cosm::spatial::strategy::drop */

NS_START(cosm, spatial, strategy, blocks, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class drop_metrics
 * \ingroup spatial strategy blocks metrics
 *
 * \brief Metrics to be collected from \ref base_drop and derived classes.
 */
class drop_metrics : virtual rmetrics::base_metrics {
 public:
  drop_metrics(void) = default;
  ~drop_metrics(void) override = default;

  /* Not move/copy constructable/assignable by default */
  drop_metrics(const drop_metrics&) = delete;
  drop_metrics& operator=(const drop_metrics&) = delete;
  drop_metrics(drop_metrics&&) = delete;
  drop_metrics& operator=(drop_metrics&&) = delete;

  /**
   * \brief Return the current block drop strategy.
   */
  virtual const cssblocks::drop::base_drop* block_drop_strategy(void) const = 0;
};

NS_END(metrics, blocks, strategy, spatial, cosm);
