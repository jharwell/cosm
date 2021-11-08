/**
 * \file base_nest_acq.hpp
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

#ifndef INCLUDE_COSM_SPATIAL_STRATEGY_NEST_ACQ_BASE_NEST_ACQ_HPP_
#define INCLUDE_COSM_SPATIAL_STRATEGY_NEST_ACQ_BASE_NEST_ACQ_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/base_strategy.hpp"
#include "cosm/spatial/strategy/metrics/nest_acq_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest_acq);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_nest_acq
 * \ingroup spatial strategy nest_acq
 *
 * \brief Base class for nest acquisition strategies, to make collecting metrics
 * and usage of the strategy pattern easier.
 */

class base_nest_acq : public csstrategy::base_strategy,
                      public cssmetrics::nest_acq_metrics {
 public:
  base_nest_acq(const csfsm::fsm_params* params, rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  base_nest_acq(const base_nest_acq&) = delete;
  base_nest_acq& operator=(const base_nest_acq&) = delete;
  base_nest_acq(base_nest_acq&&) = delete;
  base_nest_acq& operator=(base_nest_acq&&) = delete;

  const cssnest_acq::base_nest_acq* nest_acq_strategy(void) const override {
    return this;
  }
};

NS_END(nest_acq, strategy, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_STRATEGY_NEST_ACQ_BASE_NEST_ACQ_HPP_ */
