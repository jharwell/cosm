/**
 * \file base_acq.hpp
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
#include "cosm/spatial/strategy/base_strategy.hpp"
#include "cosm/spatial/strategy/nest/metrics/acq_metrics.hpp"
#include "cosm/spatial/strategy/nest/config/acq_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest, acq);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_acq
 * \ingroup spatial strategy nest acq
 *
 * \brief Base class for nest acquisition strategies, to make collecting metrics
 * and usage of the strategy pattern easier.
 */

class base_acq : public csstrategy::base_strategy,
                 public rpprototype::clonable<base_acq>,
                 public cssnest::metrics::acq_metrics {
 public:
  base_acq(const cssnest::config::acq_config* config,
           const csfsm::fsm_params* params,
           rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  base_acq(const base_acq&) = delete;
  base_acq& operator=(const base_acq&) = delete;
  base_acq(base_acq&&) = delete;
  base_acq& operator=(base_acq&&) = delete;

  const cssnest::acq::base_acq* nest_acq_strategy(void) const override {
    return this;
  }

 protected:
  const cssnest::config::acq_config* config(void) const {
    return &mc_config;
  }

 private:
  /* clang-format off */
  const cssnest::config::acq_config mc_config;
  /* clang-format on */
};

NS_END(acq, nest, strategy, spatial, cosm);
