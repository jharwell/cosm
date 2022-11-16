/**
 * \file base_acq.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
