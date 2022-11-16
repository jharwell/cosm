/**
 * \file base_flocking.hpp
 *
 * \copyright 2022 SIFT LLC, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/base_strategy.hpp"
#include "cosm/spatial/strategy/flocking/config/flocking_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::flocking {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_flocking
 * \ingroup spatial strategy flocking
 *
 * \brief Base class for flocking strategies, to make collecting metrics
 *        and usage of the strategy pattern easier.
 */

class base_flocking : public csstrategy::base_strategy,
                      public rpprototype::clonable<base_flocking> {
 public:
  base_flocking(const cssflocking::config::flocking_config* config,
                const csfsm::fsm_params* params,
                rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  base_flocking(const base_flocking&) = delete;
  base_flocking& operator=(const base_flocking&) = delete;
  base_flocking(base_flocking&&) = delete;
  base_flocking& operator=(base_flocking&&) = delete;

  const cssflock::acq::base_flocking* flocking_strategy(void) const override {
    return this;
  }

 protected:
  const cssflocking::config::flocking_config* config(void) const {
    return &mc_config;
  }

 private:
  /* clang-format off */
  const cssflocking::config::flocking_config mc_config;
  /* clang-format on */
};

} /* namespace cosm::spatial::strategy::flocking */
