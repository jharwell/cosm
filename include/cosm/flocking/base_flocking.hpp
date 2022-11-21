/**
 * \file base_flocking.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/base_strategy.hpp"
#include "cosm/flocking/config/flocking_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::flocking {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_flocking
 * \ingroup flocking
 *
 * \brief Base class for flocking strategies, to make collecting metrics
 *        and usage of the strategy pattern easier.
 */

class base_flocking : public csstrategy::base_strategy,
                      public rpprototype::clonable<base_flocking> {
 public:
  base_flocking(const cflocking::config::flocking_config* config,
                const csfsm::fsm_params* params,
                rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  base_flocking(const base_flocking&) = delete;
  base_flocking& operator=(const base_flocking&) = delete;
  base_flocking(base_flocking&&) = delete;
  base_flocking& operator=(base_flocking&&) = delete;

 protected:
  const cflocking::config::flocking_config* config(void) const {
    return &mc_config;
  }

 private:
  /* clang-format off */
  const cflocking::config::flocking_config mc_config;
  /* clang-format on */
};

} /* namespace cosm::flocking */
