/**
 * \file base_exit.hpp
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
#include "cosm/spatial/strategy/nest/config/exit_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::nest::exit {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_exit
 * \ingroup spatial strategy nest exit
 *
 * \brief Base class for nest exit strategies, to make usage of the strategy
 * pattern easier.
 */

class base_exit : public csstrategy::base_strategy,
                  public rpprototype::clonable<base_exit> {
 public:
  base_exit(const cssnest::config::exit_config* config,
                const csfsm::fsm_params* params,
                rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  base_exit(const base_exit&) = delete;
  base_exit& operator=(const base_exit&) = delete;
  base_exit(base_exit&&) = delete;
  base_exit& operator=(base_exit&&) = delete;

 protected:
  const cssnest::config::exit_config* config(void) const {
    return &mc_config;
  }

 private:
  /* clang-format off */
  const cssnest::config::exit_config mc_config;
  /* clang-format on */
};

} /* namespace cosm::spatial::strategy::nest::exit */
