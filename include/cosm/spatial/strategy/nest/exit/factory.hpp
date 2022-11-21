/**
 * \file factory.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/patterns/factory/factory.hpp"

#include "cosm/cosm.hpp"
#include "cosm/spatial/strategy/nest/exit/base_exit.hpp"
#include "cosm/spatial/fsm/fsm_params.hpp"
#include "cosm/spatial/strategy/nest/config/exit_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::nest::exit {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class factory
 * \ingroup spatial strategy nest exit
 *
 * \brief Factory for creating nest exit strategies.
 */
class factory :
    public rpfactory::releasing_factory<cssnest::exit::base_exit,
                                        std::string, /* key type */
                                        const cssnest::config::exit_config*,
                                        const csfsm::fsm_params*,
                                        rmath::rng*> {
 public:
  static inline const std::string kWander = "wander";
  static inline const std::string kAntiPhototaxis = "anti_phototaxis";

  factory(void);
};

} /* namespace cosm::spatial::strategy::nest::exit */
