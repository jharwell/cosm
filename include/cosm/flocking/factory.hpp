/**
 * \file factory.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/patterns/factory/factory.hpp"

#include "cosm/flocking/base_flocking.hpp"
#include "cosm/spatial/fsm/fsm_params.hpp"
#include "cosm/flocking/config/flocking_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::flocking {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class factory
 * \ingroup flocking
 *
 * \brief Factory for creating flocking strategies.
 */
class factory :
    public rpfactory::releasing_factory<cflocking::base_flocking,
                                        std::string, /* key type */
                                        const cflocking::config::flocking_config*,
                                        const csfsm::fsm_params*,
                                        rmath::rng*> {
 public:
  static inline const std::string kStochFOV = "stoch_fov";

  factory(void);
};

} /* namespace cosm::flocking */
