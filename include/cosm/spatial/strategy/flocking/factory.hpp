/**
 * \file factory.hpp
 *
 * \copyright 2022 SIFT LLC, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/patterns/factory/factory.hpp"

#include "cosm/spatial/strategy/flocking/base_flocking.hpp"
#include "cosm/spatial/fsm/fsm_params.hpp"
#include "cosm/spatial/strategy/flocking/config/flocking_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::flocking {


/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class factory
 * \ingroup spatial strategy flocking
 *
 * \brief Factory for creating flocking strategies.
 */
class factory :
    public rpfactory::releasing_factory<cssflocking::base_flocking,
                                        std::string, /* key type */
                                        const cssflocking::config::flocking_config*,
                                        const csfsm::fsm_params*,
                                        rmath::rng*> {
 public:
  static inline const std::string kStochFOV = "stoch_fov";

  factory(void);
};

} /* namespace cosm::spatial::strategy::flocking */
