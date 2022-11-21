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
#include "cosm/spatial/strategy/nest/acq/base_acq.hpp"
#include "cosm/spatial/fsm/fsm_params.hpp"
#include "cosm/spatial/strategy/nest/config/acq_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::nest::acq {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class factory
 * \ingroup spatial strategy nest acq
 *
 * \brief Factory for creating nest acquisition strategies.
 */
class factory :
    public rpfactory::releasing_factory<cssnest::acq::base_acq,
                                        std::string, /* key type */
                                        const cssnest::config::acq_config*,
                                        const csfsm::fsm_params*,
                                        rmath::rng*> {
 public:
  static inline const std::string kWander = "wander";
  static inline const std::string kRandomThresh = "random_thresh";
  static inline const std::string kWanderRandomThresh = "wander_random_thresh";

  factory(void);
};

} /* namespace cosm::spatial::strategy::nest::acq */
