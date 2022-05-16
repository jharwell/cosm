/**
 * \file factory.hpp
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
#include <string>

#include "rcppsw/patterns/factory/factory.hpp"

#include "cosm/cosm.hpp"
#include "cosm/spatial/strategy/nest/acq/base_acq.hpp"
#include "cosm/spatial/fsm/fsm_params.hpp"
#include "cosm/spatial/strategy/nest/config/acq_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest, acq);

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

NS_END(acq, nest, strategy, spatial, cosm);