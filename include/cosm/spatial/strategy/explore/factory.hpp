/**
 * \file factory.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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
#include "cosm/spatial/strategy/explore/base_explore.hpp"
#include "cosm/spatial/fsm/fsm_params.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm);

NS_START(spatial, strategy, explore);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class factory
 * \ingroup spatial strategy explore
 *
 * \brief Factory for creating exploration strategies.
 */
class factory :
    public rpfactory::releasing_factory<cssexplore::base_explore,
                                        std::string, /* key type */
                                        csfsm::fsm_params*,
                                        const cssexplore::config::explore_config*,
                                        rmath::rng*> {
 public:
  static inline const std::string kCRW = "CRW";

  factory(void);
};

NS_END(explore, strategy, spatial, cosm);
