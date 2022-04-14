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
#include "cosm/spatial/strategy/block_drop/base_block_drop.hpp"
#include "cosm/spatial/fsm/fsm_params.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm);

NS_START(spatial, strategy, block_drop);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class factory
 * \ingroup spatial strategy block_drop
 *
 * \brief Factory for creating block drop strategies.
 */
class factory :
    public rpfactory::releasing_factory<csstrategy::block_drop::base_block_drop,
                                        std::string, /* key type */
                                        csfsm::fsm_params*,
                                        const config::block_drop_config*,
                                        rmath::rng*> {
 public:
  static inline const std::string kBackup = "backup";
  static inline const std::string kPivot = "pivot";

  factory(void);
};

NS_END(block_drop, strategy, spatial, cosm);
