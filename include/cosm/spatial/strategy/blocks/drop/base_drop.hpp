/**
 * \file base_drop.hpp
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
#include "cosm/spatial/strategy/base_strategy.hpp"

#include "cosm/spatial/strategy/blocks/config/drop_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, blocks, drop);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_drop
 * \ingroup spatial strategy blocks drop
 *
 * \brief Base class for block drop strategies, to make doing experiments with
 * real robots where they actually have to do SOMETHING to drop a carried block
 * easier.
 */

class base_drop : public csstrategy::base_strategy,
                  public rpprototype::clonable<base_drop> {
 public:
  base_drop(const csfsm::fsm_params* params,
            const cssblocks::config::drop_config*,
            rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  base_drop(const base_drop&) = delete;
  base_drop& operator=(const base_drop&) = delete;
  base_drop(base_drop&&) = delete;
  base_drop& operator=(base_drop&&) = delete;

 protected:
  const cssblocks::config::drop_config* config(void) const { return &mc_config; }

 private:
  /* clang-formatt off */
  const cssblocks::config::drop_config mc_config;
  /* clang-formatt on */
};

NS_END(drop, blocks, strategy, spatial, cosm);
