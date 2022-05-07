/**
 * \file base_exit.hpp
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
#include "cosm/spatial/strategy/base_strategy.hpp"
#include "cosm/spatial/strategy/nest/config/exit_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest, exit);

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

NS_END(exit, nest, strategy, spatial, cosm);
