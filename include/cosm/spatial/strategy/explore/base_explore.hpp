/**
 * \file base_explore.hpp
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

#include "cosm/spatial/strategy/explore/config/explore_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, explore);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_explore
 * \ingroup spatial strategy explore
 *
 * \brief Base class for exploration strategies, to enable construction using
 * the factory pattern.
 */

class base_explore : public csstrategy::base_strategy,
                     public rpprototype::clonable<base_explore> {
 public:
  base_explore(const csfsm::fsm_params* params,
               const cssexplore::config::explore_config*,
               rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  base_explore(const base_explore&) = delete;
  base_explore& operator=(const base_explore&) = delete;
  base_explore(base_explore&&) = delete;
  base_explore& operator=(base_explore&&) = delete;

  virtual bool min_duration_met(void) const {
    return m_steps >= mc_config.min_duration;
  }

  /* taskable overrides */
  void task_reset(void) override { m_steps = rtypes::timestep(0); }
  void task_execute(void) override { ++m_steps; }

 protected:
  const config::explore_config* config(void) const { return &mc_config; }
  const rtypes::timestep& steps(void) const { return m_steps; }

 private:
  /* clang-formatt off */
  const cssexplore::config::explore_config mc_config;

  rtypes::timestep                         m_steps{rtypes::timestep(0)};
  /* clang-formatt on */
};

NS_END(explore, strategy, spatial, cosm);
