/**
 * \file env_dynamics.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_dynamics
 * \ingroup tv
 *
 * \brief Interface specifying common functionality across projects for the
 * application of temporal variance in environmental conditions to the swarm.
 */
template <typename TController>
class env_dynamics {
 public:
  env_dynamics(void) = default;
  virtual ~env_dynamics(void) = default;

  env_dynamics(const env_dynamics&) = delete;
  const env_dynamics& operator=(const env_dynamics&) = delete;

  /**
   * \brief Register a robot controller for all possible types of environmental
   * variance that could be applied to it.
   */
  virtual void register_controller(const TController& c) = 0;

  /**
   * \brief Undo \ref register_controller(), as well as flushing the controller
   * from any penalty handlers it might be serving penalties for.
   */
  virtual void unregister_controller(const TController& c) = 0;

  /**
   * \brief Flush the controller from serving penalties for ALL penalty
   * handlers.
   */
  virtual bool penalties_flush(const TController& c) = 0;
};

NS_END(tv, cosm);

