/**
 * \file swarm_manager_adaptor.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/ros/swarm_manager_adaptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal, ros);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
swarm_manager_adaptor::swarm_manager_adaptor(void)
    : ER_CLIENT_INIT("cosm.pal.ros.swarm_manager_adaptor") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void swarm_manager_adaptor::init(ticpp::Element& node) {
  /* parse configuration */
} /* init() */

NS_END(ros, pal, cosm);
