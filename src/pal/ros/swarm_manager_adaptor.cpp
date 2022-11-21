/**
 * \file swarm_manager_adaptor.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/ros/swarm_manager_adaptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::pal::ros {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
swarm_manager_adaptor::swarm_manager_adaptor(size_t n_robots)
    : ER_CLIENT_INIT("cosm.pal.ros.swarm_manager_adaptor"),
      mc_n_robots(n_robots) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

} /* namespace cosm::pal::ros */
