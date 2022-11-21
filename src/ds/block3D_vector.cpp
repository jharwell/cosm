/**
 * \file block3D_vector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ds/block3D_vector.hpp"

#include "cosm/ds/utils.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ds {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string block3D_vectoro::to_str(void) const {
  return cds::to_string(decoratee(), "b");
} /* to_str() */

std::string block3D_vectorno::to_str(void) const {
  return cds::to_string(decoratee(), "b");
} /* to_str() */

std::string block3D_vectorro::to_str(void) const {
  return cds::to_string(decoratee(), "b");
} /* to_str() */

} /* namespace cosm::ds */
