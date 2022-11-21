/**
 * \file block3D_ht.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ds/block3D_ht.hpp"

#include <string>

#include "cosm/ds/utils.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ds {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string block3D_hto::to_str(void) const {
  return cds::to_string(decoratee(), "b");
} /* to_str() */

std::string block3D_htno::to_str(void) const {
  return cds::to_string(decoratee(), "b");
} /* to_str() */

std::string block3D_htro::to_str(void) const {
  return cds::to_string(decoratee(), "b");
} /* to_str() */

} /* namespace cosm::ds */
