/**
 * \file block3D_list.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ds/block3D_list.hpp"

#include "cosm/ds/utils.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ds {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string block3D_listno::to_str(void) const {
  return cds::to_string(m_impl, "b");
} /* to_str() */

} /* namespace cosm::ds */
