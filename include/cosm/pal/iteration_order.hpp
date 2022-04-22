/**
 * \file iteration_order.hpp
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
NS_START(cosm, pal);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \enum iteration_order
 * \ingroup pal
 *
 * \brief The order to iterate through the robots in the swarm for a given
 * operation.
 */
enum class iteration_order {
  /**
   * \brief Iterate through the robots in a static/single-threaded
   * order. Locking between sub-operations is not needed in this case.
   */
  ekSTATIC,

  /**
   * \brief Iterate through the robots using multiple threads where the order is
   * determined on the fly. Locking between sub-operations is (probably) needed
   * in this case.
   */
  ekDYNAMIC,
};

NS_END(pal, cosm);
