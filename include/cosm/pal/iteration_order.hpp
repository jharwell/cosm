/**
 * \file iteration_order.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
