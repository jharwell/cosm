/**
 * \file block_type.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <iosfwd>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief The different types of blocks available in simulation. This
 * enumeration is used to eliminate the need to include the actual block type
 * header files everywhere.
 */
enum class block_type {
  ekNONE,
  /**
   * \brief A simple cubical block, AxAxA.
   */
  ekCUBE,

  /**
   * \brief A ramp block of dimension AxBxA, where A = 2B (shaped like a
   * doorstop).
   */
  ekRAMP,
};

/*******************************************************************************
 * Operators
 ******************************************************************************/
std::ostream& operator<<(std::ostream& out, const block_type& b);
std::istream& operator>>(std::istream& in, block_type& b);

} /* namespace cosm::repr */
