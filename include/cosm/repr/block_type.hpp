/**
 * \file block_type.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_REPR_BLOCK_TYPE_HPP_
#define INCLUDE_COSM_REPR_BLOCK_TYPE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <iosfwd>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

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
  ekRAMP
};

/*******************************************************************************
 * Operators
 ******************************************************************************/
std::ostream& operator<<(std::ostream& out, block_type& b);
std::istream& operator>>(std::istream& in, block_type& b);

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_BLOCK_TYPE_HPP_ */
