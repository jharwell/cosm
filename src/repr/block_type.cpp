/**
 * \file block_type.cpp
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
#include "cosm/repr/block_type.hpp"

#include <iostream>

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Operators
 ******************************************************************************/
std::ostream& operator<<(std::ostream& out, const block_type& b) {
  out << reinterpret_cast<const std::underlying_type<block_type>::type&>(b);
  return out;
}

std::istream& operator>>(std::istream& in, block_type& b) {
  in >> reinterpret_cast<std::underlying_type<block_type>::type&>(b);
  return in;
}

NS_END(repr, cosm);
