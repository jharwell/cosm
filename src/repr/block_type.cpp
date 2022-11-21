/**
 * \file block_type.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/block_type.hpp"

#include <iostream>

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {

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

} /* namespace cosm::repr */
