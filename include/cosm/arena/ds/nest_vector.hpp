/**
 * \file nest_vector.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_ARENA_DS_NEST_VECTOR_HPP_
#define INCLUDE_COSM_ARENA_DS_NEST_VECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>
#include <memory>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr { class nest; }

NS_START(cosm, arena, ds);

using nest_vectorno_type = crepr::nest*;
using nest_vectorro_type = const crepr::nest*;

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \class nest_vectorno
 * \ingroup arena ds
 *
 * \brief Specialization of \ref std::vector indicating the nests are NOTO owned
 * by this class.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class nest_vectorno : public std::vector<nest_vectorno_type> {
 public:
  using std::vector<nest_vectorno_type>::vector;
  using value_type = std::vector<nest_vectorno_type>::value_type;

  /**
   * \brief Get a string representation of the vector contents.
   */
  std::string to_str(void) const;
};


/**
 * \class nest_vectorro
 * \ingroup arena ds
 *
 * \brief Specialization of \ref std::vector indicating the nests are NOT owned
 * by this class and access is also read-only.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class nest_vectorro : public std::vector<nest_vectorro_type> {
 public:
  using std::vector<nest_vectorro_type>::vector;
  using value_type = std::vector<nest_vectorro_type>::value_type;

  /**
   * \brief Get a string representation of the vector contents.
   */
  std::string to_str(void) const;
};

NS_END(ds, arena, cosm);

#endif /* INCLUDE_COSM_ARENA_DS_NEST_VECTOR_HPP_ */
