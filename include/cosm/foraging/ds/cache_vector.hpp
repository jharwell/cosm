/**
 * \file cache_vector.hpp
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

#ifndef INCLUDE_COSM_FORAGING_DS_CACHE_VECTOR_HPP_
#define INCLUDE_COSM_FORAGING_DS_CACHE_VECTOR_HPP_

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

namespace cosm::foraging::repr {
class arena_cache;
}

NS_START(cosm, foraging, ds);

using cache_vector_type = std::shared_ptr<cfrepr::arena_cache>;

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \class cache_vector
 * \ingroup foraging ds
 *
 * \brief Specialization of \ref std::vector with an additional \ref to_str()
 * method.
 */
class cache_vector : public std::vector<cache_vector_type> {
 public:
  using std::vector<cache_vector_type>::vector;
  using value_type = std::vector<cache_vector_type>::value_type;

  /**
   * \brief Get a string representation of the vector contents.
   */
  std::string to_str(void) const;
};

NS_END(ds, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_DS_CACHE_VECTOR_HPP_ */
