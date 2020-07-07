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

#ifndef INCLUDE_COSM_ARENA_DS_CACHE_VECTOR_HPP_
#define INCLUDE_COSM_ARENA_DS_CACHE_VECTOR_HPP_

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

namespace cosm::arena::repr {
class arena_cache;
class base_cache;
}

NS_START(cosm, arena, ds);

/*
 * Must be shared_ptr because the # of caches in the arena can change
 * dynamically, resulting in dynamic vector resizing, which requires copying.
 */
using acache_vectoro_type = std::shared_ptr<carepr::arena_cache>;
using acache_vectorno_type = carepr::arena_cache*;
using acache_vectorro_type = const carepr::arena_cache*;

using bcache_vectorno_type = carepr::base_cache*;
using bcache_vectorro_type = const carepr::base_cache*;

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \class acache_vectoro
 * \ingroup arena ds
 *
 * \brief Specialization of \ref std::vector indicating the caches are OWNED by
 * this class.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class acache_vectoro : public std::vector<acache_vectoro_type> {
 public:
  using std::vector<acache_vectoro_type>::vector;
  using value_type = std::vector<acache_vectoro_type>::value_type;

  /**
   * \brief Get a string representation of the vector contents.
   */
  std::string to_str(void) const;
};

/**
 * \class acache_vectorno
 * \ingroup arena ds
 *
 * \brief Specialization of \ref std::vector indicating the caches are NOT owned
 * by this class.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class acache_vectorno : public std::vector<acache_vectorno_type> {
 public:
  using std::vector<acache_vectorno_type>::vector;
  using value_type = std::vector<acache_vectorno_type>::value_type;

  /**
   * \brief Get a string representation of the vector contents.
   */
  std::string to_str(void) const;
};

/**
 * \class acache_vectorro
 * \ingroup arena ds
 *
 * \brief Specialization of \ref std::vector indicating the caches are NOT owned
 * by this class and access is also read-only.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class acache_vectorro : public std::vector<acache_vectorro_type> {
 public:
  using std::vector<acache_vectorro_type>::vector;
  using value_type = std::vector<acache_vectorro_type>::value_type;

  /**
   * \brief Get a string representation of the vector contents.
   */
  std::string to_str(void) const;
};

/**
 * \class bcache_vectorno
 * \ingroup arena ds
 *
 * \brief Specialization of \ref std::vector indicating the caches are NOT owned
 * by this class.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class bcache_vectorno : public std::vector<bcache_vectorno_type> {
 public:
  using std::vector<bcache_vectorno_type>::vector;
  using value_type = std::vector<bcache_vectorno_type>::value_type;

  /**
   * \brief Get a string representation of the vector contents.
   */
  std::string to_str(void) const;
};

/**
 * \class bcache_vectorro
 * \ingroup arena ds
 *
 * \brief Specialization of \ref std::vector indicating the caches are NOT owned
 * by this class and access is also read-only.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class bcache_vectorro : public std::vector<bcache_vectorro_type> {
 public:
  using std::vector<bcache_vectorro_type>::vector;
  using value_type = std::vector<bcache_vectorro_type>::value_type;

  /**
   * \brief Get a string representation of the vector contents.
   */
  std::string to_str(void) const;
};

NS_END(ds, arena, cosm);

#endif /* INCLUDE_COSM_ARENA_DS_CACHE_VECTOR_HPP_ */
