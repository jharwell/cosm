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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>
#include <memory>

#include "rcppsw/er/stringizable.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"


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
class acache_vectoro : public rpdecorator::decorator<std::vector<acache_vectoro_type>>,
                       public rer::stringizable {
 public:
  RCPPSW_DECORATE_DECL(value_type);

  RCPPSW_DECORATE_DECLDEF(operator[]);
  RCPPSW_DECORATE_DECLDEF(operator[], const);
  RCPPSW_DECORATE_DECLDEF(size, const);
  RCPPSW_DECORATE_DECLDEF(push_back);
  RCPPSW_DECORATE_DECLDEF(begin);
  RCPPSW_DECORATE_DECLDEF(end);
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(erase);
  RCPPSW_DECORATE_DECLDEF(clear);
  RCPPSW_DECORATE_DECLDEF(empty);

  std::string to_str(void) const override final;
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
class acache_vectorno : public rpdecorator::decorator<std::vector<acache_vectorno_type>>,
                        public rer::stringizable {
 public:
  RCPPSW_DECORATE_DECL(value_type);

  RCPPSW_DECORATE_DECLDEF(operator[]);
  RCPPSW_DECORATE_DECLDEF(operator[], const);
  RCPPSW_DECORATE_DECLDEF(size, const);
  RCPPSW_DECORATE_DECLDEF(push_back);
  RCPPSW_DECORATE_DECLDEF(begin);
  RCPPSW_DECORATE_DECLDEF(end);
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(erase);
  RCPPSW_DECORATE_DECLDEF(clear);
  RCPPSW_DECORATE_DECLDEF(empty);

  std::string to_str(void) const override final;
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
class acache_vectorro : public rpdecorator::decorator<std::vector<acache_vectorro_type>>,
                        public rer::stringizable {
 public:
  RCPPSW_DECORATE_DECL(value_type);

  RCPPSW_DECORATE_DECLDEF(operator[]);
  RCPPSW_DECORATE_DECLDEF(operator[], const);
  RCPPSW_DECORATE_DECLDEF(size, const);
  RCPPSW_DECORATE_DECLDEF(push_back);
  RCPPSW_DECORATE_DECLDEF(begin);
  RCPPSW_DECORATE_DECLDEF(end);
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(erase);
  RCPPSW_DECORATE_DECLDEF(clear);
  RCPPSW_DECORATE_DECLDEF(empty);

  std::string to_str(void) const override;
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
class bcache_vectorno : public rpdecorator::decorator<std::vector<bcache_vectorno_type>>,
                        public rer::stringizable {
 public:
  RCPPSW_DECORATE_DECL(value_type);

  RCPPSW_DECORATE_DECLDEF(operator[]);
  RCPPSW_DECORATE_DECLDEF(operator[], const);
  RCPPSW_DECORATE_DECLDEF(size, const);
  RCPPSW_DECORATE_DECLDEF(push_back);
  RCPPSW_DECORATE_DECLDEF(begin);
  RCPPSW_DECORATE_DECLDEF(end);
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(erase);
  RCPPSW_DECORATE_DECLDEF(clear);
  RCPPSW_DECORATE_DECLDEF(empty);

  std::string to_str(void) const override;
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
class bcache_vectorro : public rpdecorator::decorator<std::vector<bcache_vectorro_type>>,
                        public rer::stringizable {
 public:
  RCPPSW_DECORATE_DECL(value_type);

  RCPPSW_DECORATE_DECLDEF(operator[]);
  RCPPSW_DECORATE_DECLDEF(operator[], const);
  RCPPSW_DECORATE_DECLDEF(size, const);
  RCPPSW_DECORATE_DECLDEF(push_back);
  RCPPSW_DECORATE_DECLDEF(begin);
  RCPPSW_DECORATE_DECLDEF(end);
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(erase);
  RCPPSW_DECORATE_DECLDEF(clear);
  RCPPSW_DECORATE_DECLDEF(empty);

  std::string to_str(void) const override;
};

NS_END(ds, arena, cosm);

