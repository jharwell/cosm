/**
 * \file cache_vector.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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

namespace cosm::arena::ds {

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

} /* namespace cosm::arena::ds */
