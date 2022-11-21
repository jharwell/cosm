/**
 * \file cache_extent_set.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/mpl/typelist.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena {
class caching_arena_map;

namespace repr {
class arena_cache;
}}

namespace cosm::arena::ds {
class arena_grid;
} /* namespace cosm::ds */

namespace cosm::arena::operations {
namespace detail {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_extent_set
 * \ingroup arena operations
 *
 * \brief Set the cells that a cache covers while in the arena from an empty or
 * unknown state to the CACHE_EXTENT state.
 *
 * \note This operation requires holding the cache and grid mutexes in
 *       multithreaded contexts.
 */
class cache_extent_set : public rer::client<cache_extent_set> {
 private:
  struct visit_typelist_impl {
    using value = rmpl::typelist<cads::arena_grid>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit cache_extent_set(carepr::arena_cache* cache);
  cache_extent_set& operator=(const cache_extent_set&) = delete;
  cache_extent_set(const cache_extent_set&) = delete;

  void visit(cads::arena_grid& grid);

 private:
  /* clang-format off */
  carepr::arena_cache* m_cache;
  /* clang-format on */
};

} /* namespace detail */


/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cache_extent_set_visitor = rpvisitor::filtered_visitor<detail::cache_extent_set>;

} /* namespace cosm::arena::operations */

