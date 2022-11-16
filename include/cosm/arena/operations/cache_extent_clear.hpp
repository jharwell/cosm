/**
 * \file cache_extent_clear.hpp
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
namespace cosm::arena::repr {
class arena_cache;
}

namespace cosm::arena::ds {
class arena_grid;
} /* namespace cosm::ds */

NS_START(cosm, arena, operations, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_extent_clear
 * \ingroup arena operations
 *
 * \brief Clear the cells that a cache covers while in the arena that are in
 * CACHE_EXTENT state, resetting them to EMPTY. Called right before deleting
 * the cache from the arena.
 *
 * \note This operation requires holding the cache and grid mutexes in
 *       multithreaded contexts.
 */
class cache_extent_clear : public rer::client<cache_extent_clear> {
 private:
  struct visit_typelist_impl {
    using value = rmpl::typelist<cads::arena_grid>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit cache_extent_clear(carepr::arena_cache* victim);
  cache_extent_clear& operator=(const cache_extent_clear&) = delete;
  cache_extent_clear(const cache_extent_clear&) = delete;

  void visit(cads::arena_grid& grid);

 private:
  /* clang-format off */
  carepr::arena_cache* m_victim;
  /* clang-format on */
};

NS_END(detail);


/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cache_extent_clear_visitor = rpvisitor::filtered_visitor<detail::cache_extent_clear>;

NS_END(operations, arena, cosm);

