/**
 * \file cache_extent_clear.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_ARENA_OPERATIONS_CACHE_EXTENT_CLEAR_HPP_
#define INCLUDE_COSM_ARENA_OPERATIONS_CACHE_EXTENT_CLEAR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"
#include "cosm/ds/operations/cell2D_op.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena {
class arena_map;
namespace repr {
class arena_cache;
} /* namespace repr */
} /* namespace cosm::arena */

namespace cosm::ds {
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
class cache_extent_clear : public rer::client<cache_extent_clear>,
                           public cdops::cell2D_op {
 private:
  struct visit_typelist_impl {
    using inherited = cell2D_op::visit_typelist;
    using others = rmpl::typelist<arena_map, cds::arena_grid>;
    using value = boost::mpl::joint_view<inherited::type, others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  cache_extent_clear(const rmath::vector2u& coord, carepr::arena_cache* victim);
  cache_extent_clear& operator=(const cache_extent_clear&) = delete;
  cache_extent_clear(const cache_extent_clear&) = delete;

  void visit(arena_map& map);

 private:
  void visit(cds::arena_grid& grid);

  /* clang-format off */
  carepr::arena_cache* m_victim;
  /* clang-format on */
};

/**
 * \brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cache_extent_clear_visitor_impl =
    rpvisitor::precise_visitor<detail::cache_extent_clear,
                               detail::cache_extent_clear::visit_typelist>;

NS_END(detail);

class cache_extent_clear_visitor : public detail::cache_extent_clear_visitor_impl {
 public:
  using detail::cache_extent_clear_visitor_impl::cache_extent_clear_visitor_impl;
};

NS_END(operations, arena, cosm);

#endif /* INCLUDE_COSM_ARENA_OPERATIONS_CACHE_EXTENT_CLEAR_HPP_ */
