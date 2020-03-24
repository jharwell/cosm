/**
 * \file cell2D_cache_extent.hpp
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

#ifndef INCLUDE_COSM_ARENA_OPERATIONS_CELL2D_CACHE_EXTENT_HPP_
#define INCLUDE_COSM_ARENA_OPERATIONS_CELL2D_CACHE_EXTENT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"

#include "cosm/ds/operations/cell2D_op.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::repr {
class base_cache;
} // namespace repr

namespace cosm::ds {
class arena_grid;
} // namespace ds

NS_START(cosm, arena, operations, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cell2D_cache_extent
 * \ingroup arena operations
 *
 * \brief Created whenever a cell needs to go from some other state to being
 * part of a cache's extent (duh). All the blocks (and the cache itself) live in
 * a single cell, but the cells that the cache covers need to be in a special
 * state in order to avoid corner cases when picking up from/dropping in a
 * cache.
 */
class cell2D_cache_extent : public cdops::cell2D_op {
 private:
  struct visit_typelist_impl {
    using inherited = cell2D_op::visit_typelist;
    using others = rmpl::typelist<cds::arena_grid>;
    using value = boost::mpl::joint_view<inherited::type, others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  cell2D_cache_extent(const rmath::vector2u& coord, carepr::base_cache* cache);
  cell2D_cache_extent& operator=(const cell2D_cache_extent&) = delete;
  cell2D_cache_extent(const cell2D_cache_extent&) = delete;

  void visit(cds::cell2D& cell);
  void visit(fsm::cell2D_fsm& fsm);
  void visit(cds::arena_grid& grid);

 private:
  /* clang-format off */
  carepr::base_cache* m_cache;
  /* clang-format on */
};

/**
 * \brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cell2D_cache_extent_visitor_impl =
    rpvisitor::precise_visitor<detail::cell2D_cache_extent,
                               detail::cell2D_cache_extent::visit_typelist>;

NS_END(detail);

class cell2D_cache_extent_visitor : public detail::cell2D_cache_extent_visitor_impl {
 public:
  using detail::cell2D_cache_extent_visitor_impl::cell2D_cache_extent_visitor_impl;
};

NS_END(operations, arena, cosm);

#endif /* INCLUDE_COSM_ARENA_OPERATIONS_CELL2D_CACHE_EXTENT_HPP_ */
