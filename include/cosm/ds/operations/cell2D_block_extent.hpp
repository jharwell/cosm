/**
 * \file cell2D_block_extent.hpp
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

#ifndef INCLUDE_COSM_DS_OPERATIONS_CELL2D_BLOCK_EXTENT_HPP_
#define INCLUDE_COSM_DS_OPERATIONS_CELL2D_BLOCK_EXTENT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"

#include "cosm/ds/operations/cell2D_op.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class base_block3D;
} // namespace repr

namespace cosm::ds {
class arena_grid;
} // namespace ds

NS_START(cosm, ds, operations, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cell2D_block_extent
 * \ingroup ds operations
 *
 * \brief Created whenever a cell needs to go from some other state to being
 * part of a block's extent (duh). The block itself lives in a single cell, but
 * the cells that the block covers need to be in a special state in order to
 * avoid corner cases when picking up from/dropping in a block.
 */
class cell2D_block_extent : public cdops::cell2D_op {
 private:
  struct visit_typelist_impl {
    using inherited = cell2D_op::visit_typelist;
    using others = rmpl::typelist<cds::arena_grid>;
    using value = boost::mpl::joint_view<inherited::type, others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  cell2D_block_extent(const rmath::vector2z& coord, crepr::base_block3D* block);
  cell2D_block_extent& operator=(const cell2D_block_extent&) = delete;
  cell2D_block_extent(const cell2D_block_extent&) = delete;

  void visit(cds::arena_grid& grid);

 private:
  void visit(cds::cell2D& cell);
  void visit(fsm::cell2D_fsm& fsm);

  /* clang-format off */
  crepr::base_block3D* m_block;
  /* clang-format on */
};

NS_END(detail);

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cell2D_block_extent_visitor = rpvisitor::filtered_visitor<detail::cell2D_block_extent>;

NS_END(operations, ds, cosm);

#endif /* INCLUDE_COSM_DS_OPERATIONS_CELL2D_BLOCK_EXTENT_HPP_ */
