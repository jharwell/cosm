/**
 * \file free_block_drop.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_ARENA_OPERATIONS_FREE_BLOCK_DROP_HPP_
#define INCLUDE_COSM_ARENA_OPERATIONS_FREE_BLOCK_DROP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/ds/operations/cell2D_op.hpp"
#include "cosm/arena/arena_map_locking.hpp"
#include "cosm/repr/block_variant.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena {
class caching_arena_map;
class base_arena_map;
} /* namespace cosm::arena */

namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

NS_START(cosm, arena, operations, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class free_block_drop
 * \ingroup arena operations detail
 *
 * \brief Created whenever a block is dropped somewhere in the arena to handle
 * block dropping for non-controller entities. Handling the controller side of
 * block dropping in the arena cannot be handled here generically, so we don't
 * try.
 *
 * Free block drops can happen when:
 *
 * - The loop functions are doing block distribution.
 * - A robot aborts its task, and is carrying a block.
 *
 * This class should never be instantiated, only derived from. To visit
 * non-controller entities to handle block dropping, use \ref
 * free_block_drop_visitor.
 */
class free_block_drop : public rer::client<free_block_drop>,
                        public cdops::cell2D_op {
 private:
  struct visit_typelist_impl {
    using inherited = cell2D_op::visit_typelist;
    using others = rmpl::typelist<base_arena_map,
                                  caching_arena_map,
                                  crepr::base_block3D>;
    using value = boost::mpl::joint_view<inherited::type, others::type>;
  };

 public:
  using visit_typelist = typename visit_typelist_impl::value;

  /**
   * \param coord The discrete coordinates of the cell to drop the block in.
   * \param resolution The resolution of the arena map.
   *
   * If this constructor is used, then the resulting object can ONLY be used to
   * visit blocks; segfaults and/or undefined behavior will occur otherwise.
   */

  static free_block_drop for_block(const rmath::vector2z& coord,
                                   const rtypes::discretize_ratio& resolution);

  ~free_block_drop(void) override = default;

  free_block_drop(const free_block_drop&) = delete;
  free_block_drop& operator=(const free_block_drop&) = delete;

  /**
   * \brief Perform actual block drop in the arena, taking/releasing locks as
   * needed.
   */
  void visit(base_arena_map& map);
  void visit(caching_arena_map& map);

  /**
   * \brief Update the cell the block was dropped into. No locking is performed.
   */
  void visit(cds::cell2D& cell);

  /**
   * \brief Update the dropped block. No locking is performed.
   */
  void visit(crepr::base_block3D& block);

 protected:
  /**
   * \param block The block to drop, which is already part of the vector owned
   *              by the \ref arena_map.
   * \param coord The discrete coordinates of the cell to drop the block in.
   * \param resolution The resolution of the arena map.
   * \param locking What locks are currently held by the caller?
   */
  free_block_drop(crepr::base_block3D* block,
                  const rmath::vector2z& coord,
                  const rtypes::discretize_ratio& resolution,
                  const arena_map_locking& locking,
                  bool update_clusters);

 private:
  void visit(fsm::cell2D_fsm& fsm);

  /* clang-format off */
  const rtypes::discretize_ratio mc_resolution;
  const arena_map_locking        mc_locking;
  const bool                     mc_update_clusters;

  crepr::base_block3D*           m_block;
  /* clang-format on */
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using free_block_drop_visitor_impl =
    rpvisitor::precise_visitor<free_block_drop,
                               free_block_drop::visit_typelist>;

NS_END(detail);

class free_block_drop_visitor : public detail::free_block_drop_visitor_impl {
 public:
  using detail::free_block_drop_visitor_impl::free_block_drop_visitor_impl;
};


NS_END(operations, arena, cosm);

#endif /* INCLUDE_COSM_ARENA_OPERATIONS_FREE_BLOCK_DROP_HPP_ */
