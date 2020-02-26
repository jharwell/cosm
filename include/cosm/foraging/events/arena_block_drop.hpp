/**
 * \file arena_block_drop.hpp
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

#ifndef INCLUDE_COSM_FORAGING_EVENTS_ARENA_BLOCK_DROP_HPP_
#define INCLUDE_COSM_FORAGING_EVENTS_ARENA_BLOCK_DROP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/foraging/events/block_drop_base_visit_set.hpp"
#include "cosm/events/cell2D_op.hpp"
#include "cosm/foraging/ds/arena_map_locking.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class arena_block_drop
 * \ingroup foraging events detail
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
 * arena_block_drop_visitor.
 */
class arena_block_drop : public rer::client<arena_block_drop>,
                        public cevents::cell2D_op {
 private:
  struct visit_typelist_impl {
    using value = boost::mpl::joint_view<block_drop_base_visit_typelist,
                                         cell2D_op::visit_typelist>;
  };

 protected:
  /**
   * \param block The block to drop, which is already part of the vector owned
   *              by the \ref arena_map.
   * \param coord The discrete coordinates of the cell to drop the block in.
   * \param resolution The resolution of the arena map.
   * \param locking What locks are currently held by the caller?
   */
  arena_block_drop(const std::shared_ptr<crepr::base_block2D>& block,
                   const rmath::vector2u& coord,
                   const rtypes::discretize_ratio& resolution,
                   const cfds::arena_map_locking& locking);

 public:
  using visit_typelist = visit_typelist_impl::value;

  /**
   * \param coord The discrete coordinates of the cell to drop the block in.
   * \param resolution The resolution of the arena map.
   *
   * If this constructor is used, then the resulting object can ONLY be used to
   * visit blocks; segfaults and/or undefined behavior will occur otherwise.
   */

  static arena_block_drop for_block(const rmath::vector2u& coord,
                                    const rtypes::discretize_ratio& resolution);

  ~arena_block_drop(void) override = default;

  arena_block_drop(const arena_block_drop& op) = delete;
  arena_block_drop& operator=(const arena_block_drop& op) = delete;

  /**
   * \brief Perform actual block drop in the arena, taking/releasing locks as
   * needed.
   */
  void visit(cfds::arena_map& map);

  /**
   * \brief Update the cell the block was dropped into. No locking is performed.
   */
  void visit(cds::cell2D& cell);

  /**
   * \brief Update the dropped block. No locking is performed.
   */
  void visit(crepr::base_block2D& block);

  /**
   * \brief Get the handle on the block that has been dropped.
   */
  std::shared_ptr<crepr::base_block2D> block(void) const { return m_block; }

 private:
  void visit(fsm::cell2D_fsm& fsm);

  /* clang-format off */
  const rtypes::discretize_ratio       mc_resolution;
  const cfds::arena_map_locking        mc_locking;

  std::shared_ptr<crepr::base_block2D> m_block;
  /* clang-format on */
};

/**
 * \brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using arena_block_drop_visitor_impl =
    rpvisitor::precise_visitor<arena_block_drop,
                               arena_block_drop::visit_typelist>;

class arena_block_drop_visitor : public arena_block_drop_visitor_impl {
  using arena_block_drop_visitor_impl::arena_block_drop_visitor_impl;
};

NS_END(events, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_EVENTS_ARENA_BLOCK_DROP_HPP_ */
