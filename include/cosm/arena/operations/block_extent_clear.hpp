/**
 * \file block_extent_clear.hpp
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
namespace cosm::repr {
class sim_block3D;
}

namespace cosm::arena::ds {
class arena_grid;
} /* namespace cosm::ds */

NS_START(cosm, arena, operations, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_extent_clear
 * \ingroup arena operations
 *
 * \brief Clear the cells that a block covers while in the arena that are in
 * BLOCK_EXTENT state, resetting them to EMPTY. Called right before deleting
 * the block from the arena.
 *
 * \note This operation requires holding the block and grid mutexes in
 *       multithreaded contexts.
 */
class block_extent_clear : public rer::client<block_extent_clear> {
 private:
  struct visit_typelist_impl {
    using value = rmpl::typelist<cads::arena_grid>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit block_extent_clear(crepr::sim_block3D* victim);
  block_extent_clear& operator=(const block_extent_clear&) = delete;
  block_extent_clear(const block_extent_clear&) = delete;

  void visit(cads::arena_grid& grid);

 private:
  /* clang-format off */
  crepr::sim_block3D* m_victim;
  /* clang-format on */
};

NS_END(detail);


/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using block_extent_clear_visitor = rpvisitor::filtered_visitor<detail::block_extent_clear>;

NS_END(operations, arena, cosm);

