/**
 * \file block_extent_set.hpp
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
namespace cosm::repr {
class sim_block3D;
}

namespace cosm::arena::ds {
class arena_grid;
} /* namespace cosm::ds */

namespace cosm::arena::operations {
namespace detail {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_extent_set
 * \ingroup arena operations
 *
 * \brief Set the cells that a block covers while in the arena that are in
 * an empty state to the BLOCK_EXTENT state.
 *
 * \note This operation requires holding the block and grid mutexes in
 *       multithreaded contexts.
 */
class block_extent_set : public rer::client<block_extent_set> {
 private:
  struct visit_typelist_impl {
    using value = rmpl::typelist<cads::arena_grid>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit block_extent_set(crepr::sim_block3D* block);
  block_extent_set& operator=(const block_extent_set&) = delete;
  block_extent_set(const block_extent_set&) = delete;

  void visit(cads::arena_grid& grid);

 private:
  /* clang-format off */
  crepr::sim_block3D* m_block;
  /* clang-format on */
};

} /* namespace detail */


/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using block_extent_set_visitor = rpvisitor::filtered_visitor<detail::block_extent_set>;

} /* namespace cosm::arena::operations */

