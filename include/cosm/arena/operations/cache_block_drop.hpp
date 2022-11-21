/**
 * \file cache_block_drop.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "rcppsw/types/discretize_ratio.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/ds/operations/cell2D_op.hpp"
#include "cosm/cosm.hpp"
#include "cosm/arena/locking.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::repr {
class arena_cache;
} // namespace repr

namespace cosm::arena {
class caching_arena_map;
} /* namespace cosm::arena */

namespace cosm::repr {
class sim_block3D;
} /* namespace cosm::repr */

namespace cosm::arena::operations {
namespace detail {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_block_drop
 * \ingroup arena operations detail
 *
 * \brief Created whenever a robot drops a block in a cache.
 *
 * The cache usuage penalty, if there is one, is not assessed during the event,
 * but at a higher level.
 */
class cache_block_drop : public rer::client<cache_block_drop>,
                         public cdops::cell2D_op {
 private:
  struct visit_typelist_impl {
    using inherited = cell2D_op::visit_typelist;
    using others = rmpl::typelist<caching_arena_map, crepr::sim_block3D>;
    using value = boost::mpl::joint_view<inherited::type, others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  ~cache_block_drop(void) override = default;

  cache_block_drop(const cache_block_drop& op) = delete;
  cache_block_drop& operator=(const cache_block_drop& op) = delete;

  /**
   * \brief Perform actual cache block drop in the arena, taking/releasing locks
   * as needed.
   */
  void visit(caching_arena_map& map);

 protected:
  /**
   * \brief Initialize a cache_block_drop event caused by a robot dropping
   * a block.
   *
   * \param arena_block The block to drop in the cache (MUST be owned by arena).
   * \param cache Cache to drop into (owned by arena).
   * \param resolution Arena resolution.
   * \param locking Is locking needed around block accesses?
   */
  cache_block_drop(crepr::sim_block3D* arena_block,
                   carepr::arena_cache* cache,
                   const rtypes::discretize_ratio& resolution,
                   const locking& locking);

 private:
  void visit(cds::cell2D& cell);
  void visit(fsm::cell2D_fsm& fsm);
  void visit(crepr::sim_block3D& block);
  void visit(carepr::arena_cache& cache);

  /* clang-format off */
  const locking                  mc_locking;
  const rtypes::discretize_ratio mc_resolution;

  crepr::sim_block3D*           m_arena_block;
  carepr::arena_cache*           m_cache;
  /* clang-format on */
};


} /* namespace detail */

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cache_block_drop_visitor = rpvisitor::filtered_visitor<detail::cache_block_drop>;

} /* namespace cosm::arena::operations */
