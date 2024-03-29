/**
 * \file random_block_distributor.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/block_dist/random_distributor.hpp"

#include <algorithm>

#include "cosm/arena/ds/arena_grid.hpp"
#include "cosm/arena/operations/block_extent_set.hpp"
#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/repr/entity2D.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::block_dist {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
random_distributor::random_distributor(
    const cads::arena_grid::view& area,
    cads::arena_grid* arena_grid,
    const cspatial::conflict_checker::map_cb_type& conflict_check,
    const dist_success_cb_type& dist_success,
    rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.foraging.block_dist.random"),
      base_distributor(arena_grid, rng),
      mc_origin(area.origin()->loc()),
      mc_xspan(mc_origin.x(), mc_origin.x() + area.shape()[0]),
      mc_yspan(mc_origin.y(), mc_origin.y() + area.shape()[1]),
      mc_conflict_check(conflict_check),
      mc_dist_success(dist_success),
      m_area(area) {
  ER_INFO("Area: xrange=%s,yrange=%s,resolution=%f",
          mc_xspan.to_str().c_str(),
          mc_yspan.to_str().c_str(),
          arena_grid->resolution().v());
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
dist_status random_distributor::distribute_block(crepr::sim_block3D* block) {
  cds::cell2D* cell = nullptr;
  auto coords = coord_search(block);
  if (!coords) {
    ER_WARN("Unable to find distribution coordinates for block%d",
            block->id().v());
    return dist_status::ekFAILURE;
  }
  ER_INFO("Found coordinates for distributing block%d: rel=%s, abs=%s",
          block->id().v(),
          rcppsw::to_string(coords->rel).c_str(),
          rcppsw::to_string(coords->abs).c_str());

  cell = &m_area[coords->rel.x()][coords->rel.y()];

  /*
   * You can only distribute blocks to cells that do not currently have
   * anything in them. If there is already something there, then our
   * distribution algorithm has a bug.
   */
  ER_ASSERT(!cell->state_has_block(),
            "Destination cell@%s already contains block%d",
            rcppsw::to_string(coords->abs).c_str(),
            cell->entity()->id().v());
  ER_ASSERT(!cell->state_has_cache(),
            "Destination cell@%s already contains cache%d",
            rcppsw::to_string(coords->abs).c_str(),
            cell->entity()->id().v());
  ER_ASSERT(!cell->state_in_cache_extent(),
            "Destination cell part of cache extent");
  ER_ASSERT(cell->state_is_empty() || !cell->state_is_known(),
            "Destination cell@%s is not empty",
            rcppsw::to_string(coords->abs).c_str());

  /*
   * This function is always called from the arena map, and it ensures that
   * all locks are held, so we don't need to do anything here.
   */
  caops::free_block_drop_visitor drop_op(block,
                                         coords->abs,
                                         arena_grid()->resolution(),
                                         carena::locking::ekALL_HELD);
  caops::block_extent_set_visitor extent_op(block);

  /* Update block position */
  drop_op.visit(*cell);
  drop_op.visit(*block);

  /* with new block position, set extent */
  extent_op.visit(*arena_grid());

  if (verify_block_dist(block, cell)) {
    ER_DEBUG("Block%d,ptr=%p distributed@%s/%s",
             block->id().v(),
             block,
             rcppsw::to_string(block->ranchor2D()).c_str(),
             rcppsw::to_string(block->danchor2D()).c_str());
    /*
     * Now that the block has been distributed, it is another entity that
     * needs to be avoided during subsequent distributions. This class doesn't
     * know how to add the block to the list of entities to be avoided during
     * distribution, so we call back into the arena map to do it.
     */
    mc_dist_success(block);
    return dist_status::ekSUCCESS;
  }
  ER_WARN("Failed to distribute block%d after finding distribution coord",
          block->id().v());
  return dist_status::ekFAILURE;
} /* distribute_block() */

bool random_distributor::verify_block_dist(
    const crepr::sim_block3D* const block,
    RCPPSW_UNUSED const cds::cell2D* const cell) {
  /* blocks should not be out of sight after distribution... */
  ER_CHECK(!block->is_out_of_sight(),
           "Block%d discrete coord still out of sight after distribution",
           block->id().v());

  /* The cell it was distributed to should refer to it */
  ER_CHECK(block == cell->entity(),
           "Block%d@%s not referenced by containing cell@%s",
           block->id().v(),
           rcppsw::to_string(block->ranchor2D()).c_str(),
           rcppsw::to_string(cell->loc()).c_str());

  /* The cell it was distributed to should be in the HAS_BLOCK state */
  ER_ASSERT(cell->state_has_block(),
            "Destination cell@%s not in HAS_BLOCK for block%d",
            rcppsw::to_string(cell->loc()).c_str(),
            cell->entity()->id().v());

  /* no entity should overlap with the block after distribution */
  {
    auto status = mc_conflict_check(block, block->ranchor2D());
    ER_ASSERT(!(status.x && status.y),
              "Placement conflict for block%d@%s/%s after distribution",
              block->id().v(),
              rcppsw::to_string(block->ranchor2D()).c_str(),
              rcppsw::to_string(block->danchor2D()).c_str());
  }
  return true;

error:
  return false;
} /* verify_block_dist() */

boost::optional<typename random_distributor::coord_search_res_t>
random_distributor::coord_search(const crepr::sim_block3D* block) {
  /* -1 because we are working with array indices */
  rmath::rangez area_xrange(m_area.index_bases()[0], m_area.shape()[0] - 1);
  rmath::rangez area_yrange(m_area.index_bases()[1], m_area.shape()[1] - 1);

  ER_INFO("Coordinate search: rel_xrange=%s,rel_yrange=%s",
          rcppsw::to_string(area_xrange).c_str(),
          rcppsw::to_string(area_yrange).c_str());

  if (coord_search_policy::ekFREE_CELL == m_search_policy) {
    ER_DEBUG("Using FREE_CELL policy");
    return coord_search_free_cell(area_xrange, area_yrange, block);
  } else if (coord_search_policy::ekRANDOM == m_search_policy) {
    ER_DEBUG("Using RANDOM policy");
    return coord_search_random(area_xrange, area_yrange, block);
  }
  return boost::none;
} /* coord_search() */

boost::optional<random_distributor::coord_search_res_t>
random_distributor::coord_search_random(const rmath::rangez& c_xrange,
                                        const rmath::rangez& c_yrange,
                                        const crepr::sim_block3D* block) {
  /*
   * Try to find an available set of relative+absolute coordinates such that if
   * the entity is placed there it will not overlap any other entities in the
   * arena. You only have a finite number of tries, for obvious reasons.
   */
  size_t count = 0;
  while (count++ < kMAX_DIST_TRIES) {
    size_t x = c_xrange.span() > 0 ? rng()->uniform(c_xrange) : mc_origin.x();
    size_t y = c_xrange.span() > 0 ? rng()->uniform(c_yrange) : mc_origin.y();
    rmath::vector2z rel(x, y);
    rmath::vector2z abs = mc_origin + rel;
    auto abs_r = rmath::zvec2dvec(abs, arena_grid()->resolution().v());
    if (coord_conflict_check(block, abs_r)) {
      coord_search_res_t coord = { rel, abs };
      return boost::make_optional(coord);
    }
  } /* while() */
  return boost::none;
} /* coord_search_random() */

boost::optional<random_distributor::coord_search_res_t>
random_distributor::coord_search_free_cell(const rmath::rangez& c_xrange,
                                           const rmath::rangez& c_yrange,
                                           const crepr::sim_block3D* block) {
  std::vector<rmath::vector2z> rel_coords;
  for (size_t i = c_xrange.lb(); i <= c_xrange.ub(); ++i) {
    for (size_t j = c_yrange.lb(); j <= c_yrange.ub(); ++j) {
      auto& cell = m_area[i][j];
      if (!cell.state_is_known() || cell.state_is_empty()) {
        rel_coords.push_back({ i, j });
      }
    } /* for(j..) */
  } /* for(i..) */
  size_t start = rng()->uniform(rmath::rangez(0, rel_coords.size() - 1));
  for (size_t i = 0; i < rel_coords.size(); ++i) {
    auto rel = rel_coords[(start + i) % rel_coords.size()];
    auto abs = mc_origin + rel;
    auto abs_r = rmath::zvec2dvec(abs, arena_grid()->resolution().v());
    if (coord_conflict_check(block, abs_r)) {
      coord_search_res_t coord = { rel, abs };
      return boost::make_optional(coord);
    }
  } /* for(i..) */
  return boost::none;
} /* coord_search_free_cell() */

bool random_distributor::coord_conflict_check(
    const crepr::sim_block3D* block,
    const rmath::vector2d& drop_loc) const {
  /*
   * If this distributor is part of a powerlaw distributor, blocks will not be
   * packed very tightly together via checking real rather than discrete
   * overlaps as is done below, because two blocks placed adjacently to each
   * other MIGHT say they overlap even though they don't, because of floating
   * point representation error, depending on block size and where they are
   * located in the arena.
   *
   * @todo This should probably be fixed at some point, or at least made a
   * user-controllable switch via input parameter.
   */
  auto conflict = mc_conflict_check(block, drop_loc);
  return !(conflict.x && conflict.y);
} /* coord_conflict_check() */

} /* namespace cosm::foraging::block_dist */
