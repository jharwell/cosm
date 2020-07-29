/**
 * \file random_block_distributor.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/block_dist/random_distributor.hpp"

#include <algorithm>

#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/foraging/utils/utils.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/repr/entity2D.hpp"
#include "cosm/arena/operations/block_extent_set.hpp"
#include "cosm/ds/arena_grid.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
random_distributor::random_distributor(const cds::arena_grid::view& area,
                                       cds::arena_grid* arena_grid,
                                       rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.foraging.block_dist.random"),
      base_distributor(arena_grid, rng),
      mc_origin(area.origin()->loc()),
      mc_xspan(mc_origin.x(), mc_origin.x() + area.shape()[0]),
      mc_yspan(mc_origin.y(), mc_origin.y() + area.shape()[1]),
      m_area(area) {
  ER_INFO("Area: xrange=%s,yrange=%s,resolution=%f",
          mc_xspan.to_str().c_str(),
          mc_yspan.to_str().c_str(),
          arena_grid->resolution().v());
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
dist_status random_distributor::distribute_block(crepr::base_block3D* block,
                                                 cds::const_spatial_entity_vector& entities) {
  cds::cell2D* cell = nullptr;
  auto coords = coord_search(entities, block->rdim2D());
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
                                         carena::arena_map_locking::ekALL_HELD);
  caops::block_extent_set_visitor extent_op(block);

  /* visit cell to update block position */
  drop_op.visit(*cell);

  /* with new block position, set extent */
  extent_op.visit(*arena_grid());

  if (verify_block_dist(block, entities, cell)) {
    ER_DEBUG("Block%d,ptr=%p distributed@%s/%s",
             block->id().v(),
             block,
             rcppsw::to_string(block->ranchor2D()).c_str(),
             rcppsw::to_string(block->danchor2D()).c_str());
    /*
     * Now that the block has been distributed, it is another entity that
     * needs to be avoided during subsequent distributions.
     */
    entities.push_back(block);
    return dist_status::ekSUCCESS;
  }
  ER_WARN("Failed to distribute block%d after finding distribution coord",
          block->id().v());
  return dist_status::ekFAILURE;
} /* distribute_block() */

bool random_distributor::verify_block_dist(
    const crepr::base_block3D* const block,
    const cds::const_spatial_entity_vector& entities,
    RCSW_UNUSED const cds::cell2D* const cell) {
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
  for (auto& e : entities) {
    if (e == block) {
      continue;
    }
    utils::placement_status_t status;
    if (crepr::entity_dimensionality::ek2D == e->dimensionality()) {
      status =
          utils::placement_conflict2D(block->ranchor2D(),
                                      block->rdim2D(),
                                      static_cast<const crepr::entity2D*>(e));
    } else {
      status =
          utils::placement_conflict2D(block->ranchor2D(),
                                      block->rdim2D(),
                                      static_cast<const crepr::entity3D*>(e));
    }
    ER_ASSERT(!(status.x_conflict && status.y_conflict),
              "Placement conflict for block%d@%s/%s after distribution",
              block->id().v(),
              rcppsw::to_string(block->ranchor2D()).c_str(),
              rcppsw::to_string(block->danchor2D()).c_str());
  } /* for(&e..) */
  return true;

error:
  return false;
} /* verify_block_dist() */

boost::optional<typename random_distributor::coord_search_res_t> random_distributor::
coord_search(const cds::const_spatial_entity_vector& c_entities,
                   const rmath::vector2d& c_block_dim) {
  /* -1 because we are working with array indices */
  rcppsw::math::rangez area_xrange(m_area.index_bases()[0],
                                   m_area.shape()[0] - 1);
  rcppsw::math::rangez area_yrange(m_area.index_bases()[1],
                                   m_area.shape()[1] - 1);

  ER_INFO("Coordinate search: rel_xrange=%s,rel_yrange=%s,block_dim=%s,n_entities=%zu",
          rcppsw::to_string(area_xrange).c_str(),
          rcppsw::to_string(area_yrange).c_str(),
          rcppsw::to_string(c_block_dim).c_str(),
          c_entities.size());

  if (coord_search_policy::ekFREE_CELL == m_search_policy) {
    ER_INFO("Using FREE_CELL policy");
    return coord_search_free_cell(area_xrange,
                                  area_yrange,
                                  c_entities,
                                  c_block_dim);
  } else if (coord_search_policy::ekRANDOM == m_search_policy) {
    ER_INFO("Using RANDOM policy");
    return coord_search_random(area_xrange,
                               area_yrange,
                               c_entities,
                               c_block_dim);
  }
  return boost::none;
} /* avail_coord_search() */

boost::optional<random_distributor::coord_search_res_t> random_distributor::coord_search_random(
    const rcppsw::math::rangez& c_xrange,
    const rcppsw::math::rangez& c_yrange,
    const cds::const_spatial_entity_vector& c_entities,
    const rmath::vector2d& c_block_dim) {
  /*
   * Try to find an available set of relative+absolute coordinates such that if
   * the entity is placed there it will not overlap any other entities in the
   * arena. You only have a finite number of tries, for obvious reasons.
   */
  size_t count = 0;
  while (count++ < kMAX_DIST_TRIES) {
    size_t x = c_xrange.span() > 0
               ? rng()->uniform(c_xrange.lb(), c_xrange.ub())
               : m_area.index_bases()[0];
    size_t y = c_xrange.span() > 0
               ? rng()->uniform(c_yrange.lb(), c_yrange.ub())
               : m_area.index_bases()[1];
    rmath::vector2z rel(x, y);
    rmath::vector2z abs = mc_origin + rel;

    if (coord_conflict_check(abs, c_entities, c_block_dim)) {
      coord_search_res_t coord = {rel, abs};
      return boost::make_optional(coord);
    }
  } /* while() */
  return boost::none;
} /* coord_search_random() */

boost::optional<random_distributor::coord_search_res_t> random_distributor::coord_search_free_cell(
    const rcppsw::math::rangez& c_xrange,
    const rcppsw::math::rangez& c_yrange,
    const cds::const_spatial_entity_vector& c_entities,
    const rmath::vector2d& c_block_dim) {
  std::vector<rmath::vector2z> rel_coords;
  for (size_t i = c_xrange.lb(); i <= c_xrange.ub(); ++i) {
    for (size_t j = c_yrange.lb(); j <= c_yrange.ub() ; ++j) {
      auto& cell = m_area[i][j];
      if (!cell.state_is_known() || cell.state_is_empty()) {
        rel_coords.push_back({i, j});
      }
    } /* for(j..) */
  } /* for(i..) */
  size_t start = rng()->uniform(0UL, rel_coords.size() - 1);
  for (size_t i = 0; i < rel_coords.size(); ++i) {
    auto rel = rel_coords[(start + i) % rel_coords.size()];
    auto abs = mc_origin + rel;
    if (coord_conflict_check(abs, c_entities, c_block_dim)) {
      coord_search_res_t coord = {rel, abs};
      return boost::make_optional(coord);
    }
  } /* for(i..) */
  return boost::none;
} /* coord_search_free_cell() */

bool random_distributor::coord_conflict_check(
    const rmath::vector2z& c_coord,
    const cds::const_spatial_entity_vector& c_entities,
    const rmath::vector2d& c_block_dim) const {
  rmath::vector2z ddim = rmath::dvec2zvec(c_block_dim,
                                           arena_grid()->resolution().v());
  rmath::rangez xrange(c_coord.x(), c_coord.x() + ddim.x());
  rmath::rangez yrange(c_coord.y(), c_coord.y() + ddim.y());

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
  auto check_conflict = [&](const auto* ent) {
    utils::placement_status_t status;
    rmath::vector2d abs_r = rmath::zvec2dvec(c_coord,
                                             arena_grid()->resolution().v());
    if (crepr::entity_dimensionality::ek2D == ent->dimensionality()) {
      status = utils::placement_conflict2D(
          abs_r, c_block_dim, static_cast<const crepr::entity2D*>(ent));
    } else {
      status = utils::placement_conflict2D(
          abs_r, c_block_dim, static_cast<const crepr::entity3D*>(ent));
    }
    return status.x_conflict && status.y_conflict;
  };

  return std::none_of(c_entities.begin(), c_entities.end(), check_conflict);
} /* coord_conflict_check() */

NS_END(block_dist, foraging, cosm);
