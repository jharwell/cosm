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

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
random_distributor::random_distributor(const cds::arena_grid::view& grid,
                                       const rtypes::discretize_ratio& resolution,
                                       rmath::rng* rng_in)
    : ER_CLIENT_INIT("cosm.foraging.block_dist.random"),
      base_distributor(rng_in),
      mc_resolution(resolution),
      mc_origin(grid.origin()->loc()),
      mc_xspan(mc_origin.x(), mc_origin.x() + grid.shape()[0]),
      mc_yspan(mc_origin.y(), mc_origin.y() + grid.shape()[1]),
      m_grid(grid) {
  ER_INFO("Area: xrange=%s,yrange=%s,resolution=%f",
          mc_xspan.to_str().c_str(),
          mc_yspan.to_str().c_str(),
          mc_resolution.v());
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
dist_status random_distributor::distribute_block(crepr::base_block3D* block,
                                                 cds::const_spatial_entity_vector& entities) {
  cds::cell2D* cell = nullptr;
  auto coords = avail_coord_search(entities, block->rdim2D());
  if (!coords) {
    ER_WARN("Unable to find distribution coordinates for block%d",
            block->id().v());
    return dist_status::ekFAILURE;
  }
  ER_INFO("Found coordinates for distributing block%d: rel=%s, abs=%s",
          block->id().v(),
          rcppsw::to_string(coords->rel).c_str(),
          rcppsw::to_string(coords->abs).c_str());

  cell = &m_grid[coords->rel.x()][coords->rel.y()];

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
  caops::free_block_drop_visitor op(block,
                                    coords->abs,
                                    mc_resolution,
                                    carena::arena_map_locking::ekALL_HELD);
  op.visit(*cell);

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
              rcppsw::to_string(block->ranchor2D()).c_str());
  } /* for(&e..) */
  return true;

error:
  return false;
} /* verify_block_dist() */

boost::optional<typename random_distributor::coord_search_res_t> random_distributor::
    avail_coord_search(const cds::const_spatial_entity_vector& entities,
                       const rmath::vector2d& block_dim) {
  rmath::vector2z rel;
  rmath::vector2z abs;
  rcppsw::math::rangeu area_xrange(m_grid.index_bases()[0], m_grid.shape()[0]);
  rcppsw::math::rangeu area_yrange(m_grid.index_bases()[1], m_grid.shape()[1]);

  /* -1 because we are working with array indices */
  std::uniform_int_distribution<uint> xdist(area_xrange.lb(),
                                            area_xrange.ub() - 1);
  std::uniform_int_distribution<uint> ydist(area_yrange.lb(),
                                            area_yrange.ub() - 1);
  uint count = 0;

  /*
   * Try to find an available set of relative+absolute coordinates such that if
   * the entity is placed there it will not overlap any other entities in the
   * arena. You only have a finite number of tries, for obvious reasons.
   */
  do {
    uint x = area_xrange.span() > 0
                 ? rng()->uniform(area_xrange.lb(), area_xrange.ub() - 1)
                 : m_grid.index_bases()[0];
    uint y = area_xrange.span() > 0
                 ? rng()->uniform(area_yrange.lb(), area_yrange.ub() - 1)
                 : m_grid.index_bases()[1];
    rel = {x, y};
    abs = {rel.x() + mc_origin.x(), rel.y() + mc_origin.y()};
  } while (std::any_of(entities.begin(), entities.end(), [&](const auto* ent) {
    rmath::vector2d abs_r = rmath::zvec2dvec(abs, mc_resolution.v());
    utils::placement_status_t status;
    if (crepr::entity_dimensionality::ek2D == ent->dimensionality()) {
      status = utils::placement_conflict2D(
          abs_r, block_dim, static_cast<const crepr::entity2D*>(ent));
    } else {
      status = utils::placement_conflict2D(
          abs_r, block_dim, static_cast<const crepr::entity3D*>(ent));
    }
    return status.x_conflict && status.y_conflict && count++ <= kMAX_DIST_TRIES;
  }));
  if (count <= kMAX_DIST_TRIES) {
    return boost::make_optional(coord_search_res_t{rel, abs});
  }
  return boost::none;
} /* avail_coord_search() */

NS_END(block_dist, foraging, cosm);
