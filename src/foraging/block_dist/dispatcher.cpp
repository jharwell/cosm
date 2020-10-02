/**
 * \file dispatcher.cpp
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
#include "cosm/foraging/block_dist/dispatcher.hpp"

#include <limits>

#include "cosm/foraging/block_dist/cluster_distributor.hpp"
#include "cosm/foraging/block_dist/multi_cluster_distributor.hpp"
#include "cosm/foraging/block_dist/powerlaw_distributor.hpp"
#include "cosm/foraging/block_dist/random_distributor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);
using cds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dispatcher::dispatcher(cds::arena_grid* const grid,
                       const rtypes::discretize_ratio& resolution,
                       const config::block_dist_config* const config)
    : ER_CLIENT_INIT("cosm.foraging.block_dist.dispatcher"),
      mc_resolution(resolution),
      mc_config(*config),
      mc_dist_type(config->dist_type),
      /*
       * All sourced distributions can't occupy more than 70% of either the X or
       * Y dimension in order to avoid physics engine errors in ARGoS near arena
       * walls. See COSM#34.
       */
      mc_cells_xrange(static_cast<size_t>(grid->xdsize() * 0.15),
                      static_cast<size_t>(grid->xdsize() * 0.85)),
      mc_cells_yrange(static_cast<size_t>(grid->ydsize() * 0.15),
                      static_cast<size_t>(grid->ydsize() * 0.85)),
      m_grid(grid),
      m_dist(nullptr) {}

dispatcher::~dispatcher(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool dispatcher::initialize(const cds::const_spatial_entity_vector& entities,
                            const rmath::vector3d& block_bb,
                            rmath::rng* rng) {
  /* clang-format off */
  cds::arena_grid::view arena = m_grid->layer<arena_grid::kCell>()->subgrid(
      rmath::vector2z(mc_cells_xrange.lb(),
                      mc_cells_yrange.lb()),
      rmath::vector2z(mc_cells_xrange.ub(),
                      mc_cells_yrange.ub()));
  auto left_ll = rmath::vector2z(mc_cells_xrange.lb(),
                                 mc_cells_yrange.lb());
  auto left_ur = rmath::vector2z(static_cast<size_t>(mc_cells_xrange.ub() * 0.25 / 0.85),
                                 mc_cells_yrange.ub());
  auto right_ll = rmath::vector2z(static_cast<size_t>(mc_cells_xrange.ub() * 0.75 / 0.85),
                                  mc_cells_yrange.lb());
  auto right_ur = rmath::vector2z(mc_cells_xrange.ub(),
                                  mc_cells_yrange.ub());
  auto bottom_ll = rmath::vector2z(mc_cells_xrange.lb(),
                                   mc_cells_yrange.lb());
  auto bottom_ur = rmath::vector2z(mc_cells_xrange.ub(),
                                   static_cast<size_t>(mc_cells_yrange.ub() * 0.25 / 0.85));
  auto top_ll = rmath::vector2z(mc_cells_xrange.lb(),
                                static_cast<size_t>(mc_cells_yrange.ub() * 0.75 / 0.85));
  auto top_ur = rmath::vector2z(mc_cells_xrange.ub(),
                                mc_cells_yrange.ub());
  if (kDistRandom == mc_dist_type) {
    /*
     * Special case: random distribution = one cluster which covers whole
     * distributable area.
     */
    m_dist = std::make_unique<cluster_distributor>(
        rtypes::type_uuid(0),
        arena,
        m_grid,
        std::numeric_limits<size_t>::max(),
        rng);
  } else if (kDistSingleSrc == mc_dist_type) {
    cds::arena_grid::view area = m_grid->layer<arena_grid::kCell>()->subgrid(right_ll,
                                                                             right_ur);
    m_dist = std::make_unique<cluster_distributor>(
        rtypes::type_uuid(0),
        arena,
        m_grid,
        std::numeric_limits<size_t>::max(),
        rng);
  } else if (kDistDualSrc == mc_dist_type) {
    cds::arena_grid::view area_l = m_grid->layer<arena_grid::kCell>()->subgrid(left_ll,
                                                                               left_ur);
    cds::arena_grid::view area_r = m_grid->layer<arena_grid::kCell>()->subgrid(right_ll,
                                                                               right_ur);
    std::vector<cds::arena_grid::view> areas{area_l, area_r};
    m_dist = std::make_unique<multi_cluster_distributor>(
        areas,
        m_grid,
        std::numeric_limits<size_t>::max(),
        rng);
  } else if (kDistQuadSrc == mc_dist_type) {
    /*
     * Quad source is a tricky distribution to use with static caches, so we
     * have to tweak the block cluster centers in tandem with the cache
     * locations to ensure that no segfaults result from cache/cache or
     * cache/cluster overlap. See FORDYCA#581, COSM#34.
     */
    cds::arena_grid::view area_l = m_grid->layer<arena_grid::kCell>()->subgrid(left_ll,
                                                                               left_ur);
    cds::arena_grid::view area_r = m_grid->layer<arena_grid::kCell>()->subgrid(right_ll,
                                                                               right_ur);
    cds::arena_grid::view area_b = m_grid->layer<arena_grid::kCell>()->subgrid(bottom_ll,
                                                                               bottom_ur);
    cds::arena_grid::view area_u = m_grid->layer<arena_grid::kCell>()->subgrid(top_ll,
                                                                               top_ur);
    std::vector<cds::arena_grid::view> areas{area_l, area_r, area_b, area_u};
    m_dist = std::make_unique<multi_cluster_distributor>(
        areas,
        m_grid,
        std::numeric_limits<size_t>::max(),
        rng);
  } else if (kDistPowerlaw == mc_dist_type) {
    auto p = std::make_unique<powerlaw_distributor>(&mc_config.powerlaw,
                                                    m_grid,
                                                    rng);
    p->initialize(entities, block_bb),
    m_dist = std::move(p);
  }
  /* clang-format on */
  return true;
} /* initialize() */

dist_status dispatcher::distribute_block(crepr::base_block3D* block,
                                         cds::const_spatial_entity_vector& entities) {
  return m_dist->distribute_block(block, entities);
} /* distribute_block() */

dist_status dispatcher::distribute_blocks(cds::block3D_vectorno& blocks,
                                          cds::const_spatial_entity_vector& entities) {
  return m_dist->distribute_blocks(blocks, entities, mc_config.strict_success);
} /* distribute_block() */

NS_END(block_dist, foraging, cosm);
