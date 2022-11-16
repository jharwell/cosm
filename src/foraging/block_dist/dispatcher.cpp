/**
 * \file dispatcher.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
using cads::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dispatcher::dispatcher(cads::arena_grid* const grid,
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
      mc_cells_xrange(static_cast<size_t>(grid->xdsize() * 0.10),
                      static_cast<size_t>(grid->xdsize() * 0.90)),
      mc_cells_yrange(static_cast<size_t>(grid->ydsize() * 0.10),
                      static_cast<size_t>(grid->ydsize() * 0.90)),
      m_grid(grid),
      m_dist(nullptr) {}

dispatcher::~dispatcher(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool dispatcher::initialize(
    carena::base_arena_map* map,
    const cds::const_spatial_entity_vector& entities,
    const rmath::vector3d& block_bb,
    const cspatial::conflict_checker::map_cb_type& conflict_check,
    const base_distributor::dist_success_cb_type& dist_success,
    rmath::rng* rng) {
  /* clang-format off */
  cads::arena_grid::view arena = m_grid->layer<arena_grid::kCell>()->subgrid(
      rmath::vector2z(mc_cells_xrange.lb(),
                      mc_cells_yrange.lb()),
      rmath::vector2z(mc_cells_xrange.ub(),
                      mc_cells_yrange.ub()));
  auto cluster_xwidth = static_cast<size_t>(m_grid->xdsize() * 0.15);
  auto cluster_ywidth = static_cast<size_t>(m_grid->ydsize() * 0.15);

  /*
   *  +/- 1 is to account for rounding errors when computing the cluster X/Y
   * width.
   */
  auto left_ll = rmath::vector2z(mc_cells_xrange.lb(),
                                 mc_cells_yrange.lb());
  auto left_ur = rmath::vector2z(mc_cells_xrange.lb() + cluster_xwidth - 1,
                                 mc_cells_yrange.ub());
  auto right_ll = rmath::vector2z(mc_cells_xrange.ub() - cluster_xwidth + 1,
                                  mc_cells_yrange.lb());
  auto right_ur = rmath::vector2z(mc_cells_xrange.ub(),
                                  mc_cells_yrange.ub());

  /*
   * Top and bottom cluster (LL,UR) X coordinates need to account for the width
   * of the left and right cluster spatial X extents to ensure no nothing
   * overlaps.
   */
  auto bottom_ll = rmath::vector2z(mc_cells_xrange.lb() + cluster_xwidth + 1,
                                   mc_cells_yrange.lb());
  auto bottom_ur = rmath::vector2z(mc_cells_xrange.ub() - cluster_xwidth - 1,
                                   mc_cells_yrange.lb() + cluster_ywidth);
  auto top_ll = rmath::vector2z(mc_cells_xrange.lb() + cluster_xwidth + 1,
                                mc_cells_yrange.ub() - cluster_ywidth);
  auto top_ur = rmath::vector2z(mc_cells_xrange.ub() - cluster_xwidth - 1,
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
        conflict_check,
        dist_success,
        std::numeric_limits<size_t>::max(),
        rng);
  } else if (kDistSingleSrc == mc_dist_type) {
    cads::arena_grid::view area = m_grid->layer<arena_grid::kCell>()->subgrid(right_ll,
                                                                             right_ur);
    m_dist = std::make_unique<cluster_distributor>(
        rtypes::type_uuid(0),
        area,
        m_grid,
        conflict_check,
        dist_success,
        std::numeric_limits<size_t>::max(),
        rng);
  } else if (kDistDualSrc == mc_dist_type) {
    cads::arena_grid::view area_l = m_grid->layer<arena_grid::kCell>()->subgrid(left_ll,
                                                                               left_ur);
    cads::arena_grid::view area_r = m_grid->layer<arena_grid::kCell>()->subgrid(right_ll,
                                                                               right_ur);
    std::vector<cads::arena_grid::view> areas{area_l, area_r};
    m_dist = std::make_unique<multi_cluster_distributor>(
        areas,
        m_grid,
        conflict_check,
        dist_success,
        std::numeric_limits<size_t>::max(),
        rtypes::type_uuid(0),
        rng);
  } else if (kDistQuadSrc == mc_dist_type) {
    /*
     * Quad source is a tricky distribution to use with static caches, so we
     * have to tweak the block cluster centers in tandem with the cache
     * locations to ensure that no segfaults result from cache/cache or
     * cache/cluster overlap. See FORDYCA#581, COSM#34.
     */
    cads::arena_grid::view area_l = m_grid->layer<arena_grid::kCell>()->subgrid(left_ll,
                                                                               left_ur);
    cads::arena_grid::view area_r = m_grid->layer<arena_grid::kCell>()->subgrid(right_ll,
                                                                               right_ur);
    cads::arena_grid::view area_b = m_grid->layer<arena_grid::kCell>()->subgrid(bottom_ll,
                                                                               bottom_ur);
    cads::arena_grid::view area_u = m_grid->layer<arena_grid::kCell>()->subgrid(top_ll,
                                                                               top_ur);

    ER_ASSERT(!cads::arena_grid::has_overlap(area_l, area_b) &&
              !cads::arena_grid::has_overlap(area_l, area_u),
              "Left block cluster overlaps with top or bottom");
    ER_ASSERT(!cads::arena_grid::has_overlap(area_r, area_b) &&
              !cads::arena_grid::has_overlap(area_r, area_u),
              "Right block cluster overlaps with top or bottom");
    std::vector<cads::arena_grid::view> areas{area_l, area_r, area_b, area_u};
    m_dist = std::make_unique<multi_cluster_distributor>(
        areas,
        m_grid,
        conflict_check,
        dist_success,
        std::numeric_limits<size_t>::max(),
        rtypes::type_uuid(0),
        rng);
  } else if (kDistPowerlaw == mc_dist_type) {
    auto p = std::make_unique<powerlaw_distributor>(&mc_config.powerlaw,
                                                    m_grid,
                                                    rng);
    p->initialize(map, entities, block_bb, conflict_check, dist_success);
    m_dist = std::move(p);
  }
  /* clang-format on */
  return true;
} /* initialize() */

dist_status dispatcher::distribute_block(crepr::sim_block3D* block) {
  return m_dist->distribute_block(block);
} /* distribute_block() */

dist_status dispatcher::distribute_blocks(cds::block3D_vectorno& blocks) {
  return m_dist->distribute_blocks(blocks, mc_config.strict_success);
} /* distribute_block() */

NS_END(block_dist, foraging, cosm);
