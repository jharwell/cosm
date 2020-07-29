/**
 * \file powerlaw_cluster_placer.hpp
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

#ifndef INCLUDE_COSM_FORAGING_BLOCK_DIST_POWERLAW_CLUSTER_PLACER_HPP_
#define INCLUDE_COSM_FORAGING_BLOCK_DIST_POWERLAW_CLUSTER_PLACER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/math/range.hpp"
#include "rcppsw/math/rng.hpp"

#include "cosm/ds/arena_grid.hpp"
#include "cosm/ds/entity_vector.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class powerlaw_cluster_placer
 * \ingroup foraging block_dist
 *
 * \brief Calculates the placement of N clusters of sizes drawn from a
 * configured powerlaw distribution in the area during initialization.
 */

class powerlaw_cluster_placer : public rer::client<powerlaw_cluster_placer> {
 public:
  struct placement {
    rtypes::type_uuid     id;
    cds::arena_grid::view view;
    rmath::rangez         xrange;
    rmath::rangez         yrange;
    size_t                capacity;
  };
  using placements = std::vector<placement>;

  powerlaw_cluster_placer(cds::arena_grid* grid,
                          const rmath::vector3d& c_block_bb,
                          size_t n_attempts,
                          rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  powerlaw_cluster_placer(const powerlaw_cluster_placer&) = delete;
  const powerlaw_cluster_placer& operator=(const powerlaw_cluster_placer&) = delete;
  powerlaw_cluster_placer(powerlaw_cluster_placer&&) = delete;
  powerlaw_cluster_placer& operator=(powerlaw_cluster_placer&&) = delete;

  /**
   * \brief Perform a "guess and check" cluster placement until you get a
   * distribution without overlap, or \ref kMAX_DIST_TRIES is exceeded,
   * whichever happens first.
   *
   * Cluster sizes are drawn from the internal power law distribution.
   */
  placements operator()(
      const cds::const_spatial_entity_vector& c_entities,
      const std::vector<size_t>& c_sizes) RCSW_COLD;

 private:
  /**
   * \brief Assign cluster center randomly, with the following restrictions:
   *
   * - Cluster edges are inside the the boundaries of the arena.
   *
   * \param size Power of 2 size of cluster.
   * \param block_bb The bounding box for all blocks in the arena.
   */
  placement placement_guess(const rtypes::type_uuid& c_id,
                            size_t size,
                            const rmath::vector3d& c_block_bb) RCSW_COLD;

  /**
   * \brief Verify that the guessed cluster placement does not overlap with:
   *
   * - Already placed clusters
   * - Any existing entities in the arena, which at this point during
   *   initialization should only be nests and (maybe) caches.
   *
   * \param c_guess Current cluster placement to check.
   * \param c_placed List of already validated cluster placements.
   * \param c_entities List of non-cluster entities check the guessed cluster
   *                   placement against, in addition to the already placed
   *                   clusters.
   *
   * \return \c TRUE if the cluster distribute is valid, \c FALSE otherwise.
   */
  bool placement_check(const placement& c_guess,
                       const placements& c_placed,
                       const cds::const_spatial_entity_vector& c_entities) const RCSW_ATTR(cold, pure);

 private:
  /* clang-format off */
  const size_t           mc_n_attempts;
  const rmath::vector3d  mc_block_bb;

  cds::arena_grid*       m_grid;
  rmath::rng*            m_rng;
  /* clang-format on */
};

NS_END(block_dist, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_BLOCK_DIST_POWERLAW_CLUSTER_PLACER_HPP_ */
