/**
 * \file random_distributor.hpp
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

#ifndef INCLUDE_COSM_FORAGING_BLOCK_DIST_RANDOM_DISTRIBUTOR_HPP_
#define INCLUDE_COSM_FORAGING_BLOCK_DIST_RANDOM_DISTRIBUTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <vector>
#include <boost/optional.hpp>
#include <memory>

#include "rcppsw/types/discretize_ratio.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/cosm.hpp"
#include "cosm/ds/arena_grid.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/foraging/block_dist/coord_search_policy.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm);

namespace repr {
class multicell_entity;
} // namespace repr

NS_START(foraging, block_dist);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class random_distributor
 * \ingroup foraging block_dist
 *
 * \brief Distributes a set of blocks randomly within a specified 2D area, such
 * that no blocks overlap with each other or other entities already present in
 * the arena (nest, cache, etc.).
 */
class random_distributor : public rer::client<random_distributor>,
                           public base_distributor {
 public:
  random_distributor(const cds::arena_grid::view& arena,
                     cds::arena_grid* arena_grid,
                     rmath::rng* rng);

  random_distributor& operator=(const random_distributor&) = delete;

  /**
   * \brief Distribution a single block in the arena.
   *
   * \param block The block to distribute.
   * \param entities Entities that need to be avoided during distribution.
   *
   * \note Holding \ref arena_map block, grid mutexes necessary to safely call
   * this function in multithreaded contexts (not handled internally).
   *
   * \return \c TRUE if the distribution was successful, \c FALSE otherwise.
   */
   dist_status distribute_block(crepr::base_block3D* block,
                                cds::const_spatial_entity_vector& entities) override;

  cfds::block3D_cluster_vector block_clusters(void) const override { return {}; }

  void coord_search_policy(coord_search_policy policy) { m_search_policy = policy; }

 private:
  struct coord_search_res_t {
    rmath::vector2z rel{};
    rmath::vector2z abs{};
  };

  /**
   * \brief Find coordinates for distribution that are outside the extent of the
   * all specified entities, while also accounting for block size.
   *
   * \param entities The entities to avoid.
   */
  boost::optional<coord_search_res_t> coord_search(
      const cds::const_spatial_entity_vector& c_entities,
      const rmath::vector2d& c_block_dim);

  /**
   * \brief Once a block has been distributed, perform distribution sanity
   * checks.
   *
   * - Blocks should not be out of sight.
   * - The cell it was distributed into should refer to it.
   * - No entity should overlap with the block after distribution.
   */
  bool verify_block_dist(const crepr::base_block3D* block,
                         const cds::const_spatial_entity_vector& entities,
                         const cds::cell2D* cell) RCSW_PURE;

  boost::optional<coord_search_res_t> coord_search_random(
      const rcppsw::math::rangez& c_xrange,
      const rcppsw::math::rangez& c_yrange,
      const cds::const_spatial_entity_vector& c_entities,
      const rmath::vector2d& c_block_dim);

  boost::optional<coord_search_res_t> coord_search_free_cell(
      const rcppsw::math::rangez& c_xrange,
      const rcppsw::math::rangez& c_yrange,
      const cds::const_spatial_entity_vector& c_entities,
      const rmath::vector2d& c_block_dim);

  bool coord_conflict_check(const rmath::vector2z& c_coord,
                            const cds::const_spatial_entity_vector& c_entities,
                            const rmath::vector2d& c_block_dim) const;
  /* clang-format off */
  const rmath::vector2z          mc_origin;
  const rmath::rangeu            mc_xspan;
  const rmath::rangeu            mc_yspan;

  enum coord_search_policy       m_search_policy{coord_search_policy::ekRANDOM};
  cds::arena_grid::view          m_area;
  /* clang-format on */
};

NS_END(block_dist, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_BLOCK_DIST_RANDOM_DISTRIBUTOR_HPP_ */
