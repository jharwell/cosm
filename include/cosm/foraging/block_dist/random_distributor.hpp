/**
 * \file random_distributor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <vector>
#include <boost/optional.hpp>
#include <memory>
#include <limits>

#include "rcppsw/types/discretize_ratio.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/cosm.hpp"
#include "cosm/arena/ds/arena_grid.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/foraging/block_dist/coord_search_policy.hpp"
#include "cosm/spatial/conflict_checker.hpp"

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
  random_distributor(const cads::arena_grid::view& area,
                     cads::arena_grid* arena_grid,
                     const cspatial::conflict_checker::map_cb_type& conflict_check,
                     const dist_success_cb_type& dist_success,
                     rmath::rng* rng);

  random_distributor& operator=(const random_distributor&) = delete;

  /* distributor metrics (stub) */
  size_t n_configured_clusters(void) const override { return 0; }
  size_t n_mapped_clusters(void) const override { return 0; }
  size_t capacity(void) const override { return 0; }
  size_t size(void) const override { return 0; }

  /**
   * \brief Distribution a single block in the arena.
   *
   * \param block The block to distribute.
   *
   * \note Holding \ref arena_map block, grid mutexes necessary to safely call
   * this function in multithreaded contexts (not handled internally).
   *
   * \return \c TRUE if the distribution was successful, \c FALSE otherwise.
   */
   dist_status distribute_block(crepr::sim_block3D* block) override;

  cfds::block3D_cluster_vectorno block_clustersno(void) override { return {}; }

  void coord_search_policy(coord_search_policy policy) { m_search_policy = policy; }

 private:
  struct coord_search_res_t {
    rmath::vector2z rel{};
    rmath::vector2z abs{};
  };

  /**
   * \brief Find coordinates for distribution that are outside the extent of the
   * all specified entities, while also accounting for block size.
   */
  boost::optional<coord_search_res_t> coord_search(const crepr::sim_block3D* block);

  /**
   * \brief Once a block has been distributed, perform distribution sanity
   * checks.
   *
   * - Blocks should not be out of sight.
   * - The cell it was distributed into should refer to it.
   * - No entity should overlap with the block after distribution.
   */
  bool verify_block_dist(const crepr::sim_block3D* block,
                         const cds::cell2D* cell) RCPPSW_PURE;

  boost::optional<coord_search_res_t> coord_search_random(
      const rmath::rangez& c_xrange,
      const rmath::rangez& c_yrange,
      const crepr::sim_block3D* block);


  boost::optional<coord_search_res_t> coord_search_free_cell(
      const rmath::rangez& c_xrange,
      const rmath::rangez& c_yrange,
      const crepr::sim_block3D* block);

  bool coord_conflict_check(const crepr::sim_block3D* block,
                            const rmath::vector2d& drop_loc) const;

  /* clang-format off */
  const rmath::vector2z                         mc_origin;
  const rmath::rangez                           mc_xspan;
  const rmath::rangez                           mc_yspan;
  const cspatial::conflict_checker::map_cb_type mc_conflict_check;
  const dist_success_cb_type                    mc_dist_success;

  enum coord_search_policy                      m_search_policy{coord_search_policy::ekRANDOM};
  cads::arena_grid::view                         m_area;
  /* clang-format on */
};

NS_END(block_dist, foraging, cosm);

