/**
 * \file powerlaw_distributor.hpp
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

#ifndef INCLUDE_COSM_FORAGING_BLOCK_DIST_POWERLAW_DISTRIBUTOR_HPP_
#define INCLUDE_COSM_FORAGING_BLOCK_DIST_POWERLAW_DISTRIBUTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <memory>
#include <boost/optional.hpp>

#include "rcppsw/math/binned_powerlaw_distribution.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/foraging/repr/block_cluster_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::config { struct powerlaw_dist_config; }

NS_START(cosm, foraging, block_dist);

class multi_cluster_distributor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class powerlaw_distributor
 * \ingroup foraging block_dist
 *
 * \brief Distributes a block, or set of blocks, within the arena as randomly
 * placed clusters with sizes ranging [minsize, maxsize], with a power law based
 * stride of 2^x between.
 *
 * - Blocks are assumed to be the same size as arena resolution (this is not
 *   checked).
 */
class powerlaw_distributor final : public rer::client<powerlaw_distributor>,
                                   public base_distributor {
 public:
  powerlaw_distributor(const config::powerlaw_dist_config* config,
                       cds::arena_grid* arena_grid,
                       rmath::rng* rng);

  /* not copy constructible or copy assignable by default */
  powerlaw_distributor(const powerlaw_distributor& ) = delete;
  powerlaw_distributor& operator=(const powerlaw_distributor&) = delete;

  cfds::block3D_cluster_vector block_clusters(void) const override;
  dist_status distribute_block(crepr::base_block3D* block,
                               cds::const_spatial_entity_vector& entities) override;

  /**
   * \brief Computer cluster locations such that no two clusters overlap, and
   * map locations and compositional block distributors into internal data
   * structures.
   *
   * \param entities The entities to avoid during cluster mapping.
   * \param block_bb The bounding box large enough to hold any block which might
   *                 be distributed in the arena.
   *
   * \return \c TRUE iff clusters were mapped successfully, \c FALSE otherwise.
   */
  bool initialize(const cds::const_spatial_entity_vector& entities,
                  const rmath::vector3d& block_bb);

 private:
  struct cluster_placement {
    rtypes::type_uuid     id;
    cds::arena_grid::view view;
    rmath::rangez         xrange;
    rmath::rangez         yrange;
    size_t                capacity;
  };

  using cluster_placements = std::vector<cluster_placement>;

  /**
   * \brief Assign cluster centers randomly, with the only restriction that the
   * edges of each cluster are within the boundaries of the arena.
   *
   * \param clust_sizes Vector of powers of 2 for the cluster sizes.
   */
  cluster_placements cluster_placements_guess(const std::vector<size_t>& clust_sizes,
                                              const rmath::vector3d& block_bb);

  /**
   * \brief Verify that no cluster placements cause overlap, after guessing
   * initial locations. No overlap with OTHER existing entities in the arena is
   * checked.
   *
   * \param placements Possible list of cluster placements.
   *
   * \return \c TRUE if the cluster distribute is valid, \c FALSE otherwise.
   */
  bool cluster_placements_check(const cluster_placements& placements,
                                const cds::const_spatial_entity_vector& entities) RCSW_PURE;

  /**
   * \brief Perform a "guess and check" cluster placement until you get a
   * distribution without overlap, or \ref kMAX_DIST_TRIES is exceeded,
   * whichever happens first.
   *
   * Cluster sizes are drawn from the internal power law distribution.
   */
  boost::optional<cluster_placements> cluster_placements_calc(
      const cds::const_spatial_entity_vector& entities,
      const rmath::vector3d& block_bb,
      size_t n_clusters);

  /* clang-format off */
  size_t                                                  m_n_clusters{0};
  std::vector<std::unique_ptr<multi_cluster_distributor>> m_dists{};
  rcppsw::math::binned_powerlaw_distribution              m_pwrdist;
  /* clang-format on */
};

NS_END(block_dist, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_BLOCK_DIST_POWERLAW_DISTRIBUTOR_HPP_ */
