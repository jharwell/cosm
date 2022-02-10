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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <memory>

#include "rcppsw/math/binned_powerlaw_distribution.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector3.hpp"

#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/spatial/conflict_checker.hpp"
#include "cosm/ds/entity_vector.hpp"

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
                       cads::arena_grid* arena_grid,
                       rmath::rng* rng);

  /* not copy constructible or copy assignable by default */
  powerlaw_distributor(const powerlaw_distributor& ) = delete;
  powerlaw_distributor& operator=(const powerlaw_distributor&) = delete;

  /* distributor metrics */
  size_t n_configured_clusters(void) const override { return m_config_clusters; }
  size_t n_mapped_clusters(void) const override {
    return block_clustersro().size();
  }
  size_t capacity(void) const override RCPPSW_PURE;
  size_t size(void) const override RCPPSW_PURE;

  cfds::block3D_cluster_vectorno block_clustersno(void) override;
  dist_status distribute_block(crepr::sim_block3D* block) override;
  /**
   * \brief Computer cluster locations such that no two clusters overlap, and
   * map locations and compositional block distributors into internal data
   * structures.
   *
   * \param c_block_bb The bounding box large enough to hold any block which
   *                   might be distributed in the arena.
   */
  void initialize(carena::base_arena_map* map,
                  const cds::const_spatial_entity_vector& c_entities,
                  const rmath::vector3d& c_block_bb,
                  const cspatial::conflict_checker::map_cb_type& conflict_check,
                  const base_distributor::dist_success_cb_type& dist_success) RCPPSW_COLD;

 private:
  /* clang-format off */
  size_t                                                  m_config_clusters{0};
  std::vector<std::unique_ptr<multi_cluster_distributor>> m_dists{};
  rcppsw::math::binned_powerlaw_distribution              m_pwrdist;
  /* clang-format on */
};

NS_END(block_dist, foraging, cosm);

