/**
 * \file multi_cluster_distributor.hpp
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

#ifndef INCLUDE_COSM_FORAGING_BLOCK_DIST_MULTI_CLUSTER_DISTRIBUTOR_HPP_
#define INCLUDE_COSM_FORAGING_BLOCK_DIST_MULTI_CLUSTER_DISTRIBUTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <memory>

#include "cosm/foraging/block_dist/cluster_distributor.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class multi_cluster_distributor
 * \ingroup foraging block_dist
 *
 * \brief Distributes a block or set of blocks within the set of specified
 * clusters bounds randomly, using \ref cluster_distributor within
 * each cluster to do the actual distribution.
 */
class multi_cluster_distributor final : public rer::client<multi_cluster_distributor>,
                                        public base_distributor {
 public:
  multi_cluster_distributor(std::vector<cds::arena_grid::view>& grids,
                            rtypes::discretize_ratio resolution,
                            uint maxsize,
                            rmath::rng* rng_in);

  /* not copy constructible or copy assignable by default */
  multi_cluster_distributor& operator=(const multi_cluster_distributor&) = delete;
  multi_cluster_distributor(const multi_cluster_distributor&) = delete;

  cfds::block3D_cluster_vector block_clusters(void) const override;
  bool distribute_block(crepr::base_block3D* block,
                        cds::const_spatial_entity_vector& entities) override;

 private:
  /* clang-format off */
  std::vector<cluster_distributor> m_dists{};
  /* clang-format on */
};

NS_END(block_dist, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_BLOCK_DIST_MULTI_CLUSTER_DISTRIBUTOR_HPP_ */
