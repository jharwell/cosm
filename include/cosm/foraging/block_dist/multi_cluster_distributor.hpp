/**
 * \file multi_cluster_distributor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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
namespace cosm::foraging::block_dist {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class multi_cluster_distributor
 * \ingroup foraging block_dist
 *
 * \brief Distributes a block or set of blocks within the set of specified
 * clusters bounds randomly, using \ref cluster_distributor within
 * each cluster to do the actual distribution. All child clusters/distributors
 * have the same maximum capacity.
 */
class multi_cluster_distributor final : public rer::client<multi_cluster_distributor>,
                                        public base_distributor {
 public:
  /**
   * \param grids The arena grid views for each cluster of size \p capacity.
   * \param arena_grid The grid for the arena map.
   * \param capacity The capacity for all sub-clusters in this multi cluster.
   */

  multi_cluster_distributor(const std::vector<cads::arena_grid::view>& grids,
                            cads::arena_grid* arena_grid,
                            const cspatial::conflict_checker::map_cb_type& conflict_check,
                            const base_distributor::dist_success_cb_type& dist_success,
                            size_t capacity,
                            const rtypes::type_uuid& id_start,
                            rmath::rng* rng);

  /* not copy constructible or copy assignable by default */
  multi_cluster_distributor& operator=(const multi_cluster_distributor&) = delete;
  multi_cluster_distributor(const multi_cluster_distributor&) = delete;

  cfds::block3D_cluster_vectorno block_clustersno(void) override;

  size_t n_configured_clusters(void) const override { return m_dists.size(); }
  size_t n_mapped_clusters(void) const override { return m_dists.size(); }
  size_t capacity(void) const override {
    return m_dists.size() * m_dists[0].capacity();
  }
  size_t size(void) const override RCPPSW_PURE;

  dist_status distribute_block(crepr::sim_block3D* block) override;

 private:
  /* clang-format off */
  std::vector<cluster_distributor> m_dists{};
  /* clang-format on */
};

} /* namespace cosm::foraging::block_dist */

