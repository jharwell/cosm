/**
 * \file cluster_distributor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <boost/variant.hpp>

#include "cosm/foraging/block_dist/random_distributor.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/foraging/repr/block_cluster.hpp"
#include "cosm/spatial/common/conflict_checker.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::block_dist {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cluster_distributor
 * \ingroup foraging block_dist
 *
 * \brief Distributes a block or set of blocks within the specified cluster
 * bounds randomly, using \ref random_distributor.
 */
class cluster_distributor final : public rer::client<cluster_distributor>,
                                  public base_distributor {
 public:
  cluster_distributor(const rtypes::type_uuid& id,
                      const cads::arena_grid::view& view,
                      cads::arena_grid* arena_grid,
                      const cspatial::conflict_checker::map_cb_type& conflict_check,
                      const base_distributor::dist_success_cb_type& dist_success,
                      size_t capacity,
                      rmath::rng* rng);
  ~cluster_distributor(void) override = default;

  /* not copy-constructible or copy-assignable by default */
  cluster_distributor& operator=(const cluster_distributor& ) = delete;

  /* distributor metrics */
  size_t n_configured_clusters(void) const override { return 1; }
  size_t n_mapped_clusters(void) const override { return 1; }
  size_t capacity(void) const override { return m_clust.capacity(); }
  size_t size(void) const override { return m_clust.n_blocks(); }

  dist_status distribute_block(crepr::sim_block3D* block) override;
  dist_status distribute_blocks(cds::block3D_vectorno& blocks,
                                bool strict_success) override;
  cfds::block3D_cluster_vectorno block_clustersno(void) override;
  rmath::vector2d ranchor2D(void) const { return m_clust.ranchor2D(); }
  rmath::vector2z danchor2D(void) const { return m_clust.danchor2D(); }
  void coord_search_policy(const coord_search_policy& policy) {
    m_impl.coord_search_policy(policy);
  }

 private:
  /* clang-format off */
  cfrepr::block_cluster m_clust;
  random_distributor    m_impl;
  /* clang-format on */
};

} /* namespace cosm::foraging::block_dist */

