/**
 * \file block_cluster.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/discretize_ratio.hpp"

#include "cosm/arena/ds/arena_grid.hpp"
#include "cosm/ds/block3D_vector.hpp"
#include "cosm/cosm.hpp"
#include "cosm/repr/grid2D_view_entity.hpp"
#include "cosm/foraging/repr/block_cluster_params.hpp"
#include "cosm/foraging/metrics/block_cluster_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::repr {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_cluster
 * \ingroup foraging repr
 *
 * \brief Represents a cluster of blocks in the arena as an entity for use
 * during block distribution.
 *
 * A cluster is defined as:
 *
 * - The 2D area in which the blocks reside.
 * - The blocks distributed in that area.
 * - The maximum capacity of the cluster.
 */
class block_cluster final : public crepr::grid2D_view_entity<cads::arena_grid,
                                                             cads::arena_grid::const_view>,
                            public metrics::block_cluster_metrics,
                            public rer::client<block_cluster> {
 public:
  using grid2D_view_entity_type = crepr::grid2D_view_entity<cads::arena_grid,
                                                            cads::arena_grid::const_view>;
  explicit block_cluster(const block_cluster_params& params)
      : block_cluster{params.id,
        params.view,
        params.resolution,
        params.capacity} {}

  block_cluster(const rtypes::type_uuid& id,
                const cads::arena_grid::const_view& view,
                const rtypes::discretize_ratio& resolution,
                size_t capacity)
      : grid2D_view_entity_type(id, view, resolution),
        ER_CLIENT_INIT("cosm.foraging.repr.block_cluster"),
        m_capacity(capacity) {}

  /* block cluster metrics */
  size_t n_blocks(void) const override { return blocks().size(); }
  rmath::ranged xrspan(void) const override { return grid2D_view_entity_type::xrspan(); }
  rmath::ranged yrspan(void) const override { return grid2D_view_entity_type::yrspan(); }
  rtypes::type_uuid id(void) const override { return base_entity::id(); }
  rmath::vector2d ranchor2D(void) const override {
    return grid2D_view_entity_type::ranchor2D();
  }

  size_t capacity(void) const { return m_capacity; }
  const cds::block3D_vectorro& blocks(void) const { return m_blocks; }
  void blocks_recalc(void);
  void update_after_drop(const crepr::sim_block3D* dropped);
  void update_after_pickup(const rtypes::type_uuid& pickup_id);

 private:
  /* clang-format off */
  size_t                m_capacity;

  /**
   * \brief The current set of blocks in the cluster. We keep this as a cached
   * value, because it is expensive to compute for large clusters, and can only
   * change if there has been a block pickup/drop. If that has not happened
   * since the last time this was calculated, we can just use the cached value,
   * which is waaayyyyy faster.
   */
  cds::block3D_vectorro m_blocks{};
  /* clang-format on */
};

} /* namespace cosm::foraging::repr */

