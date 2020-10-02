/**
 * \file block_cluster.hpp
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

#ifndef INCLUDE_COSM_FORAGING_REPR_BLOCK_CLUSTER_HPP_
#define INCLUDE_COSM_FORAGING_REPR_BLOCK_CLUSTER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/discretize_ratio.hpp"

#include "cosm/ds/arena_grid.hpp"
#include "cosm/ds/block3D_vector.hpp"
#include "cosm/cosm.hpp"
#include "cosm/repr/grid_view_entity.hpp"
#include "cosm/foraging/repr/block_cluster_params.hpp"
#include "cosm/foraging/metrics/block_cluster_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, repr);

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
class block_cluster final : public crepr::grid_view_entity<cds::arena_grid::const_view>,
  public metrics::block_cluster_metrics,
                            public rer::client<block_cluster> {
 public:
  explicit block_cluster(const block_cluster_params& params)
      : block_cluster{params.id,
        params.view,
        params.resolution,
        params.capacity} {}

  block_cluster(const rtypes::type_uuid& id,
                const cds::arena_grid::const_view& view,
                const rtypes::discretize_ratio& resolution,
                size_t capacity)
      : grid_view_entity<cds::arena_grid::const_view>(id, view, resolution),
        ER_CLIENT_INIT("cosm.foraging.repr.block_cluster"),
        m_capacity(capacity) {}

  /* block cluster metrics */
  size_t n_blocks(void) const override { return blocks().size(); }
  rmath::ranged xrspan(void) const override { return grid_view_entity::xrspan(); }
  rmath::ranged yrspan(void) const override { return grid_view_entity::yrspan(); }
  rtypes::type_uuid id(void) const override { return base_entity::id(); }
  rmath::vector2d ranchor2D(void) const override {
    return grid_view_entity::ranchor2D();
  }

  size_t capacity(void) const { return m_capacity; }
  cds::block3D_vectorro blocks(void) const RCSW_PURE;


 private:
  /* clang-format off */
  size_t m_capacity;
  /* clang-format on */
};

NS_END(repr, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_REPR_BLOCK_CLUSTER_HPP_ */
