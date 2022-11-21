/**
 * \file neighbors_update.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <utility>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/ds/graph/hgrid3D.hpp"
#include "rcppsw/ds/graph/hgrid3D_view.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::controller::operations {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct neighbors_update
 * \ingroup controller operations
 *
 * \brief Functor to update the neighbors of a robot each timestep as it moves.
 *
 * Neighbors are selected for inclusion for a given robot if they fall within a
 * specified minimum distance, and up to a maximum count.
 */
template <typename TController,
          typename TSwarmSrc,
          typename TLOS>
class neighbors_update final
    : public rer::client<neighbors_update<TController,
                                          TSrcGraph,
                                          TLOS>> {
 public:
  using los_type = TLOS;

  neighbors_update(const rspatial::euclidean_dist& max_dist)
      : ER_CLIENT_INIT("cosm.controller.operations.neighbors_update"),
        mc_max_dist(max_dist) {}

  neighbors_update(const neighbors_update&) = delete;
  neighbors_update& operator=(const neighbors_update&) = delete;

  void operator()(TController* const controller,
                  const rmath::vector3d& pos) const {

    auto max_dist = rtypes::manhattan_dist(static_cast<int>(
        std::round(controller->los_dim() / mc_unit_dim.v())));
    neighbors_set(controller, center, max_dist);
  }

 private:
  /**
   * \brief Set the LOS of a robot as it moves within a graph.
   *
   * \todo This should eventually be replaced by a calculation of a robot's LOS
   * by the robot, probably using on-board cameras.
   */
  void neighbors_set(TController* const controller,
                     const typename TSrcGraph::vertex_descriptor& center,
                     const rtypes::manhattan_dist& max_dist) const {
    auto los = std::make_unique<los_type>(controller->entity_id(),
                                          mc_graph->subgraph(center, max_dist),
                                          mc_unit_dim);
    controller->perception()->los(std::move(los));
  }

  /* clang-format off */
  const TSrcGraph*               mc_graph;
  const rspatial::euclidean_dist mc_max_dist;
  /* clang-format on */
};

} /* namespace cosm::controller::operations */
