/**
 * \file graph_los_update.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_CONTROLLER_OPERATIONS_GRAPH_LOS_UPDATE_HPP_
#define INCLUDE_COSM_CONTROLLER_OPERATIONS_GRAPH_LOS_UPDATE_HPP_

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
#include "rcppsw/types/spatial_dist.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, controller, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct graph_los_update
 * \ingroup controller operations
 *
 * \brief Functor to update robot LOS each timestep as it moves within a graph.
 */
template <typename TController,
          typename TSrcGraph,
          typename TLOS>
class graph_los_update final
    : public rer::client<graph_los_update<TController,
                                          TSrcGraph,
                                          TLOS>> {
 public:
  using los_type = TLOS;

  graph_los_update(const TSrcGraph* graph,
                   const rtypes::spatial_dist& unit_dim)
      : ER_CLIENT_INIT("cosm.support.graph_los_update"),
        mc_graph(graph),
        mc_unit_dim(unit_dim) {}

  /*
   * \todo Ideally these would be deleted, but emplace() does not seem to do
   * what I think it does (i.e. construct an object in place without a need for
   * a copy constructor), so it is defaulted instead.
   */
  graph_los_update(const graph_los_update&) = default;
  graph_los_update& operator=(const graph_los_update&) = delete;

  void operator()(TController* const controller,
                  typename TSrcGraph::vertex_descriptor center) const {
    ER_ASSERT(rmath::is_multiple_of(controller->los_dim(),
                                    mc_unit_dim.v()),
              "LOS dimension (%f) not an even multiple of graph resolution (%f)",
              controller->los_dim(),
              mc_unit_dim.v());

    auto max_dist = rtypes::manhattan_dist(static_cast<int>(controller->los_dim() / mc_unit_dim.v()));
    graph_los_set(controller, center, max_dist);
  }

 private:
  /**
   * \brief Set the LOS of a robot as it moves within a graph.
   *
   * \todo This should eventually be replaced by a calculation of a robot's LOS
   * by the robot, probably using on-board cameras.
   */
  void graph_los_set(TController* const controller,
                     const typename TSrcGraph::vertex_descriptor& center,
                     const rtypes::manhattan_dist& max_dist) const {
    auto los = std::make_unique<los_type>(controller->entity_id(),
                                          mc_graph->subgraph(center, max_dist),
                                          mc_unit_dim);
    controller->perception()->los(std::move(los));
  }

  /* clang-format off */
  const TSrcGraph*           mc_graph;
  const rtypes::spatial_dist mc_unit_dim;
  /* clang-format on */
};

NS_END(operations, controller, cosm);

#endif /* INCLUDE_COSM_CONTROLLER_OPERATIONS_GRAPH_LOS_UPDATE_HPP_ */
