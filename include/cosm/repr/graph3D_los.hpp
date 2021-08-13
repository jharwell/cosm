/**
 * \file graph3D_los.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_REPR_GRAPH3D_LOS_HPP_
#define INCLUDE_COSM_REPR_GRAPH3D_LOS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/ds/graph/hgrid3D.hpp"

#include "cosm/repr/base_graph_los.hpp"
#include "cosm/repr/graph3D_view_entity.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class graph3D_los
 * \ingroup repr
 *
 * \brief A representation of the robot's current line-of-sight in 3D.
 */
template <typename TSpecType>
class graph3D_los : public crepr::base_graph_los<
  graph3D_view_entity<rdgraph::hgrid3D<TSpecType>,
                      rdgraph::hgrid3D_view<TSpecType>
                      >
  > {
 public:
  using graph_view_entity_type = graph3D_view_entity<rdgraph::hgrid3D<TSpecType>,
                                                     rdgraph::hgrid3D_view<TSpecType>>;
  using graph_view_type = typename graph_view_entity_type::graph_view_type;
  using graph_view_entity_type::access;
  using graph_view_entity_type::find;

  graph3D_los(const rtypes::type_uuid& c_id,
              const graph_view_type& c_view,
              const rtypes::spatial_dist& c_unit)
      : base_graph_los<graph_view_entity_type>(c_id, c_view, c_unit) {}
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_GRAPH3D_LOS_HPP_ */
