/**
 * \file graph3D_los.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>

#include "rcppsw/ds/graph/hgrid3D.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/repr/base_graph_los.hpp"
#include "cosm/repr/graph3D_view_entity.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {

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
                                            rdgraph::hgrid3D_view<TSpecType>>> {
 public:
  using graph_view_entity_type =
      graph3D_view_entity<rdgraph::hgrid3D<TSpecType>,
                          rdgraph::hgrid3D_view<TSpecType>>;
  using graph_view_type = typename graph_view_entity_type::graph_view_type;
  using graph_view_entity_type::access;
  using graph_view_entity_type::find;

  graph3D_los(const rtypes::type_uuid& c_id,
              graph_view_type&& the_view,
              const rspatial::euclidean_dist& c_unit)
      : base_graph_los<graph_view_entity_type>(c_id,
                                               std::move(the_view),
                                               c_unit) {}
};

} /* namespace cosm::repr */
