/**
 * \file base_graph_los.hpp
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

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_graph_los
 * \ingroup repr
 *
 * \brief A representation of the robot's current line-of-sight as it moves
 * through a weighted graph of some kind. The robot is only able to update its
 * internal state based on the information present in the per-timestep updates
 * to this object.
 *
 * The LOS for a robot does not have a regular shape per-se, as it depends on
 * the robot's current position and what vertices are within the LOS radius.
 */
template <typename TGraphViewEntityType>
class base_graph_los : public rer::client<base_graph_los<TGraphViewEntityType>>,
                       public TGraphViewEntityType {
 public:
  using graph_view_entity_type = TGraphViewEntityType;
  using graph_view_type = typename graph_view_entity_type::graph_view_type;

  base_graph_los(const rtypes::type_uuid& c_id,
                 graph_view_type&& the_view,
                 const rspatial::euclidean_dist& c_unit)
      : ER_CLIENT_INIT("cosm.repr.base_graph_los"),
        graph_view_entity_type(c_id, std::move(the_view), c_unit) {}
};

NS_END(repr, cosm);
