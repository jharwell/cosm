/**
 * \file base_graph_los.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/types/spatial_dist.hpp"
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
                 const rtypes::spatial_dist& c_unit)
      : ER_CLIENT_INIT("cosm.repr.base_graph_los"),
        graph_view_entity_type(c_id, std::move(the_view), c_unit) {}
};

NS_END(repr, cosm);

