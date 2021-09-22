/**
 * \file base_graph_view_entity.hpp
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

#ifndef INCLUDE_COSM_REPR_BASE_GRAPH_VIEW_ENTITY_HPP_
#define INCLUDE_COSM_REPR_BASE_GRAPH_VIEW_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>

#include "rcppsw/types/spatial_dist.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_graph_view_entity
 * \ingroup repr
 *
 * \brief Representation of an entity within the arena which is a view from a
 * graph. This representation makes it much easier to query and obtain summary
 * information about the underlying subgraph without having to iterate over
 * vertices/edges, by providing a "bounding-box" interface to the subgraph.
 *
 * It has the following characteristics:
 *
 * - Does not "exist" in the sense that it is not detectable by robots.
 * - Has no concept of movability/immovability (it is abstract).
 */
template <typename TGraphType, typename TGraphViewType>
class base_graph_view_entity : public rpdecorator::decorator<TGraphViewType> {
 public:
  using graph_type = TGraphType;
  using graph_view_type = TGraphViewType;
  using vertex_coord_type = typename TGraphViewType::vertex_coord_type;
  using vertex_property_type = typename TGraphViewType::vertex_property_type;
  using edge_property_type = typename TGraphViewType::edge_property_type;
  using vertex_descriptor = typename TGraphViewType::vertex_descriptor;
  using edge_descriptor = typename TGraphViewType::edge_descriptor;

  using decorator_type = rpdecorator::decorator<graph_view_type>;
  using decoratee_type = typename decorator_type::decoratee_type;
  using decorator_type::decoratee;

  base_graph_view_entity(graph_view_type&& view,
                         const rtypes::spatial_dist& unit)
      : rpdecorator::decorator<graph_view_type>(std::move(view)),
        mc_unit(unit) {}

  virtual ~base_graph_view_entity(void) = default;

  RCPPSW_DECORATE_DECL(vertex_iterator);

  RCPPSW_DECORATE_DECLDEF(find, const);
  RCPPSW_DECORATE_DECLDEF(vertices, const);
  RCPPSW_DECORATE_DECLDEF(n_vertices, const);
  RCPPSW_DECORATE_DECLDEF(adjacent_vertices, const);
  RCPPSW_DECORATE_DECLDEF(out_edges, const);
  RCPPSW_DECORATE_DECLDEF(target, const);

  RCPPSW_DECORATE_DECLDEF(find);
  RCPPSW_DECORATE_DECLDEF(vertices);
  RCPPSW_DECORATE_DECLDEF(adjacent_vertices);
  RCPPSW_DECORATE_DECLDEF(out_edges);
  RCPPSW_DECORATE_DECLDEF(target);

  /**
   * \brief Get the vertex associated with a particular set of coordinates
   * within the view. Asserts that coordinates are within the bounds of the
   * graph underlying the view.
   *
   * \param c The coordinates.
   *
   * \return A reference to the vertex.
   */
  virtual const vertex_property_type* access(const vertex_descriptor vd) const = 0;
  virtual const vertex_property_type* access(const vertex_coord_type& c) const = 0;

  virtual const edge_property_type* access(const edge_descriptor vd) const = 0;

  /**
   * \brief Determine if the specified coordinates lie within the view entity.
   */
  virtual bool contains(const vertex_coord_type& c) const = 0;

 protected:
  const rtypes::spatial_dist& unit(void) const { return mc_unit; }
  const graph_view_type& view(void) const { return decoratee(); }
  graph_view_type& view(void) { return decoratee(); }

 private:
  /* clang-format off */
  const rtypes::spatial_dist mc_unit;
  /* clang-format on */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_BASE_GRAPH_VIEW_ENTITY_HPP_ */
