/**
 * \file graph3D_view_entity.hpp
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

#ifndef INCLUDE_COSM_REPR_GRAPH3D_VIEW_ENTITY_HPP_
#define INCLUDE_COSM_REPR_GRAPH3D_VIEW_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/entity3D.hpp"
#include "cosm/repr/base_graph_view_entity.hpp"
#include <boost/optional.hpp>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class graph3D_view_entity
 * \ingroup repr
 *
 * \brief An abstract entity which is built on a view from a graph embeddable in
 * R3 Euclidean space.
 */
template <typename TGraphType, typename TGraphViewType>
class graph3D_view_entity : public crepr::entity3D,
                            public crepr::base_graph_view_entity<TGraphType, TGraphViewType>,
                            public rer::client<graph3D_view_entity<TGraphType, TGraphViewType>> {
 public:
  using graph_view_entity_type = base_graph_view_entity<TGraphType, TGraphViewType>;

  using typename graph_view_entity_type::graph_view_type;
  using typename graph_view_entity_type::vertex_property_type;
  using typename graph_view_entity_type::vertex_descriptor;
  using typename graph_view_entity_type::vertex_coord_type;
  using typename graph_view_entity_type::vertex_iterator;
  using typename graph_view_entity_type::edge_property_type;
  using typename graph_view_entity_type::edge_descriptor;

  using graph_view_entity_type::access;
  using graph_view_entity_type::find;
  using graph_view_entity_type::unit;
  using graph_view_entity_type::out_edges;
  using graph_view_entity_type::target;
  using graph_view_entity_type::n_vertices;

  graph3D_view_entity(const rtypes::type_uuid& c_id,
                      const graph_view_type& c_view,
                      const rtypes::spatial_dist& c_unit)
      : entity3D(c_id),
        graph_view_entity_type(c_view, c_unit),
        ER_CLIENT_INIT("cosm.repr.graph3D_view_entity") {}

  ~graph3D_view_entity(void) override = default;


  rmath::vector3d ranchor3D(void) const override {
    return rmath::zvec2dvec(danchor3D(), unit().v());
  }
  rmath::vector3d rcenter3D(void) const override {
    return ranchor3D() + rmath::vector3d(xrsize().v(),
                                         yrsize().v(),
                                         zrsize().v()) / 2.0;
  }

  vertex_coord_type danchor3D(void) const override final {
    return view().ll();
  }
  vertex_coord_type dcenter3D(void) const override final {
    return view().center();
  }


  rmath::ranged xrspan(void) const override {
    return entity3D::xrspan(ranchor3D(), xrsize());
  }
  rmath::ranged yrspan(void) const override {
    return entity3D::yrspan(ranchor3D(), yrsize());
  }
  rmath::ranged zrspan(void) const override {
    return entity3D::zrspan(ranchor3D(), yrsize());
  }

  rmath::rangez xdspan(void) const override {
    return entity3D::xdspan(danchor3D(), xdsize());
  }
  rmath::rangez ydspan(void) const override {
    return entity3D::ydspan(danchor3D(), ydsize());
  }
  rmath::rangez zdspan(void) const override {
    return entity3D::zdspan(danchor3D(), zdsize());
  }

  rtypes::spatial_dist xrsize(void) const override final {
    return rtypes::spatial_dist(xdsize() * unit().v());
  }
  rtypes::spatial_dist yrsize(void) const override final {
    return rtypes::spatial_dist(ydsize() * unit().v());
  }
  rtypes::spatial_dist zrsize(void) const override final {
    return rtypes::spatial_dist(zdsize() * unit().v());
  }


  const vertex_property_type* access(const vertex_coord_type& c) const override {
    ER_ASSERT(c.x() < xdsize(),
              "Out of bounds X access: %zu >= %lu",
              c.x(),
              xdsize());
    ER_ASSERT(c.y() < ydsize(),
              "Out of bounds Y access: %zu >= %lu",
              c.y(),
              ydsize());
    ER_ASSERT(c.z() < ydsize(),
              "Out of bounds Z access: %zu >= %lu",
              c.z(),
              zdsize());


    if (auto vd = view().find(c)) {
      return access(*vd);
    }
    return nullptr;
  }
  const vertex_property_type* access(vertex_descriptor vd) const override {
      return &view()[vd];
  }
  const edge_property_type* access(edge_descriptor vd) const override {
      return &view()[vd];
  }

  bool contains(const vertex_coord_type& c) const override {
    return boost::none != view().find(c);
  }

  size_t xdsize(void) const override final { return view().xsize(); }
  size_t ydsize(void) const override final { return view().ysize(); }
  size_t zdsize(void) const override final { return view().zsize(); }

 protected:
  using graph_view_entity_type::view;
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_GRAPH3D_VIEW_ENTITY_HPP_ */
