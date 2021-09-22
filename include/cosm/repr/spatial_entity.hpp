/**
 * \file spatial_entity.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_REPR_SPATIAL_ENTITY_HPP_
#define INCLUDE_COSM_REPR_SPATIAL_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/spatial_dist.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/math/bounding_box.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/base_entity.hpp"
#include "cosm/repr/entity_dimensionality.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class spatial_entity : public base_entity {
 public:
  using rbb_type = rmath::bounding_box<rmath::vector3d>;
  using dbb_type = rmath::bounding_box<rmath::vector3z>;

  spatial_entity(const rtypes::type_uuid& id,
                 const rmath::vector3d& dims,
                 const rtypes::spatial_dist& factor)
      : base_entity(id),
        m_rbb(dims),
        m_dbb(rmath::dvec2zvec(dims, factor.v())) {}

  spatial_entity(const rtypes::type_uuid& id,
                 const rmath::vector3z& dims,
                 const rtypes::spatial_dist& factor)
      : base_entity(id),
        m_rbb(rmath::zvec2dvec(dims, factor.v())),
        m_dbb(dims) {}


  spatial_entity(const rtypes::type_uuid& id,
                 const rmath::vector3d& dims,
                 const rmath::vector3d& center,
                 const rtypes::spatial_dist& factor)
      : base_entity(id),
        m_rbb(dims, center),
        m_dbb(rmath::dvec2zvec(dims, factor.v()),
              rmath::dvec2zvec(center, factor.v())) {}

  spatial_entity(const rtypes::type_uuid& id,
                 const rmath::vector3z& dims,
                 const rmath::vector3z& center,
                 const rtypes::spatial_dist& factor)
      : base_entity(id),
        m_rbb(rmath::zvec2dvec(dims, factor.v()),
              rmath::zvec2dvec(center, factor.v())),
        m_dbb(dims, center) {}

  ~spatial_entity(void) override = default;

  /**
   * \brief Return whether the entity is 2D or 3D.
   */
  virtual entity_dimensionality dimensionality(void) const = 0;

 protected:
  auto& rbb(void) { return m_rbb; }
  const auto& rbb(void) const { return m_rbb; }

  auto& dbb(void) { return m_dbb; }
  const auto& dbb(void) const { return m_dbb; }

 private:
  /* clang-format off */
  rbb_type m_rbb;
  dbb_type m_dbb;
  /* clang-format on */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_SPATIAL_ENTITY_HPP_ */
