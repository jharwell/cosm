/**
 * \file spatial_entity.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/spatial/bounding_box.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/base_entity.hpp"
#include "cosm/repr/entity_dimensionality.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class spatial_entity : public base_entity {
 public:
  using rbb_type = rspatial::bounding_box<rmath::vector3d>;
  using dbb_type = rspatial::bounding_box<rmath::vector3z>;

  spatial_entity(const rtypes::type_uuid& id,
                 const rmath::vector3d& dims,
                 const rspatial::euclidean_dist& factor)
      : base_entity(id), m_rbb(dims), m_dbb(rmath::dvec2zvec(dims, factor.v())) {}

  spatial_entity(const rtypes::type_uuid& id,
                 const rmath::vector3z& dims,
                 const rspatial::euclidean_dist& factor)
      : base_entity(id), m_rbb(rmath::zvec2dvec(dims, factor.v())), m_dbb(dims) {}

  spatial_entity(const rtypes::type_uuid& id,
                 const rmath::vector3d& dims,
                 const rmath::vector3d& center,
                 const rspatial::euclidean_dist& factor)
      : base_entity(id),
        m_rbb(dims, center),
        m_dbb(rmath::dvec2zvec(dims, factor.v()),
              rmath::dvec2zvec(center, factor.v())) {}

  spatial_entity(const rtypes::type_uuid& id,
                 const rmath::vector3z& dims,
                 const rmath::vector3z& center,
                 const rspatial::euclidean_dist& factor)
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

} /* namespace cosm::repr */
