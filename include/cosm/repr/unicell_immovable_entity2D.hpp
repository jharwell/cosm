/**
 * \file unicell_immovable_entity2D.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/discretize_ratio.hpp"

#include "cosm/repr/unicell_entity2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class unicell_immovable_entity2D
 * \ingroup repr
 *
 * \brief A class representing 2D objects that reside within one or more squares
 * within a 2D grid whose position CANNOT change during the lifetime of the
 * object.
 */
class unicell_immovable_entity2D : public unicell_entity2D {
 public:
  static constexpr bool is_movable(void) { return false; }

  unicell_immovable_entity2D(const rtypes::type_uuid& id,
                             const rmath::vector2d& dims,
                             const rmath::vector2d& anchor,
                             const rtypes::discretize_ratio& resolution)
      : unicell_entity2D(id, dims, anchor, resolution) {}

  ~unicell_immovable_entity2D(void) override = default;
};

} /* namespace cosm::repr */
