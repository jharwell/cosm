/**
 * \file real_block.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/repr/base_entity.hpp"
#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class real_block3D
 * \ingroup repr
 *
 * \brief Base class for representing blocks (i.e. things that robots carry
 * within the arena). Blocks have both real (where they actually live in the
 * world) and discretized locations (where they are mapped to within the arena).
 */
class real_block3D : public rer::client<real_block3D>,
                     public crepr::base_block3D {
 public:
  /**
   * The dimensions and arena_res fields are not used for real blocks (for now).
   */
  real_block3D(const rutils::color& color,
               const crepr::block_type& type)
      : ER_CLIENT_INIT("cosm.repr.real_block3D"),
        base_block3D(rtypes::type_uuid(m_next_id++),
                     rmath::vector3d{1.0, 1.0, 1.0},
                     rtypes::discretize_ratio(1.0),
                     color,
                     type) {}

  ~real_block3D(void) override = default;

  std::unique_ptr<base_block3D> clone(void) const override {
    auto tmp = std::make_unique<real_block3D>(md()->color(), md()->type());
    this->real_block3D::clone_impl(tmp.get());
    return tmp;
  } /* clone() */

  void update_on_pickup(const rtypes::type_uuid& robot_id,
                        const rtypes::timestep& t,
                        const crops::block_pickup_owner& owner) override {
    switch (owner) {
      case crops::block_pickup_owner::ekROBOT:
        md()->robot_pickup_event(robot_id);
        md()->first_pickup_time(t);
        break;
      default:
        ER_FATAL_SENTINEL("Unhandled pickup owner %d",
                          rcppsw::as_underlying(owner));
        break;
    } /* switch() */
  }

 private:
  /* clang-format off */
  static inline int m_next_id{0};
  /* clang-format on */
};

NS_END(repr, cosm);

