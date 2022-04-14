/**
 * \file adaptor2D.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/controller/base_controller2D.hpp"
#include "cosm/pal/ros/controller/base_adaptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal, ros, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class adaptor2D
 * \ingroup pal ros controller
 *
 * \brief Adaptor for \ref controller::base_controller2D to provide an interface
 * for creating controllers within ROS.
 */
class adaptor2D : public ccontroller::base_controller2D,
                  public cpros::controller::base_adaptor {
 public:
  void entity_id(const rtypes::type_uuid& entity_id) {
    m_entity_id = entity_id;
  }

  /* base_controller2D overrides */
  rtypes::type_uuid entity_id(void) const override {
    return m_entity_id;
  }

  /* clang-format off */
  /**
   * \brief We use a mutator rather than making this a constructor argument so
   * that we have the same interface as ARGoS.
   */
  rtypes::type_uuid m_entity_id{rtypes::type_uuid(rtypes::constants::kNoUUID)};
  /* clang-format on */
};

NS_END(controller, ros, pal, cosm);
