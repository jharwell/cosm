/**
 * \file adaptor2D.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "cosm/pal/argos/controller/base_adaptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal, argos, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class adaptor2D
 * \ingroup pal argos controller
 *
 * \brief Adaptor for \ref controller::base_controller2D to provide an interface
 * for creating controllers within ARGoS.
 */
class adaptor2D : public ccontroller::base_controller2D,
                             public cpargos::controller::base_adaptor {
 public:
  /* ARGoS hook overrides */
  void Init(ticpp::Element& node) override RCPPSW_COLD { init(node); }
  void Reset(void) override RCPPSW_COLD { reset(); }
  void ControlStep(void) override { control_step(); }

  /* base_controller2D overrides */
  rtypes::type_uuid entity_id(void) const override {
    return rtypes::type_uuid(std::atoi(GetId().c_str() + 2));
  }
};

NS_END(controller, argos, pal, cosm);

