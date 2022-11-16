/**
 * \file adaptorQ3D.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/controller/base_controllerQ3D.hpp"
#include "cosm/pal/argos/controller/base_adaptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal, argos, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class adaptorQ3D
 * \ingroup pal argos controller
 *
 * \brief Adaptor for \ref controller::base_controllerQ3D to provide an
 * interface for creating controllers within ARGoS.
 */
class adaptorQ3D : public ccontroller::base_controllerQ3D,
                   public cpargos::controller::base_adaptor {
 public:
  /* ARGoS hook overrides */
  void Init(ticpp::Element& node) override RCPPSW_COLD { init(node); }
  void Reset(void) override RCPPSW_COLD { reset(); }
  void ControlStep(void) override { control_step(); }

  /* base_controllerQ3D overrides */
  rtypes::type_uuid entity_id(void) const override {
    return rtypes::type_uuid(std::atoi(GetId().c_str() + 2));
  }

};

NS_END(controller, argos, pal, cosm);

