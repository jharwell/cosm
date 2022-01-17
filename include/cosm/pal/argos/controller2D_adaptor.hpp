/**
 * \file controller2D_adaptor.hpp
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

#ifndef INCLUDE_COSM_PAL_ARGOS_CONTROLLER2D_ADAPTOR_HPP_
#define INCLUDE_COSM_PAL_ARGOS_CONTROLLER2D_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/mpl/reflectable.hpp"

#include "cosm/pal/pal.hpp"
#include "cosm/controller/base_controller2D.hpp"
#include "cosm/pal/argos/controller_adaptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal, argos);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class controller2D_adaptor
 * \ingroup pal argos
 *
 * \brief Adaptor for \ref controller::base_controller2D to provide an interface
 * for creating controllers within ARGoS.
 */
class controller2D_adaptor : public ccontroller::base_controller2D,
                             public cpargos::controller_adaptor,
                             public rmpl::reflectable {
 public:
  /* ARGoS hook overrides */
  void Init(ticpp::Element& node) override RCPPSW_COLD { init(node); }
  void Reset(void) override RCPPSW_COLD { reset(); }
  void ControlStep(void) override { control_step(); }
};

NS_END(argos, pal, cosm);

#endif /* INCLUDE_COSM_PAL_ARGOS_CONTROLLER2D_ADAPTOR_HPP_ */
