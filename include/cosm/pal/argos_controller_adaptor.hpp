/**
 * \file argos_controller_adaptor.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_PAL_ARGOS_CONTROLLER_ADAPTOR_HPP_
#define INCLUDE_COSM_PAL_ARGOS_CONTROLLER_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/control_interface/ci_controller.h>

#include "cosm/controller/base_controller.hpp"
#include "cosm/hal/hal.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class argos_controller_adaptor
 * \ingroup pal
 *
 * \brief Adaptor to provide an interface for creating controllers within ARGoS.
 */
class argos_controller_adaptor : public argos::CCI_Controller {};

NS_END(pal, cosm);

#endif /* INCLUDE_COSM_PAL_ARGOS_CONTROLLER_ADAPTOR_HPP_ */
