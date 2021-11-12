/**
 * \file nest_light.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/nest_light.hpp"

#include "cosm/pal/argos/sm_adaptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
nest_light::nest_light(const std::string& name,
                       const rmath::vector3d& pos,
                       const rutils::color& color,
                       double intensity)
    : colored_entity(color),
      m_impl(new argos::CLightEntity(name,
                                     argos::CVector3(pos.x(),
                                                     pos.y(),
                                                     pos.z()),
                                     argos::CColor(color.red(),
                                                   color.green(),
                                                   color.blue()),
                                     intensity)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest_light::initialize(cpargos::sm_adaptor* sm) {
  sm->AddEntity(*m_impl);
} /* initialize() */

NS_END(repr, cosm);
