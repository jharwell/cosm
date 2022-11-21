/**
 * \file nest_light.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/nest_light.hpp"

#include "cosm/pal/argos/swarm_manager_adaptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
nest_light::nest_light(const std::string& name,
                       const rmath::vector3d& pos,
                       const rutils::color& color,
                       double intensity)
    : colored_entity(color),
      m_impl(new ::argos::CLightEntity(
          name,
          ::argos::CVector3(pos.x(), pos.y(), pos.z()),
          ::argos::CColor(color.red(), color.green(), color.blue()),
          intensity)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest_light::initialize(cpargos::swarm_manager_adaptor* sm) {
  sm->AddEntity(*m_impl);
} /* initialize() */

} /* namespace cosm::repr */
