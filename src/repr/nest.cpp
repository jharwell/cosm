/**
 * \file nest.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/nest.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Constants
 ******************************************************************************/
int nest::m_nest_id = 0;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
nest::nest(const config::nest_config* config,
           const rmath::vector2d& arena_dim,
           const rtypes::discretize_ratio& resolution)
    : unicell_immovable_entity2D(rtypes::type_uuid(m_nest_id++),
                                 config->dims,
                                 config->center - config->dims / 2.0,
                                 resolution),
      colored_entity(rutils::color::kGRAY70),
      ER_CLIENT_INIT("cosm.repr.nest"),
      mc_light_intensity(light_intensity_calc(arena_dim)),
      mc_light_height(light_height_calc(arena_dim)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest::initialize(cpargos::swarm_manager_adaptor* sm,
                      const rutils::color& light_color) {
  if (m_initialized) {
    return;
  }

  if (std::fabs(rdims2D().x() - rdims2D().y()) <=
      std::numeric_limits<double>::epsilon()) {
    m_lights = init_square(light_color);
  } else {
    m_lights = init_rect(light_color);
  }

  for (auto light : m_lights) {
    light.initialize(sm);
  } /* for(light..) */
  m_initialized = true;
} /* initialize() */


std::list<nest_light> nest::init_square(const rutils::color& color) {
  return std::list<nest_light>{
    nest_light("nest" + rcppsw::to_string(id()) + "_" + "light0",
               rmath::vector3(rcenter2D().x(),
                              rcenter2D().y(),
                              mc_light_height.v()),
               color,
               mc_light_intensity)
      };
} /* init_square() */

std::list<nest_light> nest::init_rect(const rutils::color& color) {
  std::list<nest_light> ret;
  rmath::vector3d loc0, loc1, loc2;
  std::string name0 = "nest" + rcppsw::to_string(id()) + "_" + "light0";
  std::string name1 = "nest" + rcppsw::to_string(id()) + "_" + "light1";
  std::string name2 = "nest" + rcppsw::to_string(id()) + "_" + "light2";

  if (xrsize() > yrsize()) {
    loc0.set((ranchor2D().x() + xrsize() * 0.25).v(),
             rcenter2D().y(),
             mc_light_height.v());
    loc1.set((ranchor2D().x() + xrsize() * 0.5).v(),
             rcenter2D().y(),
             mc_light_height.v());
    loc2.set((ranchor2D().x() + xrsize() * 0.75).v(),
             rcenter2D().y(),
             mc_light_height.v());
  } else {
    loc0.set(rcenter2D().x(),
             (ranchor2D().y() + yrsize() * 0.25).v(),
             mc_light_height.v());
    loc1.set(rcenter2D().x(),
             (ranchor2D().y() + yrsize() * 0.5).v(),
             mc_light_height.v());
    loc2.set(rcenter2D().x(),
             (ranchor2D().y() + yrsize() * 0.75).v(),
             mc_light_height.v());
  }

  return { nest_light(name0,
                      loc0,
                      color,
                      mc_light_intensity),
           nest_light(name1,
                      loc1,
                      color,
                      mc_light_intensity),
           nest_light(name2,
                      loc2,
                      color,
                      mc_light_intensity)
  };
} /* init_rect() */


std::string nest::to_str(void) const {
  /* Can't call dcenter2D(), as the nest might be even in X and/or Y */
  return "nest" + rcppsw::to_string(id()) + "@" + rcenter2D().to_str();
} /* to_str() */

NS_END(repr, cosm);
