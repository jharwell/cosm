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
           const rtypes::discretize_ratio& resolution)
    : unicell_immovable_entity2D(rtypes::type_uuid(m_nest_id++),
                                 config->dims,
                                 resolution,
                                 config->center),
      colored_entity(rutils::color::kGRAY70),
      ER_CLIENT_INIT("cosm.repr.nest"),
      mc_config(*config) {
  ER_ASSERT(mc_config.light_height > 0UL, "Light height must be > 0");
  ER_ASSERT(mc_config.light_intensity > 0.0, "Light intensity must be > 0");
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest::initialize(pal::argos_sm_adaptor* sm,
                      const rutils::color& light_color) {
  if (std::fabs(rdim2D().x() - rdim2D().y()) <=
      std::numeric_limits<double>::epsilon()) {
    m_lights = init_square(light_color);
  } else {
    m_lights = init_rect(light_color);
  }

  for (auto light : m_lights) {
    light.initialize(sm);
  } /* for(light..) */
} /* initialize() */


std::list<nest_light> nest::init_square(const rutils::color& color) const {
  return std::list<nest_light>{
    nest_light("nest" + rcppsw::to_string(id()) + "_" + "light0",
               rmath::vector3(rcenter2D().x(),
                              rcenter2D().y(),
                              mc_config.light_height.v()),
               color,
               mc_config.light_intensity)
      };
} /* init_square() */

std::list<nest_light> nest::init_rect(const rutils::color& color) const {
  std::list<nest_light> ret;
  rmath::vector3d loc0, loc1, loc2;
  std::string name0 = "nest" + rcppsw::to_string(id()) + "_" + "light0";
  std::string name1 = "nest" + rcppsw::to_string(id()) + "_" + "light1";
  std::string name2 = "nest" + rcppsw::to_string(id()) + "_" + "light2";

  if (xrsize() > yrsize()) {
    loc0.set((ranchor2D().x() + xrsize() * 0.25).v(),
             rcenter2D().y(),
             mc_config.light_height.v());
    loc1.set((ranchor2D().x() + xrsize() * 0.5).v(),
             rcenter2D().y(),
             mc_config.light_height.v());
    loc2.set((ranchor2D().x() + xrsize() * 0.75).v(),
             rcenter2D().y(),
             mc_config.light_height.v());
  } else {
    loc0.set(rcenter2D().x(),
             (ranchor2D().y() + yrsize() * 0.25).v(),
             mc_config.light_height.v());
    loc1.set(rcenter2D().x(),
             (ranchor2D().y() + yrsize() * 0.5).v(),
             mc_config.light_height.v());
    loc2.set(rcenter2D().x(),
             (ranchor2D().y() + yrsize() * 0.75).v(),
             mc_config.light_height.v());
  }

  return { nest_light(name0,
                      loc0,
                      color,
                      mc_config.light_intensity),
           nest_light(name1,
                      loc1,
                      color,
                      mc_config.light_intensity),
           nest_light(name2,
                      loc2,
                      color,
                      mc_config.light_intensity)
  };
} /* init_rect() */


std::string nest::to_str(bool full) const {
  /* Can't call dcenter2D(), as the nest might be even in X and/or Y */
  std::string base =
      "nest" + rcppsw::to_string(id()) + "@" + rcenter2D().to_str();

  if (full) {
    return base + " x=" + rcppsw::to_string(xrspan()) + "/" +
           rcppsw::to_string(xdspan()) + " y=" + rcppsw::to_string(yrspan()) +
           "/" + rcppsw::to_string(ydspan());
  } else {
    return base;
  }
} /* to_str() */

NS_END(repr, cosm);
