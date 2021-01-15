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
nest::nest(const rmath::vector2d& dim,
           const rmath::vector2d& center,
           const rtypes::discretize_ratio& resolution,
           const rutils::color& light_color)
    : unicell_immovable_entity2D(rtypes::type_uuid(m_nest_id++),
                                 dim,
                                 resolution,
                                 center),
      colored_entity(rutils::color::kGRAY70),
      m_lights(init_lights(light_color)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
nest::light_list nest::init_lights(const rutils::color& color) const {
  if (std::fabs(rdim2D().x() - rdim2D().y()) <=
      std::numeric_limits<double>::epsilon()) {
    return init_square(color);
  } else {
    return init_rect(color);
  }
} /* init_lights() */

nest::light_list nest::init_square(const rutils::color& color) const {
  argos::CVector3 loc(rcenter2D().x(), rcenter2D().y(), kLIGHT_HEIGHT);
  return light_list{ new argos::CLightEntity(
      "nest_light0",
      loc,
      argos::CColor(color.red(), color.green(), color.blue()),
      kLIGHT_INTENSITY) };
} /* init_square() */

nest::light_list nest::init_rect(const rutils::color& color) const {
  light_list ret;
  argos::CVector3 loc1, loc2, loc3;

  if (xrsize() > yrsize()) {
    loc1.Set((ranchor2D().x() + xrsize() * 0.25).v(), rcenter2D().y(), kLIGHT_HEIGHT);
    loc2.Set((ranchor2D().x() + xrsize() * 0.5).v(), rcenter2D().y(), kLIGHT_HEIGHT);
    loc3.Set((ranchor2D().x() + xrsize() * 0.75).v(), rcenter2D().y(), kLIGHT_HEIGHT);
  } else {
    loc1.Set(rcenter2D().x(), (ranchor2D().y() + yrsize() * 0.25).v(), kLIGHT_HEIGHT);
    loc2.Set(rcenter2D().x(), (ranchor2D().y() + yrsize() * 0.5).v(), kLIGHT_HEIGHT);
    loc3.Set(rcenter2D().x(), (ranchor2D().y() + yrsize() * 0.75).v(), kLIGHT_HEIGHT);
  }

  return { new argos::CLightEntity(
               "nest_light0",
               loc1,
               argos::CColor(color.red(), color.green(), color.blue()),
               kLIGHT_INTENSITY),
           new argos::CLightEntity(
               "nest_light1",
               loc2,
               argos::CColor(color.red(), color.green(), color.blue()),
               kLIGHT_INTENSITY),
           new argos::CLightEntity(
               "nest_light2",
               loc3,
               argos::CColor(color.red(), color.green(), color.blue()),
               kLIGHT_INTENSITY) };
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
