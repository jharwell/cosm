/**
 * \file base_adaptor.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/control_interface/ci_controller.h>

#include "rcppsw/mpl/reflectable.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal, argos, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_adaptor
 * \ingroup pal argos controller
 *
 * \brief Adaptor to provide an interface for creating controllers within ARGoS.
 */
class base_adaptor : public ::argos::CCI_Controller,
                           public rmpl::reflectable {
 public:
  /**
   * \brief Set whether or not a robot is supposed to display it's ID above its
   * head during simulation.
   */
  void display_id(bool b) { m_display_id = b; }

  /**
   * \brief If \c TRUE, then the robot should display its ID above its head
   * during simulation.
   */
  bool display_id(void) const { return m_display_id; }

  /**
   * \brief If \c TRUE, then the robot should display its current LOS during
   * simulation, if it has one.
   */
  bool display_los(void) const { return m_display_los; }

  /**
   * \brief Set whether or not a robot is supposed to display it's LOS during
   * simulation.
   */
  void display_los(bool b) { m_display_los = b; }

  void display_steer2D(bool b) { m_display_steer2D = b; }
  bool display_steer2D(void) const { return m_display_steer2D; }

 private:
  bool m_display_id{false};
  bool m_display_steer2D{false};
  bool m_display_los{false};
};

NS_END(controller, argos, pal, cosm);

