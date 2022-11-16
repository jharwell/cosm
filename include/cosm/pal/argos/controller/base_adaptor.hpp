/**
 * \file base_adaptor.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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

  void display_apf2D(bool b) { m_display_apf2D = b; }
  bool display_apf2D(void) const { return m_display_apf2D; }

 private:
  bool m_display_id{false};
  bool m_display_apf2D{false};
  bool m_display_los{false};
};

NS_END(controller, argos, pal, cosm);
