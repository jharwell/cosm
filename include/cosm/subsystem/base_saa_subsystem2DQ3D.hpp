/**
 * \file base_saa_subsystem2DQ3D.hpp
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

#ifndef INCLUDE_COSM_SUBSYSTEM_BASE_SAA_SUBSYSTEM2DQ3D_HPP_
#define INCLUDE_COSM_SUBSYSTEM_BASE_SAA_SUBSYSTEM2DQ3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/steer2D/force_calculator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_saa_subsystem2DQ3D
 * \ingroup subsystem
 *
 * \brief Base Sensing and Actuation (SAA) subsystem for 2D and quasi-3D (Q3D)
 * SAA subsystems to provide a common interface/some degree of reuse between
 * them.
 */
class base_saa_subsystem2DQ3D : public steer2D::boid {
 public:
  base_saa_subsystem2DQ3D(
      const steer2D::config::force_calculator_config* const steer_config)
      : m_steer2D_calc(*this, steer_config) {}

  /**
   * \brief Apply the summed steering forces; change wheel speeds. Resets the
   * summed forces.
   */
  virtual void steer_force2D_apply(void) = 0;

  const steer2D::force_calculator& steer_force2D(void) const {
    return m_steer2D_calc;
  }
  steer2D::force_calculator& steer_force2D(void) { return m_steer2D_calc; }

 private:
  /* clang-format off */
  steer2D::force_calculator m_steer2D_calc;
  /* clang-format on */
};

NS_END(subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_BASE_SAA_SUBSYSTEM2DQ3D_HPP_ */
