/**
 * \file base_saa_subsystem.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/subsystem/sensing_subsystem.hpp"
#include "cosm/subsystem/actuation_subsystem.hpp"
#include "cosm/apf2D/apf_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_saa_subsystem
 * \ingroup subsystem
 *
 * \brief Base Sensing and Actuation (SAA) subsystem for all robots.
 */
class base_saa_subsystem : public rer::client<base_saa_subsystem>,
                           public capf2D::boid {
 public:
  using sensing_type = csubsystem::sensing_subsystem;
  using actuation_type = csubsystem::actuation_subsystem;
  using sensor_map = typename chsubsystem::sensor_variant_map<COSM_HAL_ROBOT_AVAILABLE_SENSORS>;
  using actuator_map = typename chsubsystem::actuator_variant_map<COSM_HAL_ROBOT_AVAILABLE_ACTUATORS>;

  base_saa_subsystem(sensor_map&& sensors,
                     actuator_map&& actuators,
                     const apf2D::config::apf_manager_config* const apf_config);

  sensing_type* sensing(void) { return m_sensing.get(); }
  const sensing_type* sensing(void) const { return m_sensing.get(); }
  actuation_type* actuation(void) { return m_actuation.get(); }
  const actuation_type* actuation(void) const { return m_actuation.get(); }

  ckin::odometry odometry(void) const override;
  double max_velocity(void) const override RCPPSW_PURE;

  /**
   * \brief Apply and then reset the summed APF forces.
   */
  virtual void apf_apply(void) = 0;

  const apf2D::apf_manager& apf(void) const { return m_apf; }
  apf2D::apf_manager& apf(void) { return m_apf; }


 private:
  /* clang-format off */
  std::unique_ptr<actuation_type> m_actuation;
  std::unique_ptr<sensing_type>   m_sensing;
  capf2D::apf_manager             m_apf;
  /* clang-format on */
};

} /* namespace cosm::subsystem */
