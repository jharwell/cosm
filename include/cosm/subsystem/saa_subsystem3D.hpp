/**
 * \file saa_subsystem3D.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/subsystem/base_saa_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class saa_subsystem3D
 * \ingroup subsystem
 *
 * \brief Sensing and Actuation (SAA) subsystem for a robot when it is operating
 * in 3D environments. The precise set of sensors/actuators abstracted away at a
 * lower level, so that this class can be used for any robot which has 3D
 * bindings in the HAL.
 */
class saa_subsystem3D final : public rer::client<saa_subsystem3D>,
                              public base_saa_subsystem {
 public:
  saa_subsystem3D(sensor_map&& sensors,
                  actuator_map&& actuators,
                  const apf2D::config::apf_manager_config* const apf_config);

  void apf_apply(void) override;
};

} /* namespace cosm::subsystem */
