/**
 * \file crw.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/expstrat/crw.hpp"

#include "cosm/subsystem/saa_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces
******************************************************************************/
NS_START(cosm, spatial, expstrat);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
crw::crw(csubsystem::saa_subsystemQ3D* saa, rmath::rng* rng)
    : base_expstrat(saa, rng), ER_CLIENT_INIT("cosm.spatial.expstrat.crw") {}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void crw::task_execute(void) {
  saa()->steer_force2D().accum(saa()->steer_force2D().wander(rng()));
  auto* prox = saa()->sensing()->sensor<chal::sensors::proximity_sensor>();
  auto* leds = saa()->actuation()->actuator<chal::actuators::led_actuator>();

  if (auto obs = prox->avg_prox_obj()) {
    inta_tracker()->inta_enter();
    saa()->steer_force2D().accum(saa()->steer_force2D().avoidance(*obs));

    ER_DEBUG("Found threatening obstacle: %s@%f [%f]",
             obs->to_str().c_str(),
             obs->angle().v(),
             obs->length());
    leds->set_color(-1, rutils::color::kRED);
  } else {
    inta_tracker()->inta_exit();

    ER_DEBUG("No threatening obstacle found");
    leds->set_color(-1, rutils::color::kMAGENTA);
    rmath::vector2d force = saa()->steer_force2D().value();
    /*
     * This can be 0 if the wander force is not active this timestep.
     */
    if (force.length() >= std::numeric_limits<double>::epsilon()) {
      saa()->steer_force2D().value(saa()->steer_force2D().value() * 0.7);
    }
  }
} /* task_execute() */

NS_END(expstrat, spatial, cosm);
