/**
 * \file switchable_tv_generator.cpp
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
#include "cosm/tv/switchable_tv_generator.hpp"

#include "rcppsw/control/config/waveform_config.hpp"
#include "rcppsw/control/waveform_generator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, tv);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
switchable_tv_generator::switchable_tv_generator(
    const rcontrol::config::waveform_config* const config)
    : m_waveform(rcontrol::waveform_generator()(config->type, config)) {}

switchable_tv_generator::~switchable_tv_generator(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void switchable_tv_generator::update(const rtypes::timestep& t) {
  double value = m_waveform->value(t.v());
  m_applied = value;
  m_active = (m_en) ? value : 0.0;
} /* update() */

NS_END(tv, cosm);
