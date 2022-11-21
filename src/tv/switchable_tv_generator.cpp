/**
 * \file switchable_tv_generator.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
namespace cosm::tv {

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

} /* namespace cosm::tv */
