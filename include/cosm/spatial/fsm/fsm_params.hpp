/**
 * \file fsm_params.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/subsystem/subsystem_fwd.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial {
class interference_tracker;
class nest_zone_tracker;
} /* namespace cosm::spatial */

namespace cosm::spatial::fsm {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct fsm_params
 * \ingroup spatial fsm
 */
struct fsm_params {
  csubsystem::base_saa_subsystem* saa;
  cspatial::interference_tracker* const inta;
  cspatial::nest_zone_tracker* const nz;
};

} /* namespace cosm::spatial::fsm */
