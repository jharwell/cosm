/**
 * \file wifi_packet.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief A wireless communication sensor payload.
 */
struct wifi_packet {
  std::vector<uint8_t> data;

  wifi_packet(void) = default;
};

} /* namespace cosm::hal */
