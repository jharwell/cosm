/**
 * \file msg_traits.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <ros/ros.h>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ros::metrics::msg_traits {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
template<class TMsg>
struct payload_type;

} /* namespace cosm::ros::metrics::msg_traits */
