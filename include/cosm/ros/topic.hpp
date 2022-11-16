/**
 * \file topic.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <filesystem>

#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"
#include "cosm/pal/pal.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ros);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
using topic = std::filesystem::path;

/*******************************************************************************
 * Free Functions
 ******************************************************************************/
topic to_ns(const rtypes::type_uuid& robot_id);

NS_END(ros, cosm);
