/**
 * \file factory.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/blocks/drop/factory.hpp"

#include "cosm/spatial/strategy/blocks/drop/backup.hpp"
#include "cosm/spatial/strategy/blocks/drop/backup_pivot.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::blocks::drop {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
factory::factory(void) {
  register_type<backup>(kBackup);
  register_type<backup_pivot>(kBackupPivot);
}

} /* namespace cosm::spatial::strategy::blocks::drop */
