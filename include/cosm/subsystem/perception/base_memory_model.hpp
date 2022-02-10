/**
 * \file base_memory_model.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, subsystem, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_memory_model
 * \ingroup subsystem perception
 *
 * \brief The base class from which all robot perception models derived.
 */
class base_memory_model {
 public:
  base_memory_model(void) = default;
  virtual ~base_memory_model(void) = default;

  /* Not move/copy constructable/assignable by default */
  base_memory_model(const base_memory_model&) = delete;
  base_memory_model& operator=(const base_memory_model&) = delete;
  base_memory_model(base_memory_model&&) = delete;
  base_memory_model& operator=(base_memory_model&&) = delete;
};

NS_END(perception, subsystem, cosm);

