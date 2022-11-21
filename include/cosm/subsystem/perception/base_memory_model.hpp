/**
 * \file base_memory_model.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::subsystem::perception {

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

} /* namespace cosm::subsystem::perception */

