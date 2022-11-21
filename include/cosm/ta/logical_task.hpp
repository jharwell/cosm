/**
 * \file logical_task.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <utility>

#include "cosm/ta/time_estimate.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class logical_task
 * \ingroup ta
 *
 * \brief Represents the logical concept of a task, which only contains a name
 * right now.
 */
class logical_task {
 public:
  explicit logical_task(std::string name) : m_name(std::move(name)) {}

  virtual ~logical_task(void);

  logical_task(const logical_task& other) = default;

  /**
   * \brief Get the name of the task
   */
  const std::string& name(void) const { return m_name; }

  logical_task& operator=(const logical_task&) = delete;
  bool operator==(const logical_task& other) const {
    return this->m_name == other.name();
  }

 private:
  std::string m_name;
};

} /* namespace cosm::ta */
