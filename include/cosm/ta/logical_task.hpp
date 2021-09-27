/**
 * \file logical_task.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_TA_LOGICAL_TASK_HPP_
#define INCLUDE_COSM_TA_LOGICAL_TASK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/ta/time_estimate.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta);

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

NS_END(ta, cosm);

#endif /* INCLUDE_COSM_TA_LOGICAL_TASK_HPP_ */
