/**
 * \file mlos_perception_subsystem.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_SUBSYSTEM_PERCEPTION_MLOS_PERCEPTION_SUBSYSTEM_HPP_
#define INCLUDE_COSM_SUBSYSTEM_PERCEPTION_MLOS_PERCEPTION_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <utility>

#include "cosm/subsystem/perception/rlos_perception_subsystem.hpp"
#include "cosm/subsystem/perception/base_memory_model.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class mlos_perception_subsystem
 * \ingroup subsystem perception
 *
 * \brief Base class for Line-Of-Sight (LOS) based robot perception which
 * includes memory.
 */
template<typename TLOS>
class mlos_perception_subsystem : public rlos_perception_subsystem<TLOS> {
 public:
  mlos_perception_subsystem(
      const cspconfig::rlos_config* const config,
      std::unique_ptr<base_memory_model> model)
      : rlos_perception_subsystem<TLOS>(config),
        m_model(std::move(model)) {}

  ~mlos_perception_subsystem(void) override = default;

  const base_memory_model* model(void) const { return m_model.get(); }
  base_memory_model* model(void) { return m_model.get(); }

 protected:
  void model(std::unique_ptr<base_memory_model> model) {
    m_model = std::move(model);
  }

 private:
  /* clang-format off */
  std::unique_ptr<base_memory_model> m_model{nullptr};
  /* clang-format on */
};

NS_END(perception, subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_PERCEPTION_MLOS_PERCEPTION_SUBSYSTEM_HPP_ */
