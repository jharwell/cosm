/**
 * \file mlos_perception_subsystem.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <utility>

#include "cosm/subsystem/perception/rlos/rlos_perception_subsystem.hpp"
#include "cosm/subsystem/perception/base_memory_model.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem::perception {

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
class mlos_perception_subsystem : public rlos::rlos_perception_subsystem<TLOS> {
 public:
  mlos_perception_subsystem(
      const csprlos::config::rlos_config* const config,
      std::unique_ptr<base_memory_model> model)
      : csprlos::rlos_perception_subsystem<TLOS>(config),
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

} /* namespace cosm::subsystem::perception */
