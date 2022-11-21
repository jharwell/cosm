/**
 * \file applicator.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
namespace cosm::controller::operations {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class applicator
 * \ingroup controller operations
 *
 * \brief Wrapping functor to applicator an operation to a controller which is
 * derived from a common base class.
 *
 * \tparam TBaseController The type of the base controller class which all
 *                         controllers processed by this class are derived from.
 *
 * \tparam TOperation The operation to perform. All operations must take the
 *                    controller type to process as their first template
 *                    argument, and can take any number of additional template
 *                    arguments.
 */
template<class TBaseController,
         template <class TDerivedController, class...> class TOperation,
         class ...Args>
class applicator {
 public:
  explicit applicator(TBaseController* const c) : m_controller(c) {}

  template <typename TDerivedController, typename ...OpArgs>
  auto operator()(const TOperation<TDerivedController, Args...>& op,
                  OpArgs&& ...args) const -> decltype(op(std::declval<TDerivedController*>(), args...)) {
    return op(static_cast<TDerivedController*>(m_controller),
              std::forward<OpArgs>(args)...);
  }

 private:
  /* clang-format off */
  TBaseController* const m_controller;
  /* clang-format on */
};

} /* namespace cosm::controller::operations */

