/**
 * \file applicator.hpp
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
#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, controller, operations);

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

NS_END(operations, controller, cosm);

