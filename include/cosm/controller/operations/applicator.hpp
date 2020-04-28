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
#ifndef INCLUDE_COSM_CONTROLLER_OPERATIONS_APPLICATOR_HPP_
#define INCLUDE_COSM_CONTROLLER_OPERATIONS_APPLICATOR_HPP_

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
 * \tparam TBaseControllerType The type of the base controller class which all
 *                             controllers processed by this class are derived
 *                             from.
 *
 * \tparam TOperation The operation to perform. All operations must take the
 *                    controller type to process as their first template
 *                    argument, and can take any number of additional template
 *                    arguments.
 */
template<class TBaseControllerType,
         template <class TDerivedControllerType, class...> class TOperation,
         class ...Args>
class applicator {
 public:
  explicit applicator(TBaseControllerType* const c) : m_controller(c) {}

  template <typename TDerivedControllerType>
  auto operator()(const TOperation<TDerivedControllerType, Args...>& op) const -> decltype(op(std::declval<TDerivedControllerType*>())) {
    return op(static_cast<TDerivedControllerType*>(m_controller));
  }

 private:
  /* clang-format off */
  TBaseControllerType* const m_controller;
  /* clang-format on */
};

NS_END(operations, controller, cosm);

#endif /* INCLUDE_COSM_CONTROLLER_OPERATIONS_APPLICATOR_HPP_ */
