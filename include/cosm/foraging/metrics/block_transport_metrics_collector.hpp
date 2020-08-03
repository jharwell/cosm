/**
 * \file block_transport_metrics_collector.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_FORAGING_METRICS_BLOCK_TRANSPORT_METRICS_COLLECTOR_HPP_
#define INCLUDE_COSM_FORAGING_METRICS_BLOCK_TRANSPORT_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>
#include <atomic>

#include "rcppsw/metrics/base_metrics_collector.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_transport_metrics_collector
 * \ingroup cosm foraging metrics
 *
 * \brief Collector for \ref block_transport_metrics.
 *
 * Metrics are written out at the specified collection interval.
 */
class block_transport_metrics_collector final : public rmetrics::base_metrics_collector {
 public:
  /**
   * \param ofname_stem The output file name stem.
   * \param interval Collection interval.
   */
  block_transport_metrics_collector(const std::string& ofname_stem,
                              const rtypes::timestep& interval);

  void reset(void) override;
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;

  size_t cum_transported(void) const { return m_cum.transported; }

 private:
  /**
   * \brief Container for holding block_transported statistics. Must be atomic so
   * counts are valid in parallel metric collection contexts. Ideally the times
   * would be atomic \ref rtypes::timestep, but that type does not meet the
   * std::atomic requirements.
   */
  struct stats {
    /**
     * \brief  Total # blocks transported.
     */
    std::atomic_size_t transported{0};

    /**
     * \brief  Total # cube blocks transported.
     */
    std::atomic_size_t cube_transported{0};

    /**
     * \brief  Total # ramp blocks transported in interval.
     */
    std::atomic_size_t ramp_transported{0};

    /**
     * \brief Total # transporters for transported blocks in interval.
     */
    std::atomic_size_t transporters{0};

    /**
     * \brief Total amount of time taken for all transported blocks to be
     * transported from original distribution locations to the nest within an
     * interval.
     */
    std::atomic_size_t transport_time{0};

    /**
     * \brief Total amount of time between original arena distribution and first
     * pickup for all transported blocks in interval.
     */
    std::atomic_size_t initial_wait_time{0};
  };

  std::list<std::string> csv_header_cols(void) const override;
  boost::optional<std::string> csv_line_build(void) override;

  /* clang-format off */
  struct stats m_interval{};
  struct stats m_cum{};
  /* clang-format on */
};

NS_END(metrics, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_METRICS_BLOCK_TRANSPORT_METRICS_COLLECTOR_HPP_ */
