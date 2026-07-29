// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROMEA_CORE_LOCALISATION__FILTER_KALMAN_HPP_
#define ROMEA_CORE_LOCALISATION__FILTER_KALMAN_HPP_

// std
#include <memory>
#include <optional>
#include <string>
#include <utility>

// romea
#include "romea_core_localisation/filter_impl.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

template<class Traits>
class Filter<KALMAN, Traits>
{
private:
  using Base = FilterImpl<Traits>;

public:
  using Results = typename Base::Results;
  using LogCallback = typename Base::LogCallback;
  using LoggerMap = typename Base::LoggerMap;
  template<class Updater>
  using UpdateCallback = typename Base::template UpdateCallback<Updater>;

  template<class... PredictorArgs>
  explicit Filter(const std::size_t & state_pool_size, PredictorArgs &&... predictor_args)
  : impl_(
      std::make_unique<typename Base::Filter>(state_pool_size),
      std::make_unique<typename Base::Predictor>(std::forward<PredictorArgs>(predictor_args)...),
      std::make_unique<typename Base::MetaState>(),
      std::make_unique<typename Base::MetaStateToResults>())
  {
  }

  bool initialize(
    LogCallback event_log_callback = LogCallback(),
    LogCallback get_results_log_callback = LogCallback(),
    FSMEventCallback fsm_event_callback = nullptr,
    const LoggerMap & loggers = LoggerMap())
  {
    return impl_.initialize(
      std::move(event_log_callback),
      std::move(get_results_log_callback),
      fsm_event_callback,
      loggers);
  }

  void reset() { impl_.reset(); }

  FilterQuery<Results> get_results(const Duration & duration)
  {
    return impl_.get_results(duration);
  }

  DiagnosticReport make_diagnostic_report(const Duration & duration, const FSMState & fsm_state)
  {
    return impl_.make_diagnostic_report(duration, fsm_state);
  }

  template<class Updater, class... Args>
  std::optional<UpdateCallback<Updater>> add_proprioceptive_updater(
    const std::string & updater_name, const double & minimal_rate, Args &&... args)
  {
    auto updater =
      std::make_unique<Updater>(updater_name, minimal_rate, std::forward<Args>(args)...);
    return impl_.add_proprioceptive_updater(updater_name, std::move(updater));
  }

  template<class Updater, class... Args>
  std::optional<UpdateCallback<Updater>> add_exteroceptive_updater(
    const std::string & updater_name,
    const double & minimal_rate,
    const UpdaterTriggerMode & trigger_mode,
    const double & maximal_mahalanobis_distance,
    Args &&... args)
  {
    auto updater = std::make_unique<Updater>(
      updater_name,
      minimal_rate,
      trigger_mode,
      maximal_mahalanobis_distance,
      std::forward<Args>(args)...);
    return impl_.add_exteroceptive_updater(updater_name, std::move(updater));
  }

private:
  Base impl_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__FILTER_KALMAN_HPP_
