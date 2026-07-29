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

#ifndef ROMEA_CORE_LOCALISATION__FILTER_STORAGE_HPP_
#define ROMEA_CORE_LOCALISATION__FILTER_STORAGE_HPP_

// std
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>

// romea
#include "romea_core_common/diagnostic/DiagnosticReport.hpp"
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/updater_traits.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

template<class Traits>
struct FilterStorage
{
  using Filter = typename Traits::Filter;
  using MetaState = typename Traits::MetaState;
  using Predictor = typename Traits::Predictor;
  using Results = typename Traits::Results;
  using MetaStateToResults = typename Traits::MetaStateToResults;

  class UpdaterHandleBase;
  using UpdaterHandles = std::map<std::string, std::unique_ptr<UpdaterHandleBase>>;

  template<class Updater>
  using UpdateCallback =
    std::function<void(const Duration &, const typename Updater::Observation &)>;

  class UpdaterHandleBase
  {
  public:
    virtual ~UpdaterHandleBase() = default;

    virtual bool heart_beat_callback(const Duration & duration) = 0;

    virtual DiagnosticReport get_report() = 0;

    virtual void register_fsm_event_callback(FSMEventCallback callback) = 0;

    virtual void register_logger(std::shared_ptr<Logger> logger) = 0;
  };

  template<class Updater>
  class UpdaterHandle : public UpdaterHandleBase
  {
  public:
    explicit UpdaterHandle(std::unique_ptr<Updater> updater)
    : updater_(std::move(updater))
    {
    }

    Updater & updater() { return *updater_; }

    bool heart_beat_callback(const Duration & duration) override
    {
      return updater_->heart_beat_callback(duration);
    }

    DiagnosticReport get_report() override { return updater_->get_report(); }

    void register_fsm_event_callback(FSMEventCallback callback) override
    {
      updater_->register_fsm_event_callback(std::move(callback));
    }

    void register_logger(std::shared_ptr<Logger> logger) override
    {
      if constexpr (is_exteroceptive_updater<Updater>()) {
        updater_->register_logger(std::move(logger));
      }
    }

  private:
    std::unique_ptr<Updater> updater_;
  };

  std::unique_ptr<Filter> filter;
  std::unique_ptr<Predictor> predictor;
  std::unique_ptr<MetaState> current_meta_state;
  std::unique_ptr<MetaStateToResults> meta_state_to_results;
  UpdaterHandles proprioceptive_updater_handles;
  UpdaterHandles exteroceptive_updater_handles;
};

template<class Traits>
void register_fsm_callback(
  FilterStorage<Traits> & storage,
  FSMEventCallback fsm_event_callback)
{
  storage.predictor->register_fsm_event_callback(fsm_event_callback);

  for (auto & [updater_name, updater_handle] : storage.proprioceptive_updater_handles) {
    (void)updater_name;
    updater_handle->register_fsm_event_callback(fsm_event_callback);
  }

  for (auto & [updater_name, updater_handle] : storage.exteroceptive_updater_handles) {
    (void)updater_name;
    updater_handle->register_fsm_event_callback(fsm_event_callback);
  }
}

template<class Traits>
void register_topic_loggers(
  FilterStorage<Traits> & storage,
  const std::map<std::string, std::shared_ptr<Logger>> & loggers)
{
  const auto register_logger =
    [&loggers](const std::string & logger_name, auto & object) {
      const auto iter = loggers.find(logger_name);
      if (iter != loggers.end() && iter->second) {
        object.register_logger(iter->second);
      }
    };

  register_logger("predictor", *storage.predictor);

  for (auto & [updater_name, updater_handle] : storage.exteroceptive_updater_handles) {
    register_logger(updater_name, *updater_handle);
  }
}

template<class UpdaterHandles>
DiagnosticReport make_diagnostic_report(
  const Duration & duration,
  UpdaterHandles & updater_handles)
{
  DiagnosticReport report;

  for (auto & [updater_name, updater_handle] : updater_handles) {
    (void)updater_name;
    updater_handle->heart_beat_callback(duration);
    report += updater_handle->get_report();
  }

  return report;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__FILTER_STORAGE_HPP_
