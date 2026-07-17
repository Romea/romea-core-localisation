# romea_core_localisation

## Overview

`romea_core_localisation` is the C++ localisation-domain library used by the ROMEA localisation stack.

It does not provide complete standalone localisation filters by itself. Instead, it provides the localisation-specific components needed to assemble such filters with the generic Kalman or particle filtering machinery from `romea_core_filtering`.

These components include states, predictors, updaters, observations and result extractors used to estimate:

* a robot pose in the world frame;
* the relative pose between two robots;
* the relative position between a robot and a human.

The package is framework-independent C++ code. Middleware-specific nodes, message conversions and runtime configuration are intentionally kept outside this library.

---

## Localisation components

`romea_core_filtering` provides the generic asynchronous filtering layer:

* `FilterBase`, which manages timestamped meta-states and applies prediction and update steps;
* `KalmanFilter` and `ParticleFilter`, which provide the concrete filter storage for Kalman and particle states;
* `FilterPredictorBase`, which defines the prediction interface used between two timestamps;
* Kalman and particle updater base classes and equations, which implement the reusable mathematical parts of the update step.

`romea_core_localisation` provides the localisation-domain components plugged into this filtering layer:

| Component | Purpose |
| --------- | ------- |
| Observations | Typed measurements such as twist, angular speed, position, pose or range, with their uncertainty. |
| Meta-states | Localisation state variables, inputs, covariances and additional data for each localisation problem. |
| Predictors | Motion propagation logic between two timestamps. |
| Proprioceptive updaters | Update motion inputs such as twist, linear speed or angular speed. |
| Exteroceptive updaters | Correct the predicted state from external observations such as position, course, pose or range. |
| Results | Extract estimated poses, twists and uncertainty information from a filter state. |
| Traits | Group the right filter, predictor, updater and result classes for a selected model and filter type. |

Applications assemble a complete filter by selecting a `romea_core_filtering` filter type and registering the localisation predictor and updaters provided by this package.

---

## Localisation models

| Model | Namespace / prefix | Purpose |
| ----- | ------------------ | ------- |
| Robot-to-world | `R2W*` | Estimate the vehicle pose in a world frame from proprioceptive and exteroceptive observations. |
| Robot-to-robot | `R2R*` | Estimate the relative pose between a follower robot and a leader robot. |
| Robot-to-human | `R2H*` | Estimate the relative position between a robot and a human. |

These model families are independent C++ components. Application or integration packages choose which model to assemble and expose.

---

## Observations

The library defines typed Gaussian observations that can be fused by localisation filters.

| Observation | Content |
| ----------- | ------- |
| `ObservationTwist` | 2D body twist observation. |
| `ObservationLinearSpeed` | Single longitudinal speed observation. |
| `ObservationLinearSpeeds` | Left/right longitudinal speed observations. |
| `ObservationAngularSpeed` | Yaw angular speed observation. |
| `ObservationAttitude` | Roll and pitch attitude observation. |
| `ObservationPosition` | 2D position observation. |
| `ObservationCourse` | Course or heading observation. |
| `ObservationPose` | 2D pose observation. |
| `ObservationRange` | Range observation between two points or transceivers. |

Each observation carries both a value and uncertainty information, either as a standard deviation or covariance depending on its dimension.

---

## Predictors and updaters

The localisation filters are built from:

* predictors, which propagate the state using the current proprioceptive inputs;
* proprioceptive updaters, which update internal motion inputs such as twist, linear speed or angular speed;
* exteroceptive updaters, which correct the state with observations such as position, course, pose or range.

Exteroceptive updaters support trigger modes:

| Trigger | Meaning |
| ------- | ------- |
| `always` | Each valid observation can update the filter. |
| `once` | Only the first valid observation is used, commonly for initialisation. |

Kalman exteroceptive updaters support Mahalanobis-distance based rejection to discard observations that are inconsistent with the current filter state. Particle exteroceptive updaters provide the particle weighting and resampling logic for the corresponding observations.

---

## Available component sets

The package provides Kalman and particle component sets for the currently implemented localisation models:

| Model | Kalman components | Particle components | Traits |
| ----- | ----------------- | ------------------- | ------ |
| Robot-to-world | `robot_to_world/kalman/*` | `robot_to_world/particle/*` | `R2WTraits<KALMAN>`, `R2WTraits<PARTICLE>` |
| Robot-to-robot | `robot_to_robot/kalman/*` | `robot_to_robot/particle/*` | `R2RTraits<KALMAN>`, `R2RTraits<PARTICLE>` |
| Robot-to-human | `robot_to_human/kalman/*` | Not currently provided in this package. | No traits wrapper currently provided. |

For example, a robot-to-world Kalman localisation filter is assembled from the following component families:

| Component | Main class or examples | Role |
| --------- | ---------------------- | ---- |
| Filter | `romea::core::KalmanFilter<R2WKFMetaState, FSMState, Duration>` | Manages timestamped states asynchronously and applies prediction and update steps. |
| Meta-state | `R2WKFMetaState` | Stores the vehicle pose, motion inputs and covariance information. |
| FSM state | `FSMState` | Tracks the filter status associated with each stored state: `INIT`, `RUNNING`, `RESET` or `ABORTED`. |
| Predictor | `R2WKFPredictor` | Propagates the vehicle state from motion inputs and the vehicle kinematic model. |
| Exteroceptive updaters | `R2WKFUpdaterPosition`, `R2WKFUpdaterCourse`, `R2WKFUpdaterPose`, `R2WKFUpdaterRange` | Correct the predicted state with external observations. |
| Proprioceptive updaters | `UpdaterTwist`, `UpdaterLinearSpeed`, `UpdaterLinearSpeeds`, `UpdaterAngularSpeed`, `R2WUpdaterAttitude` | Update the motion inputs and attitude data used by the predictor. |
| Results | `R2WResults` | Stores the estimated pose, twist and covariance information converted from the current filter state. |

The corresponding traits can be used to select the right component set:

```cpp
using Traits = romea::core::localisation::R2WTraits<romea::core::KALMAN>;

auto filter = std::make_unique<Traits::Filter>(state_pool_size);
auto predictor = std::make_unique<Traits::Predictor>(
  maximal_dead_reckoning_elapsed_time,
  maximal_dead_reckoning_travelled_distance,
  maximal_position_circular_error_probability);

filter->register_predictor(std::move(predictor));

auto position_updater = std::make_unique<Traits::UpdaterPosition>(
  "position_updater",
  minimal_rate,
  trigger_mode,
  mahalanobis_distance_rejection_threshold,
  log_filename);

auto twist_updater = std::make_unique<Traits::UpdaterTwist>(
  "twist_updater",
  minimal_rate);

// ...
// Build observations from sensor data, keep updater objects alive,
// handle diagnostics, trigger policies and application-specific state.
// ...

auto twist_update = std::bind(
  &Traits::UpdaterTwist::update,
  twist_updater.get(),
  std::placeholders::_1,
  twist_observation,
  std::placeholders::_2,
  std::placeholders::_3);

filter->process(twist_observation_time, std::move(twist_update));

auto position_update = std::bind(
  &Traits::UpdaterPosition::update,
  position_updater.get(),
  std::placeholders::_1,
  position_observation,
  std::placeholders::_2,
  std::placeholders::_3);

filter->process(position_observation_time, std::move(position_update));

Traits::MetaState current_meta_state;
Traits::MetaStateToResults meta_state_to_results;
if (filter->get_state(query_time, &current_meta_state)) {
  const auto current_results = meta_state_to_results.convert(current_meta_state);
  const auto & current_pose = current_results.robot_pose;
  auto current_status = filter->get_fsm_state();
}
```

This example only shows the assembly principle. Real applications usually build observations inside sensor callbacks, keep updater objects alive and register several proprioceptive and exteroceptive updaters.

---

## Related packages

| Package | Role |
| ------- | ---- |
| `romea_core_filtering` | Generic Kalman and particle filtering layer. |
| `romea_core_localisation_gps` | GPS-to-localisation observation utilities. |
| `romea_core_localisation_imu` | IMU-to-localisation observation utilities. |
| `romea_core_localisation_rtls` | RTLS-to-localisation observation utilities. |

---

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

This library was developed by **Jean Laneurit** with scientific contributions from **Christophe Debain**, **Roland Chapuis** and **Romuald Aufrere**, in the context of several research projects including Baudet ROB, Baudet Rob 2 and Adap2E.
