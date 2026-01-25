#include "optimizer.h"
#include "esphome/components/ecodan/ecodan.h"

#define FLOW_TEMP_TOLERANCE 0.1f

using std::isnan;

namespace esphome {
namespace optimizer {

using namespace esphome::ecodan;

/**
 * @brief Construct a new Optimizer::Optimizer object.
 * 
 * @param state The initial state and configuration for the optimizer.
 */
Optimizer::Optimizer(OptimizerState state) : state_(state) {
    this->total_bias_sensor_ = state.total_bias_sensor;
    this->target_delta_t_sensor_ = state.target_delta_t_sensor;
    this->cold_factor_sensor_ = state.cold_factor_sensor;

    // Helper lambda to update a state variable and trigger a callback only if the new value has changed significantly.
    auto update_if_changed = [](float &storage, float new_val, auto callback) {
        if (std::isnan(new_val)) return;
        if (std::isnan(storage) || std::abs(storage - new_val) > 0.01f) {
            auto previous = storage;
            storage = new_val;
            callback(new_val, previous);
        }
    };

    // --- State Callbacks for real-time monitoring ---
    // Register callbacks for various sensors to react to changes in the heat pump's state in real-time.
    if (this->state_.hp_feed_temp != nullptr) {
        this->state_.hp_feed_temp->add_on_state_callback([this, update_if_changed](float x) {
            update_if_changed(this->last_hp_feed_temp_, x, [this](float new_v, float old_v) {
                this->on_feed_temp_change(new_v, OptimizerZone::SINGLE);
            });
        });
    }

    if (this->state_.z1_feed_temp != nullptr) {
        this->state_.z1_feed_temp->add_on_state_callback([this, update_if_changed](float x) {
            update_if_changed(this->last_z1_feed_temp_, x, [this](float new_v, float old_v) {
                this->on_feed_temp_change(new_v, OptimizerZone::ZONE_1);
            });
        });
    }

    if (this->state_.z2_feed_temp != nullptr) {
        this->state_.z2_feed_temp->add_on_state_callback([this, update_if_changed](float x) {
            update_if_changed(this->last_z2_feed_temp_, x, [this](float new_v, float old_v) {
                this->on_feed_temp_change(new_v, OptimizerZone::ZONE_2);
            });
        });
    }

    if (this->state_.operation_mode != nullptr) {
        this->state_.operation_mode->add_on_state_callback([this, update_if_changed](float x) {
            update_if_changed(this->last_operation_mode_, x, [this](float new_v, float old_v) {
                this->on_operation_mode_change(static_cast<uint8_t>(new_v), static_cast<uint8_t>(old_v));
            });
        });
    }

    if (this->state_.status_compressor != nullptr) {
        this->state_.status_compressor->add_on_state_callback([this, update_if_changed](float x) {
            update_if_changed(this->last_compressor_status_, x, [this](float new_v, float old_v) {
                this->on_compressor_state_change(static_cast<bool>(new_v), static_cast<bool>(old_v));
            });
        });
    }

    if (this->state_.status_defrost != nullptr) {
        this->state_.status_defrost->add_on_state_callback([this, update_if_changed](float x) {
            update_if_changed(this->last_defrost_status_, x, [this](float new_v, float old_v) {
                this->on_defrost_state_change(static_cast<bool>(new_v), static_cast<bool>(old_v));
            });
        });
    }

    // --- Load Integral Error from NVS (Preferences) ---
    // The integral term of the PID controller is saved to non-volatile storage (NVS)
    // to preserve it across reboots. This avoids having to re-learn the long-term
    // heating/cooling error of the system each time the device starts.
    this->integral_pref_ = esphome::global_preferences->make_preference<float>(esphome::fnv1_hash("optimizer.integral_error_z1"));
    float saved_integral = 0.0f;
    if (this->integral_pref_.load<float>(&saved_integral)) {
        this->integral_error_z1_ = saved_integral;
        this->last_saved_integral_ = saved_integral;
        ESP_LOGD(OPTIMIZER_TAG, "Loaded saved integral_error_z1_: %.3f", saved_integral);
    } else {
        ESP_LOGD(OPTIMIZER_TAG, "No saved integral_error_z1_, using default %.3f", this->integral_error_z1_);
    }
}

/**
 * @brief Processes the auto-adaptive logic for a single heating/cooling zone.
 * 
 * This function contains the core PID and Delta-T calculation logic. It determines the
 * ideal flow temperature for a zone based on the difference between the room's current
 * temperature and its target temperature.
 * 
 * @param i Zone index (0 for Zone 1, 1 for Zone 2).
 * @param status The current status report from the Ecodan heat pump.
 * @param cold_factor A normalized value (0-1) indicating how cold it is outside.
 * @param min_delta_cold_limit The minimum Delta-T to use in cold weather.
 * @param base_min_delta_t The absolute minimum Delta-T for the system type.
 * @param max_delta_t The absolute maximum Delta-T for the system type.
 * @param max_error_range The room temperature error range used for scaling the response.
 * @param actual_outside_temp The current outdoor temperature.
 * @param zone_max_flow_temp The maximum allowed flow temperature for this zone.
 * @param zone_min_flow_temp The minimum allowed flow temperature for this zone.
 * @param out_flow_heat Output parameter for the calculated heating flow temperature.
 * @param out_flow_cool Output parameter for the calculated cooling flow temperature.
 */
void Optimizer::process_adaptive_zone_(
    std::size_t i,
    const ecodan::Status &status,
    float cold_factor,
    float min_delta_cold_limit,
    float base_min_delta_t,
    float max_delta_t,
    float max_error_range,
    float actual_outside_temp,
    float zone_max_flow_temp,
    float zone_min_flow_temp,
    float &out_flow_heat,
    float &out_flow_cool) 
{
    // --- SMART GRID SAFETY: FREEZE PID CALCULATIONS ---
    // If the Smart Grid "OFF" mode is active (due to overheating), suspend all PID calculations
    // to prevent the optimizer from fighting the "OFF" state.
    if (this->state_.sg_mode_off != nullptr && this->state_.sg_mode_off->state) {
        ESP_LOGD(OPTIMIZER_TAG, "Z%d: Smart Grid OFF active. PID calculations suspended.", (i + 1));
        return; 
    }

    auto zone = (i == 0) ? esphome::ecodan::Zone::ZONE_1 : esphome::ecodan::Zone::ZONE_2;
    auto heating_type_index = this->state_.heating_system_type->active_index().value_or(0);

    // Determine if the zone is in heating or cooling mode and if the heat pump is actively running.
    bool is_heating_mode = status.is_auto_adaptive_heating(zone);
    bool is_heating_active = status.Operation == esphome::ecodan::Status::OperationMode::HEAT_ON;
    bool is_cooling_mode = status.has_cooling() && status.is_auto_adaptive_cooling(zone);
    bool is_cooling_active = status.Operation == esphome::ecodan::Status::OperationMode::COOL_ON;

    // Handle multi-zone activity status. The logic here determines if a specific zone
    // is actively being heated in a two-zone system.
    if (is_heating_active && status.has_2zones()) {
        auto multizone_status = status.MultiZoneStatus;
        if (i == 0 && (multizone_status == 0 || multizone_status == 3)) {
            is_heating_active = false;
        } else if (i == 1 && (multizone_status == 0 || multizone_status == 2)) {
            is_heating_active = false;
        }
        if (status.has_independent_z2() && (status.WaterPump2Active || status.WaterPump3Active))
            is_heating_active = true;
    }

    float setpoint_bias = this->state_.auto_adaptive_setpoint_bias->state;
    if (isnan(setpoint_bias)) setpoint_bias = 0.0f;

    // Get the relevant temperatures for the current zone.
    float room_temp = (i == 0) ? status.Zone1RoomTemperature : status.Zone2RoomTemperature;
    float room_target_temp = (i == 0) ? status.Zone1SetTemperature : status.Zone2SetTemperature;
    float requested_flow_temp = (i == 0) ? status.Zone1FlowTemperatureSetPoint : status.Zone2FlowTemperatureSetPoint;
    float actual_flow_temp = status.has_independent_z2() ? ((i == 0) ? status.Z1FeedTemperature : status.Z2FeedTemperature) : status.HpFeedTemperature;
    float actual_return_temp = status.has_independent_z2() ? ((i == 0) ? status.Z1ReturnTemperature : status.Z2ReturnTemperature) : status.HpReturnTemperature;

    // If the zone is not in auto-adaptive mode, reset the integral term to zero.
    // This prevents the integral from accumulating error when the optimizer is not in control.
    if (!is_heating_mode && !is_cooling_mode) {
        if (i == 0) { // Integral is only tracked for Zone 1
            this->integral_error_z1_ = 0.0f;
            if (!std::isnan(this->last_saved_integral_)) {
                this->integral_pref_.save<float>(&this->integral_error_z1_);
                this->last_saved_integral_ = this->integral_error_z1_;
            }
        }
        return;
    }

    // Allow using an external temperature sensor as the feedback source instead of the Ecodan's sensor.
    if (this->state_.temperature_feedback_source->active_index().value_or(0) == 1) {
        room_temp = (i == 0) ? this->state_.temperature_feedback_z1->state : this->state_.temperature_feedback_z2->state;
    }

    // Exit if any of the required temperature readings are invalid.
    if (isnan(room_temp) || isnan(room_target_temp) || isnan(requested_flow_temp) || isnan(actual_flow_temp))
        return;

    float total_bias = setpoint_bias;

    // --- Zone 1 specific PID logic (Master Zone) ---
    // The full PID controller is only applied to Zone 1, which is considered the master zone.
    if (i == 0) {
        // The "raw" or proportional error is the direct difference between target and actual room temperature.
        float raw_error = room_target_temp - room_temp;
        uint32_t now = millis();

        // --- Derivative Term ---
        // Calculates the rate of temperature change. This can help predict future temperature
        // and react more quickly, but it's sensitive to noise. Currently disabled (DERIVATIVE_GAIN = 0).
        if (this->last_derivative_time_ != 0 && !std::isnan(this->last_room_temp_z1_)) {
            float dt = (now - this->last_derivative_time_) / 1000.0f;
            if (dt > 60.0f && dt < 900.0f) { // Only calculate if time delta is reasonable
                float temp_diff = room_temp - this->last_room_temp_z1_;
                float slope = temp_diff / dt;
                const float DERIVATIVE_GAIN = 0.0f; // Currently disabled
                this->derivative_term_ = slope * DERIVATIVE_GAIN;
            }
        }
        this->last_room_temp_z1_ = room_temp;
        this->last_derivative_time_ = now;

        // --- Integral Term ---
        // Accumulates the error over time. This helps correct for long-term, steady-state errors
        // where the proportional term alone isn't enough to reach the target.
        // Includes anti-windup logic to prevent the integral from growing excessively large.
        if (is_heating_active || is_cooling_active) {
            float integration_step = raw_error * INTEGRAL_GAIN;
            bool apply_step = true;

            // Don't apply integral correction if the error is already large (deadband).
            if (std::abs(raw_error) > 0.5f) {
                apply_step = false;
            }

            // Anti-windup: Check if applying the next integration step would cause the
            // target Delta-T to exceed its maximum or minimum limits. If so, don't apply the step.
            const float DELTA_LIMIT_EPS = 0.01f;
            float prospective_integral = this->integral_error_z1_ + integration_step;
            float prospective_error = raw_error + prospective_integral;
            float prospective_error_sign = (prospective_error > 0.0f) ? 1.0f : ((prospective_error < 0.0f) ? -1.0f : 0.0f);
            float prospective_x = fmin(fabs(prospective_error) / max_error_range, 1.0f);
            bool use_linear_error_local = (heating_type_index % 2 != 0);
            float prospective_profile = use_linear_error_local ? prospective_x : prospective_x * prospective_x * (3.0f - 2.0f * prospective_x);
            float prospective_error_factor = prospective_profile * prospective_error_sign;
            float prospective_dynamic_min_delta_t = base_min_delta_t + (cold_factor * (min_delta_cold_limit - base_min_delta_t));
            float prospective_target_delta_t = prospective_dynamic_min_delta_t + prospective_error_factor * (max_delta_t - prospective_dynamic_min_delta_t);

            if (prospective_target_delta_t >= (max_delta_t - DELTA_LIMIT_EPS) && integration_step > 0) apply_step = false;
            else if (prospective_target_delta_t <= (base_min_delta_t + DELTA_LIMIT_EPS) && integration_step < 0) apply_step = false;

            // Further anti-windup: Block integration if the flow temperature is already at its limits.
            if (is_heating_mode) {
                if (actual_flow_temp >= zone_max_flow_temp) {
                    ESP_LOGD(OPTIMIZER_TAG, "Z%d: Max flow temp reached (%.1f >= %.1f). Integrator step blocked.", (i + 1), actual_flow_temp, zone_max_flow_temp);
                    apply_step = false;
                }
                else if (integration_step > 0 && actual_flow_temp >= (zone_max_flow_temp - 1.0f)) apply_step = false;
                else if (integration_step < 0 && actual_flow_temp <= (zone_min_flow_temp + 1.0f)) apply_step = false;
            }

            // If all checks pass, apply the integration step.
            if (apply_step) {
                this->integral_error_z1_ += integration_step;
                this->integral_error_z1_ = std::clamp(this->integral_error_z1_, -MAX_INTEGRAL_BIAS, MAX_INTEGRAL_BIAS);
                // Save the new integral value to NVS if it has changed.
                if (std::isnan(this->last_saved_integral_) || std::fabs(this->integral_error_z1_ - this->last_saved_integral_) > 0.01f) {
                    this->integral_pref_.save<float>(&this->integral_error_z1_);
                    this->last_saved_integral_ = this->integral_error_z1_;
                }
            }
        }
        // The total bias is the sum of the manual bias, the integral term, and the derivative term.
        total_bias += this->integral_error_z1_ - this->derivative_term_;
        this->total_bias_ = total_bias;
        ESP_LOGD(OPTIMIZER_TAG, "Z1 PID: RawErr=%.3f, IntErr=%.3f, Deriv=%.3f, TotalBias=%.3f", raw_error, this->integral_error_z1_, this->derivative_term_, total_bias);
        if (this->total_bias_sensor_ != nullptr) this->total_bias_sensor_->publish_state(total_bias);
    }

    // --- Delta T Calculation using S-Curve or Linear profiles ---
    // This section translates the final temperature error (including bias) into a target
    // Delta-T (the desired temperature difference between flow and return water).
    // An S-curve (smoothstep) or linear profile is used to map the error to the Delta-T range.
    room_target_temp += total_bias; // Apply the calculated bias to the target temperature.
    float error = is_heating_mode ? (room_target_temp - room_temp) : (room_temp - room_target_temp);
    float error_sign = (error > 0.0f) ? 1.0f : ((error < 0.0f) ? -1.0f : 0.0f);
    float x = fmin(fabs(error) / max_error_range, 1.0f); // Normalize error to [0, 1]
    // Use linear or S-curve profile based on heating system type
    float profile = (heating_type_index % 2 != 0) ? x : x * x * (3.0f - 2.0f * x); 
    // Calculate the final target Delta-T
    float target_delta_t = fmax(base_min_delta_t + (cold_factor * (min_delta_cold_limit - base_min_delta_t)) + (profile * error_sign) * (max_delta_t - (base_min_delta_t + (cold_factor * (min_delta_cold_limit - base_min_delta_t)))), base_min_delta_t);
    ESP_LOGD(OPTIMIZER_TAG, "Z%d DeltaT: Err=%.3f, Sign=%.1f, X=%.3f, Profile=%.3f, BaseMin=%.1f, ColdFact=%.3f, MinColdLim=%.1f, MaxDT=%.1f, TargetDT=%.3f", (i + 1), error, error_sign, x, profile, base_min_delta_t, cold_factor, min_delta_cold_limit, max_delta_t, target_delta_t);

    if (i == 0) { // Store and publish the Delta-T for Zone 1
        this->target_delta_t_ = target_delta_t;
        if (this->target_delta_t_sensor_ != nullptr) this->target_delta_t_sensor_->publish_state(this->target_delta_t_);
    }

    // --- Final Target Flow calculation ---
    // Calculates the final flow temperature setpoint to be sent to the heat pump.
    if (is_heating_mode && is_heating_active) {
        if (isnan(actual_return_temp)) {
            // Fallback to minimum flow temp if return temp is unknown
            out_flow_heat = zone_min_flow_temp;
        } else {
            // Target flow temp = return temp + target Delta-T
            out_flow_heat = this->round_nearest(actual_return_temp + target_delta_t);
        }
        // Ensure the calculated flow temperature is within the allowed min/max range for the zone.
        out_flow_heat = this->clamp_flow_temp(enforce_step_down(actual_flow_temp, out_flow_heat), zone_min_flow_temp, zone_max_flow_temp);
    } else if (is_cooling_mode && is_cooling_active) {
        // Similar logic for cooling mode (subtracting Delta-T).
        out_flow_cool = this->round_nearest_half(this->clamp_flow_temp(actual_return_temp - target_delta_t, this->state_.minimum_cooling_flow_temp->state, this->state_.cooling_smart_start_temp->state));
    }
}

/**
 * @brief The main entry point for the optimizer's control loop.
 * 
 * This function is called periodically. It gathers all necessary data, determines
 * system parameters, and then calls process_adaptive_zone_ for each active zone
 * to calculate and set the required flow temperatures. It also contains the
 * Smart Grid logic.
 */
void Optimizer::run_auto_adaptive_loop() {
    if (this->total_bias_sensor_ != nullptr) this->total_bias_sensor_->publish_state(this->total_bias_);
    if (!this->state_.auto_adaptive_control_enabled->state) return;

    auto &status = this->state_.ecodan_instance->get_status();
    if (this->is_system_hands_off(status)) return;

    float actual_outside_temp = status.OutsideTemperature;
    if (isnan(this->state_.hp_feed_temp->state) || isnan(actual_outside_temp)) return;

    // --- System Type Selection & Parameters ---
    // Selects a set of parameters (min/max Delta-T, etc.) based on the configured
    // type of heating system (e.g., underfloor, radiators).
    auto heating_type_index = this->state_.heating_system_type->active_index().value_or(0);
    float base_min_delta_t, max_delta_t, max_error_range, min_delta_cold_limit;

    if (heating_type_index <= 1) { // Underfloor
        base_min_delta_t = 2.3f; min_delta_cold_limit = 4.0f; max_delta_t = 8.0f; max_error_range = 2.0f; 
    } else if (heating_type_index <= 3) { // Radiators
        base_min_delta_t = 3.0f; min_delta_cold_limit = 5.0f; max_delta_t = 8.0f; max_error_range = 2.0f; 
    } else { // Fan Coils
        base_min_delta_t = 2.9f; min_delta_cold_limit = 8.0f; max_delta_t = 11.0f; max_error_range = 3.0f; 
    }

    // Calculate a "cold factor" based on the outside temperature. This factor is used
    // to dynamically adjust the minimum Delta-T, increasing it in colder weather.
    float cold_factor = (15.0f - std::clamp(actual_outside_temp, -10.0f, 15.0f)) / 25.0f;
    this->cold_factor_ = cold_factor;
    if (this->cold_factor_sensor_ != nullptr) this->cold_factor_sensor_->publish_state(this->cold_factor_);

    ESP_LOGD(OPTIMIZER_TAG, "Cycle Start: Outside %.1f°C, Cold Factor %.2f, Mode Index %d", actual_outside_temp, cold_factor, heating_type_index);

    float calculated_flows_heat[2] = {0.0f, 0.0f};
    float calculated_flows_cool[2] = {100.0f, 100.0f};
    auto max_zones = status.has_2zones() ? 2 : 1;

    // --- Main Zone Processing Loop ---
    // Iterate through each zone and call the processing function to calculate its target flow temp.
    for (std::size_t i = 0; i < max_zones; i++) {
        this->process_adaptive_zone_(i, status, cold_factor, min_delta_cold_limit, base_min_delta_t, max_delta_t, max_error_range, actual_outside_temp, 
            (i == 0) ? this->state_.maximum_heating_flow_temp->state : this->state_.maximum_heating_flow_temp_z2->state,
            (i == 0) ? this->state_.minimum_heating_flow_temp->state : this->state_.minimum_heating_flow_temp_z2->state,
            calculated_flows_heat[i], calculated_flows_cool[i]);
    }

    // --- SMART GRID SAFETY: BLOCK COMMAND EMISSION ---
    // If the Smart Grid "OFF" mode is active, do not send any new flow temperature commands to the Ecodan.
    bool sg_off_active = (this->state_.sg_mode_off != nullptr && this->state_.sg_mode_off->state);

    if (!sg_off_active) {
        if (status.has_independent_z2()) { // Independent zones
            if (calculated_flows_heat[0] > 0.0f && std::abs(status.Zone1FlowTemperatureSetPoint - calculated_flows_heat[0]) > FLOW_TEMP_TOLERANCE)
                set_flow_temp(calculated_flows_heat[0], OptimizerZone::ZONE_1);
            if (calculated_flows_heat[1] > 0.0f && std::abs(status.Zone2FlowTemperatureSetPoint - calculated_flows_heat[1]) > FLOW_TEMP_TOLERANCE)
                set_flow_temp(calculated_flows_heat[1], OptimizerZone::ZONE_2);
        } else { // Single or combined zones
            float final_flow = fmax(calculated_flows_heat[0], calculated_flows_heat[1]);
            if (final_flow > 0.0f && std::abs(status.Zone1FlowTemperatureSetPoint - final_flow) > FLOW_TEMP_TOLERANCE)
                set_flow_temp(final_flow, OptimizerZone::SINGLE);
        }
    } else {
        ESP_LOGD(OPTIMIZER_TAG, "Smart Grid OFF active: Ecodan flow commands blocked.");
    }

    // --- SMART GRID LOGIC: Always evaluated to allow exiting OFF mode ---
    // This logic manages two Smart Grid modes:
    // 1. Recommendation ("reco") Mode: Activates when there is surplus energy (e.g., from solar panels)
    //    to pre-heat the house by a small amount, storing energy in the building's thermal mass.
    // 2. OFF Mode: A safety feature that forces the heat pump off if the room temperature significantly
    //    exceeds the setpoint, preventing overheating.
    if (this->state_.sg_overheat_offset != nullptr && this->state_.sg_mode_off != nullptr &&
        this->state_.sg_mode_reco != nullptr && this->state_.sg_energy_available != nullptr) {

        ESP_LOGD(OPTIMIZER_TAG, "Smart Grid: Checking conditions...");
        bool energy_available = this->state_.sg_energy_available->state;
        float storage_offset_val = (this->state_.sg_storage_offset != nullptr) ? this->state_.sg_storage_offset->state : 2.0f;
        float overheat_offset_val = this->state_.sg_overheat_offset->state;

        float current_room = (this->state_.temperature_feedback_source->active_index().value_or(0) == 1) ? 
                              this->state_.temperature_feedback_z1->state : status.Zone1RoomTemperature;
        float current_target = status.Zone1SetTemperature;

        // Condition to activate "reco" (storage) mode: energy is available and room temp is below the extended target.
        bool activate_reco = energy_available && (current_room < (current_target + storage_offset_val));
        
        // --- Handle Recommendation Mode (Storage) ---
        if (activate_reco != this->state_.sg_mode_reco->state) {
            activate_reco ? this->state_.sg_mode_reco->turn_on() : this->state_.sg_mode_reco->turn_off();
        }

        if (activate_reco) {
            // If we are in "reco" mode, make sure "OFF" mode is disabled.
            if (this->state_.sg_mode_off->state) this->state_.sg_mode_off->turn_off();
        } else {
            // --- Handle OFF Mode (Overheat Protection) ---
            // This logic runs if the conditions for "reco" mode are not met.
            bool hp_running = (status.Operation == esphome::ecodan::Status::OperationMode::HEAT_ON) || 
                              (status.Operation == esphome::ecodan::Status::OperationMode::COOL_ON);
            
            // Condition to trigger "OFF" mode: room is too hot, HP is running, water is warm, and heating demand is at minimum.
            bool condition_trigger_off = current_room > (current_target + overheat_offset_val) &&
                                        hp_running &&
                                        (status.HpReturnTemperature >= 26.0f) &&
                                        (std::abs(this->target_delta_t_ - base_min_delta_t) < 0.01f);
            
            // Condition to release "OFF" mode: room temperature has returned to the setpoint.
            bool condition_release_off = (current_room <= current_target);
            
            if (condition_trigger_off && !this->state_.sg_mode_off->state) {
                ESP_LOGI(OPTIMIZER_TAG, "Smart Grid: Overheat detected (room %.1f > %.1f, return %.1f). Turning OFF.", current_room, (current_target + overheat_offset_val), status.HpReturnTemperature);
                this->state_.sg_mode_off->turn_on();
            } else if (condition_release_off && this->state_.sg_mode_off->state) {
                ESP_LOGI(OPTIMIZER_TAG, "Smart Grid: Temperature normalized (%.1f <= %.1f). Releasing OFF.", current_room, current_target);
                this->state_.sg_mode_off->turn_off();
            }
        }
    }
}

} // namespace optimizer
} // namespace esphome
