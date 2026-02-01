# Auto-Adaptive Control

This document describes how the "Auto-Adaptive" control mode for the Ecodan heat pump works. This feature aims to optimize comfort and energy efficiency by dynamically adjusting the heating water temperature based on the actual needs of your home.

The algorithm is based on a **Delta-T control** principle defined by Gekkekoe driven by a **PID controller**, which allows for finer and more responsive regulation than a simple thermostat or a basic weather compensation curve.

**Important Note:** For this mode to work correctly, the heat pump must be set to **"Fixed Flow Temperature"** mode for heating.

## Principles of Operation

### Delta-T Control

The core of the auto-adaptive strategy is not to impose a fixed flow temperature but to calculate it in real-time. To do this, the algorithm measures the return water temperature from the heating circuit and adds a calculated temperature difference, called "Delta-T".

`Target Flow Temperature = Return Temperature + Target Delta-T`

The `Target Delta-T` is not constant. It is dynamically calculated by the PID controller to precisely meet the heating demand of the house.

### The PID Controller (Proportional, Integral, Derivative)

The PID is an intelligent control mechanism that constantly adjusts the `Target Delta-T` based on the error between the desired room temperature (your setpoint) and the actual room temperature.

- **Proportional Term (P):** This is the direct response to the current error. If your room is at 19.5°C for a 20°C setpoint, the -0.5°C error generates a base Delta-T. This is the main reaction force.

- **Integral Term (I):** It corrects for persistent, long-term errors. If, despite the P term, the temperature stagnates at 19.8°C without ever reaching 20°C (a "steady-state" error), the integrator will gradually increase the Delta-T to close this gap.
    - **Anti-Windup:** Safeguards ("anti-windup") prevent this term from growing indefinitely, especially when the heat pump is already at its maximum or minimum power, thus avoiding unstable behavior.
    - **Persistence:** The value of this term is saved in non-volatile storage (NVS). This means the optimizer does not need to "re-learn" the thermal behavior of your home after a reboot.

- **Derivative Term (D):** It anticipates the future by looking at the rate of temperature change. It can speed up the response if the temperature drops quickly. *Note: In the current implementation, this term is present but disabled by default (`DERIVATIVE_GAIN = 0`).*

The `total_bias` visible in the logs is the sum of the actions of these terms and is used to adjust the internal setpoint before the final Delta-T calculation.

### Heating Profiles: S-Curve vs. Linear

The way the temperature error is converted into a Delta-T depends on the selected **heating system type**. This allows the system's responsiveness to be adapted.

- **Gentle Profile / S-Curve (Smoothstep):** Ideal for high-inertia systems like **underfloor heating**. The response is very gentle for small errors and intensifies non-linearly as the error increases. This prevents jolts and promotes stability.

- **Responsive Profile / Linear:** Recommended for low-inertia systems like **radiators**. The response is directly proportional to the error, allowing for a faster temperature rise.

The profile is chosen via the `heating_system_type`

## Advanced Features

### The "Setpoint Bias"

The `auto_adaptive_setpoint_bias` parameter allows you to manually apply a "bias" to the temperature setpoint.

### "Smart Grid" Logic

This optional feature allows interaction with external signals (like surplus solar production) for intelligent energy management. The `sg_mode_reco` and `sg_mode_off` switches are exposed to Home Assistant. They serve as signals for automations to activate the corresponding physical inputs (IN11 and IN12) on the heat pump, thus enabling its native smart grid capabilities.

- **Recommendation Mode (Energy Storage):**
    - **Condition:** An external switch (`sg_energy_available`) indicates a surplus of energy.
    - **Action:** The system activates a "recommendation" mode (`sg_mode_reco`) which aims to slightly overheat the house to store this free energy in the building's thermal mass. The limit for this overheating is defined by `sg_storage_offset`.

- **OFF Mode (Overheat Protection):**
    - **Condition:** The room temperature exceeds the setpoint by an adjustable value (defined by `sg_overheat_offset`), the return water is already warm, and the heating demand is minimal.
    - **Action:** The system forces the compressor to shut down (`sg_mode_off`) to prevent uncomfortable overheating and energy waste. The compressor is automatically re-enabled when the temperature returns to normal.

## Configuration Parameters

All parameters are adjustable in real-time from the Home Assistant interface.

- **`Auto-Adaptive: Control`:** Enables or disables the entire feature.
- **`Auto-Adaptive: Heating System Type`:** Selects the heating profile (Linear or S-Curve) suitable for your installation (radiators, underfloor heating, hybrid).
- **`Auto-Adaptive: Temperature Feedback Source`:** Allows you to choose whether the reference room temperature is the one read by the heat pump or an external value (e.g., another temperature sensor in Home Assistant).
- **`Auto-Adaptive: Setpoint Bias`:** Allows for manual correction of the setpoint (see above).
- **`Max. / Min. Heating Flow Temperature`:** High and low safety limits for the flow water temperature. The algorithm will never exceed these values.
- **`Base Min Delta T` / `Max Delta T`:** Defines the working range (in °C) for the `Target Delta-T`.
- **`Integral Gain`:** Adjusts the aggressiveness of the PID's Integral term. The default value is 0.05. This should be reduced for high-inertia systems (e.g., underfloor heating) to prevent overshoot and instability. A higher value will correct long-term errors faster on responsive systems.
- **`SG Overheat Offset` / `SG Storage Offset`:** Adjusts the thresholds in °C for the Smart Grid logic.
- **`SG Energy Available` / `SG Mode Off` / `SG Mode Reco`:** Switches for integration with the Smart Grid logic.

## Getting Started

1**Configure the temperature source:** Via `temperature_feedback_source`, choose whether you are using the heat pump's sensor or a more accurate external sensor. For best performance, use a sensor with a resolution of 0.1°C or 0.2°C. A sensor with a 0.5°C resolution (like the main remote controller) will introduce correction delays, which can be detrimental to comfort and stability.
2.  **Set the safety temperatures:** Adjust `maximum_heating_flow_temp` and `minimum_heating_flow_temp` to reasonable and safe values for your system.
3.  **Set Delta-T range:** Adjust `Base Min Delta T` and `Max Delta T`. The minimum should be based on your observation of the system's lowest stable Delta T. The maximum should then be set to approximately 4 times the minimum value to cover the heat pump's full operating range.
4.  **Choose the heating profile:** Select the `heating_system_type` that best matches your installation.
5.  **Tune Integral Gain:** Start with the default `Integral Gain` of 0.05. For high-inertia systems (e.g., underfloor heating), you may need to reduce this value to prevent temperature overshooting the setpoint.
6.  **Enable control:** Turn the `auto_adaptive_control_enabled` switch to ON.
7.  **Monitor and adjust:** Observe the system's behavior. You can monitor the `Target Delta T` and `Total Bias` sensors to understand the algorithm's decisions. If necessary, adjust the other gains or biases to fine-tune comfort.
