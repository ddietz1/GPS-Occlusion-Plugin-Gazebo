# GPS-Denied Launch Demo

This models the RF GPS occlusion and multipath interference experienced by a
tube-launched UAV (such as a Starling drone) stored inside a metal launcher
prior to deployment, using two Gazebo plugins together:

- **[GPS_degradation](.)** - a transport relay. It subscribes to the raw
  Gazebo `NavSat` sensor topic, injects Gaussian noise and a deterministic
  bias into latitude/longitude/altitude, and republishes the degraded
  message onto the topic PX4's `EKF2` state estimator actually consumes.
  The degradation factor starts at `1.0` and decays linearly to `0.0` over a
  30s window after a "launch" (a velocity threshold) is detected.
- **[body_wrench](https://github.com/ddietz1/BodyWrench-Plugin-Gazebo)** -
  applies a one-shot body-frame force to a link, used here to simulate the
  launcher charge impulse.

Both are wired into the `x500_gps_denied` model
(`models/x500_gps_denied/model.sdf`) and demonstrated in the
`gps_denied_launch.sdf` world in
[PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models), used as the
`Tools/simulation/gz` submodule of `PX4-Autopilot`.

## Simulation workflow

- **Initialization & arming:** before launch, the drone rests inside an
  angled tube model. `GpsDegradationPlugin` initializes with a degradation
  factor of `1.0`. Because the injected noise causes high horizontal
  position variance, standard PX4 pre-flight GPS checks fail - arming
  requires bypassing them (`param set COM_ARM_WO_GPS 1`, or
  `commander arm -f` from the PX4 shell).
- **Launch trigger:** an instantaneous impulse force (5000 N along the
  drone's local Z-axis) simulates the launcher charge, via `body_wrench`.
  The plugin monitors the drone's linear speed; once it exceeds the launch
  velocity threshold, it triggers the recovery sequence.
- **Signal recovery:** the degradation factor decays linearly from `1.0` to
  `0.0` over a 30s window, modeling the time required for the receiver to
  reacquire clean line-of-sight satellite signals after clearing the tube.

## Observed flight behavior

During testing in HOLD mode, the drone exhibits significant erratic movement
upon launch. In HOLD mode, PX4 actively attempts to maintain a fixed spatial
coordinate; as the plugin injects artificial position noise, the EKF updates
the estimated position away from the target setpoint, and the position
controller interprets this as physical drift and commands thrust to correct
the false error - the drone aggressively "chases" the noise. As the
degradation factor decays toward zero, the estimated position converges back
to the drone's true position and the aircraft stabilizes into a stationary
hover. If the artificial position noise is high enough (e.g. jumping 15m in
a single step), the EKF's position innovation test ratio spikes past its
maximum value and the sample is rejected outright - the EKF then relies on
IMU/other onboard sensors until a more trustworthy GPS sample arrives.

## Running the demo

1. Install Gazebo Harmonic (gz-sim8) and a PX4-Autopilot checkout with the
   `Tools/simulation/gz` submodule pointed at a `PX4-gazebo-models` checkout
   that includes `models/x500_gps_denied` and `worlds/gps_denied_launch.sdf`.
2. Build both plugins and point Gazebo at them:
   ```bash
   cd body_wrench && mkdir build && cd build && cmake .. && make
   cd ../../GPS_degradation && mkdir build && cd build && cmake .. && make

   export GZ_SIM_SYSTEM_PLUGIN_PATH="/path/to/body_wrench/build:/path/to/GPS_degradation/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
   ```
3. Launch PX4 SITL with the demo world, attaching to the vehicle already
   included in the world file rather than PX4's normal auto-spawn (the
   world poses the vehicle inside the launch tube, which the auto-spawn
   path can't reproduce):
   ```bash
   cd /path/to/PX4-Autopilot
   PX4_GZ_WORLD=gps_denied_launch PX4_GZ_MODEL_NAME=x500_gps_denied_0 make px4_sitl gz_x500
   ```
4. From the PX4 shell, bypass the GPS pre-flight check and arm:
   ```
   param set COM_ARM_WO_GPS 1
   commander arm -f
   ```
5. From a separate terminal, trigger the launch impulse:
   ```bash
   gz topic -t "/model/x500_gps_denied_0/body_wrench" -m gz.msgs.Wrench -p 'force: {x: 0, y: 0, z: 5000}'
   ```

Watch the PX4 console for `[GpsDegradationPlugin] INITIALIZED - subscribing
to [...], publishing to [...]` at startup (confirms the SDF topic overrides
were picked up) and `[GpsDegradationPlugin] LAUNCH DETECTED at ...s` after
the wrench fires.
