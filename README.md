# Physics Simulator

An interactive, modular physics simulator written in **C++ with raylib**. Describe a motion experiment in JSON, run it in the built-in editor, and inspect its numerical trajectory, physical quantities, and target contacts.

The project supports STEM learning by making physical models and numerical error visible. The physics engine runs independently of any AI service; natural-language command generation remains future work.

## Features

- Seven configurable kinematic models for the primary particle and moving targets.
- Consecutive motion phases on one physical object, with continuous position and velocity.
- Explicit Euler, RK2 Explicit Midpoint, and Classical RK4, selectable per phase.
- Static, moving, phase-local, and shared sequence targets.
- Swept target detection, HIT/MISS feedback, and a projectile launch solver for static targets.
- Numerical trajectories, measurement guides, velocity and acceleration vectors, and automatic viewport fitting.
- Global timeline, Play/Pause, Restart, and numerical seek/replay.
- JSON validation with status and error messages in the application.

## Supported Motion Models

Motion models are configurations of `Kinematics`, rather than separate classes for each phenomenon. Use these exact, case-sensitive JSON identifiers:

| Model | JSON `phenomenon` | Description |
| --- | --- | --- |
| Uniform Motion / MRU | `uniform_motion` | Constant velocity. |
| Uniformly Accelerated Motion / MRUA | `uniformly_accelerated_motion` | Constant acceleration parallel to the velocity; starting from rest is supported. |
| Free Fall | `free_fall` | Vertical fall from rest under gravity. |
| Inclined Plane Motion | `inclined_plane_motion` | Motion along a slope with acceleration derived from its angle and gravity. |
| Projectile Motion | `projectile_motion` | Two-dimensional motion under constant downward gravity. |
| Uniform Circular Motion / MCU | `uniform_circular_motion` | State-dependent centripetal acceleration. |
| Uniformly Accelerated Circular Motion / MCUA | `uniformly_accelerated_circular_motion` | State-dependent centripetal and tangential acceleration. |

World coordinates use meters, seconds, kilograms, and positive Y upward. JSON fields ending in `_degrees`, and Angle termination values, use degrees. Angular velocity and acceleration use radians per second and radians per second squared.

For circular initialization, `circular_motion` defines `center`, `radius`, `initial_angle_degrees`, and `angular_velocity`; MCUA also requires `angular_acceleration`. The initial position and velocity are derived from these parameters. Subsequent circular phases inherit the actual incoming state and validate compatibility with their configured center.

## Motion Sequences

A `motion_sequence` runs multiple physical regimes on the **same `PhysicalObject`**. For example:

```text
MCUA → MRUA → MRU → Projectile
```

Position, velocity, and mass carry across phase boundaries. Acceleration changes with the active model. The numerical trajectory and timeline remain global; each phase has its own elapsed time, traveled distance, and exit condition.

Every phase requires `phenomenon` and `until`. MRUA phases specify a new `acceleration`, either as vector components or as `{"mode": "along_velocity", "value": 2.0}`. The latter resolves a constant acceleration along the velocity inherited when the phase starts.

When an event ends a phase partway through a physical step, the remaining time advances the next phase with its own model and integrator. A sequence supports up to 64 phases. The parser rejects incompatible transitions, unreachable termination conditions, and unsupported reversals.

The single-phenomenon format remains supported and is internally normalized to a one-phase sequence. Use either root `phenomenon` with `simulation.termination`, or `motion_sequence` with phase `until` fields; do not combine the two formats.

## Termination Conditions

Within `motion_sequence`, each phase's `until` determines when it advances to the next phase. The legacy single-phenomenon format uses the same names under `simulation.termination`.

| JSON `type` | Parameters | Meaning |
| --- | --- | --- |
| `natural` | No value required | End at the model's natural boundary, such as ground impact. |
| `time` | Positive `value` | End after this many physical seconds in the phase. |
| `distance` | Positive `value` | End after this much traveled path length, in meters. |
| `speed` | Nonnegative `value` | End at the requested speed, including zero where physically supported. |
| `angle` | Positive `value` | End after this accumulated angular travel, in degrees. |
| `target` | Positive `max_time` | End at the first accepted target contact, with a time limit. |

Support depends on the model. MRU requires an explicit time, distance, or target condition. Free Fall and Projectile support natural/time/target termination. MRUA and Inclined Plane support time/distance/speed/target. Circular motion requires an explicit condition; Speed termination is supported for MCUA, not MCU.

For example, an Angle condition with `value: 360` means one revolution of accumulated angular travel from the phase's start, rather than an absolute angular position of 360 degrees.

A Target condition requires an active target. If its time limit or a natural end is reached without contact, the sequence reports MISS and stops instead of advancing to the next phase.

## Targets

A target defines an acceptance circle in world coordinates. Static targets use `position`; moving targets add `motion` and reuse `PhysicalObject` plus `Kinematics`. All seven motion models are available for target motion.

Two target lifetimes are supported:

- **Phase-local:** place `target` inside a phase. Only the active phase's target is simulated and drawn. Previous targets disappear, future targets remain hidden, and each new target starts from its own configured state.
- **Shared:** place one root `target` alongside `motion_sequence`. Its state continues across phases. A shared root target cannot be combined with phase-local targets.

With `until.type: "target"`, the first accepted contact ends the phase. With another termination condition, contact records HIT while the phase continues. A moving target that reaches its natural end remains stationary afterward.

`radius` is a physical acceptance radius, not a display-only size. Optional `crossing` values are `any`, `ascending`, and `descending`; the directional filter uses the primary particle's vertical velocity in world coordinates.

`TargetDetector` uses swept collision detection in relative motion, allowing contact detection when both bodies move between numerical samples. The first contact is interpolated within the step. This operates on numerical segments and does not make collision localization a fourth-order calculation when RK4 is selected.

Primary and active target use the same phase integrator and physical step. A shared target changes numerical method with the phase while preserving its state. Target-specific integrators and target motion sequences are not supported.

### Projectile Launch Solver

`ProjectileSolver` computes launch conditions for **static targets** through `target_solution`:

| `mode` | Required input | Solved quantity |
| --- | --- | --- |
| `known_speed` | `speed` | Launch angle; select `arc: "low"` or `"high"` when applicable. |
| `known_angle` | `angle_degrees` | Launch speed. |
| `known_time` | `time` | Launch velocity for the specified arrival time. |

If two valid known-speed branches exist, select an arc explicitly. The solver checks crossing and ground constraints. It prepares initial conditions; subsequent evolution is numerical. Moving targets with `target_solution` are unsupported, and sequence launch solving is restricted to the first phase.

## Numerical Integration

`Kinematics` determines acceleration from position and velocity. `Integrator` determines how that state advances, using an acceleration evaluator:

| JSON integrator | Algorithm | Order |
| --- | --- | --- |
| `euler` | Explicit Euler | 1 |
| `rk2` | Explicit Midpoint | 2 |
| `rk4` | Classical RK4 | 4 |

The effective method is resolved as **phase `integrator` → `simulation.integrator` → Euler**. A phase without an override uses the simulation default, not the previous phase's method. The information panel shows the active method.

Euler advances position with the old velocity and velocity with acceleration at the old state. RK2 and RK4 reevaluate acceleration at hypothetical intermediate states without changing the real object during those stages. In particular, circular acceleration is recalculated from each intermediate position and velocity.

Explicit sequences use a fixed physical step of `1/240 s`, with event boundaries consuming partial steps. Single-phenomenon playback retains its existing stepping policy, including circular substeps no larger than `1/240 s`. Selecting RK4 does not increase the allowed step size. `playback_duration` controls playback pacing, not a physical termination time.

Position and velocity evolve numerically rather than being replaced by analytical trajectories. Analytical calculations support reference metrics, initial launch solving, duration estimates, validation, and visual guides. Numerical event localization and ground-boundary handling determine the actual phase endpoints.

Euler can produce visible **radial drift** in MCU and MCUA. The particle is never projected back onto the configured circle. The drawn circle is an ideal guide; the trail shows the actual numerical positions. Changing integrator preserves existing drift. RK methods can reduce observed error, but their results remain subject to step size, floating-point arithmetic, and event localization. Circular stopping does not artificially erase residual radial velocity.

### Preflight, Timeline, and Replay

Before an explicit motion sequence starts, preflight runs the same numerical engine, phase methods, targets, and event rules to validate dynamic transitions, determine global duration, and collect viewport bounds.

Play/Pause controls playback. Restart restores the initial phase, method, targets, counters, and trajectory history. Timeline seeking reconstructs the numerical evolution from the start through each preceding phase; it does not place the particle using analytical position formulas.

## Visualization

The primary particle has a numerical trajectory trail and velocity/acceleration vectors. Guides depend on the active model:

| Motion | Visual references |
| --- | --- |
| Linear diagonal motion | Path length `s`, horizontal `Δx`, and vertical `Δy`. |
| Inclined Plane | `s`, `Δx`, `Δy`, and a geometric angle guide. |
| Projectile | Horizontal/vertical references, trajectory, and analytical launch/flight metrics. |
| Circular | Ideal circle, center, radius, numerical trajectory, and circular metrics. |

Circular metrics include configured/current radius, radial error, angle traveled, angular velocity, and centripetal/tangential acceleration. The viewport fits the experiment's bounds.

Targets are deliberately simple acceptance circles with center markers and contact status. They have no measurement scales, angle/circular guides, trajectory trails, velocity vectors, or acceleration vectors.

## Architecture

```text
JSON
 |
 v
CommandParser ----> ProjectileSolver (static-target launch preparation)
 |
 v
SimulationConfig
 |
 v
Simulation
 |
 +--> MotionPhase
 |      +--> Kinematics configuration
 |      +--> until
 |      +--> effective IntegratorType
 |      +--> optional phase-local Target
 |
 +--> Kinematics (acceleration from state)
 +--> Integrator (Euler / RK2 Midpoint / RK4)
 +--> PhysicalObject (primary and active target state)
 +--> TargetDetector (relative swept contacts)
 |
 v
Renderer (raylib visualization)
```

`Simulation` coordinates phases, numerical stepping, events, targets, preflight, and replay. `Vector2D` represents physical vectors; `Scale` and guide structures describe measurements. `CommandEditor` handles JSON text editing. The numerical algorithms do not contain phenomenon-specific physics or target logic.

## JSON Examples

Paste a complete command into the application editor and click **Run Command**. Use **Edit Command** to return to the editor.

### Single-Phenomenon MRU

A particle travels at 5 m/s for 5 physical seconds. This demonstrates the supported legacy format.

```json
{
  "phenomenon": "uniform_motion",
  "object": {
    "mass": 1.0,
    "position": {"x": 0.0, "y": 0.0},
    "velocity": {"x": 5.0, "y": 0.0}
  },
  "simulation": {
    "integrator": "euler",
    "termination": {"type": "time", "value": 5.0},
    "playback_duration": 5.0
  }
}
```

### Continuous Sequence with Per-Phase Methods

MCUA initializes the particle above ground. MRUA accelerates along the inherited velocity, MRU continues for 10 meters, and Projectile runs until ground impact. The third phase explicitly selects RK2; the second uses the simulation's Euler default.

```json
{
  "object": {"mass": 1.0},
  "motion_sequence": [
    {
      "phenomenon": "uniformly_accelerated_circular_motion",
      "integrator": "rk4",
      "circular_motion": {
        "center": {"x": 0.0, "y": 50.0},
        "radius": 5.0,
        "initial_angle_degrees": 0.0,
        "angular_velocity": 1.0,
        "angular_acceleration": 0.2
      },
      "until": {"type": "angle", "value": 180.0}
    },
    {
      "phenomenon": "uniformly_accelerated_motion",
      "acceleration": {"mode": "along_velocity", "value": 2.0},
      "until": {"type": "time", "value": 3.0}
    },
    {
      "phenomenon": "uniform_motion",
      "integrator": "rk2",
      "until": {"type": "distance", "value": 10.0}
    },
    {
      "phenomenon": "projectile_motion",
      "integrator": "rk4",
      "projectile": {"gravity": 9.8, "ground_height": 0.0},
      "until": {"type": "natural"}
    }
  ],
  "simulation": {
    "integrator": "euler",
    "playback_duration": 15.0
  }
}
```

### Phase-Local Moving Target and Observation Target

The first phase ends when the primary reaches the falling target. The second introduces a new static target and runs for one second, recording any HIT without ending early. No root target is used.

```json
{
  "object": {
    "mass": 1.0,
    "position": {"x": 0.0, "y": 5.0},
    "velocity": {"x": 10.0, "y": 0.0}
  },
  "motion_sequence": [
    {
      "phenomenon": "uniform_motion",
      "integrator": "rk4",
      "target": {
        "position": {"x": 10.0, "y": 10.0},
        "radius": 0.25,
        "motion": {"phenomenon": "free_fall", "gravity": 9.8}
      },
      "until": {"type": "target", "max_time": 3.0}
    },
    {
      "phenomenon": "uniform_motion",
      "target": {
        "position": {"x": 15.0, "y": 5.0},
        "radius": 0.3
      },
      "until": {"type": "time", "value": 1.0}
    }
  ],
  "simulation": {"integrator": "euler", "playback_duration": 6.0}
}
```

### Static-Target Projectile Solver

This requests the low-arc solution for a known launch speed. Do not also supply a direct launch velocity.

```json
{
  "phenomenon": "projectile_motion",
  "object": {
    "mass": 1.0,
    "position": {"x": 0.0, "y": 10.0}
  },
  "projectile": {"gravity": 9.8, "ground_height": 0.0},
  "target": {
    "position": {"x": 20.0, "y": 10.0},
    "radius": 0.25,
    "crossing": "descending"
  },
  "target_solution": {"mode": "known_speed", "speed": 20.0, "arc": "low"},
  "simulation": {
    "integrator": "rk4",
    "termination": {"type": "target", "max_time": 5.0},
    "playback_duration": 5.0
  }
}
```

## Building and Running

Dependencies:

- A C++ compiler with C++17 support (`std::optional` is used).
- raylib headers and libraries, including the `IsKeyPressedRepeat` API used by the editor.
- The header-only nlohmann/json library, available as `<nlohmann/json.hpp>`.

From the repository root, build the current source files:

```bash
cd physics-simulator

g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror \
    main.cpp CommandEditor.cpp CommandParser.cpp IntegratorFactory.cpp \
    Vector2D.cpp PhysicalObject.cpp Scale.cpp Kinematics.cpp \
    ProjectileSolver.cpp TargetDetector.cpp EulerIntegrator.cpp \
    RK2Integrator.cpp RK4Integrator.cpp Simulation.cpp Renderer.cpp \
    -o simulator -lraylib

./simulator
```

This command assumes the dependencies are installed in compiler/linker search paths. Additional platform-specific raylib link flags may be required. Running the application requires a graphical display.

## Project Structure

```text
Physics_Simulator/
├── README.md
└── physics-simulator/
    ├── main.cpp
    ├── CommandEditor.hpp / .cpp
    ├── CommandParser.hpp / .cpp
    ├── SimulationConfig.hpp     # MotionPhase and validated command data
    ├── TerminationCondition.hpp
    ├── Simulation.hpp / .cpp
    ├── Kinematics.hpp / .cpp    # Models, validation, metrics, and guides
    ├── PhysicalObject.hpp / .cpp
    ├── Vector2D.hpp / .cpp
    ├── Integrator.hpp          # IntegratorType, MotionState, evaluator
    ├── EulerIntegrator.hpp / .cpp
    ├── RK2Integrator.hpp / .cpp
    ├── RK4Integrator.hpp / .cpp
    ├── IntegratorFactory.hpp / .cpp
    ├── ProjectileSolver.hpp / .cpp
    ├── TargetDetector.hpp / .cpp
    ├── TrajectoryTarget.hpp
    ├── Scale.hpp / .cpp
    └── Renderer.hpp / .cpp
```

The paired-file notation denotes a header and source file with the same base name. The repository currently uses direct compiler commands; no CMake build definition is included.

## Validation

Development regression checks have covered linear motion, projectile and circular motion, moving targets, motion sequences, target collision, reset/seek, and parser compatibility. The latest verified baseline preserved **72 numerical Euler regression records** and **926 parser compatibility cases and messages** through the integrator changes.

Additional checks covered RK stage evaluation, Explicit Midpoint behavior, method convergence, per-phase defaults/overrides, state continuity, target lifetimes, preflight/live agreement, and independence from rendering frame budgets. The application has compiled cleanly with `-Wall -Wextra -Wpedantic -Werror`.

These checks were run with development harnesses outside the repository. A checked-in automated test suite and one-command test runner are not currently provided.

## Roadmap

- Expanded numerical-method comparisons, examples, and error visualization.
- A reproducible, repository-hosted regression test suite and build automation.
- Additional physical phenomena as the engine evolves.
- AI-assisted translation of natural-language physics problems into validated JSON commands, kept separate from the physics engine.
