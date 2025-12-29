defmodule BB.Mujoco do
  @moduledoc """
  MuJoCo WASM physics simulation and 3D visualization for Beam Bots.

  BB.Mujoco provides browser-based physics simulation using MuJoCo WASM,
  allowing you to visualize and simulate your BB robots with full physics
  (gravity, collisions, motor dynamics) without any native dependencies.

  ## Architecture

  The browser acts as a "physics server" - it runs MuJoCo WASM and responds
  to commands from Elixir via Phoenix Channels. This provides:

  - Low-latency physics (runs at 60fps in browser)
  - Rich 3D visualization with Three.js
  - No native compilation required
  - Works on any platform with a modern browser

  ## Quick Start

      # In your Phoenix router
      scope "/mujoco" do
        pipe_through :browser
        live "/viewer", BbMujocoWeb.ViewerLive
      end

      # Start your robot with physics simulation
      MyRobot.start_link(simulation: :physics)

  ## Components

  - `BB.Mujoco.Bridge` - GenServer coordinating Elixir ↔ browser physics
  - `BB.Mujoco.Channel` - Phoenix Channel for bidirectional communication
  - `BB.Mujoco.Exporter` - Converts BB robot definitions to MJCF XML
  - `BB.Mujoco.SimulationController` - 50Hz control loop

  ## Message Protocol

  Commands sent to browser:

  - `set_joints(joints)` - Set motor control targets
  - `step(dt)` - Advance physics by dt seconds
  - `get_joints()` - Get current joint positions
  - `reset()` - Reset simulation to initial state

  Responses from browser include joint positions, velocities, and simulation time.
  """

  @doc """
  Start the MuJoCo simulation for a robot.

  ## Options

  - `:robot` - The robot module (required)
  - `:hz` - Control loop frequency, default 50
  - `:dt` - Physics timestep, default 0.02

  ## Example

      BB.Mujoco.start_link(robot: MyRobot)
  """
  defdelegate start_link(opts), to: BB.Mujoco.Supervisor

  @doc """
  Check if the browser physics backend is connected.
  """
  defdelegate connected?(robot), to: BB.Mujoco.Bridge

  @doc """
  Export a BB robot definition to MJCF XML format.

  ## Example

      mjcf = BB.Mujoco.to_mjcf(MyRobot)
      File.write!("robot.xml", mjcf)
  """
  defdelegate to_mjcf(robot, opts \\ []), to: BB.Mujoco.Exporter
end
