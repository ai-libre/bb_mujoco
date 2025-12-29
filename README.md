# BB MuJoCo

MuJoCo WASM physics simulation and 3D visualization for [Beam Bots](https://github.com/beam-bots/bb).

## Quick Start

```bash
mix setup
mix phx.server
```

Open http://localhost:4000 to see the landing page with example robots.

### Example Robots

- **Pan-Tilt Camera** - `/viewer/BbMujoco.Examples.PanTiltRobot`
- **Linear Actuator** - `/viewer/BbMujoco.Examples.LinearActuator`
- **6-DOF Arm** - `/viewer/BbMujoco.Examples.SixDofArm`
- **Differential Drive** - `/viewer/BbMujoco.Examples.DifferentialDrive`
- **Mesh Robot** - `/viewer/BbMujoco.Examples.MeshRobot`

## Overview

BB MuJoCo provides browser-based physics simulation using MuJoCo WASM, allowing you to visualize and simulate your BB robots with full physics (gravity, collisions, motor dynamics) without any native dependencies.

### Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Browser (MuJoCo WASM)                        │
├─────────────────────────────────────────────────────────────────────┤
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐  │
│  │  MuJoCo WASM     │  │   Three.js       │  │  Phoenix Channel │  │
│  │  (Physics)       │──│   (Rendering)    │──│  (Communication) │  │
│  └──────────────────┘  └──────────────────┘  └──────────────────┘  │
└───────────────────────────────────────────┬─────────────────────────┘
                                            │ WebSocket
                                            │
┌───────────────────────────────────────────┴─────────────────────────┐
│                     Elixir (BB + BB.Mujoco)                         │
├─────────────────────────────────────────────────────────────────────┤
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐  │
│  │  BB.Robot        │  │  BB.Mujoco       │  │  BB.Mujoco       │  │
│  │  (Robot Runtime) │──│  .Bridge         │──│  .Channel        │  │
│  └──────────────────┘  └──────────────────┘  └──────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

The browser acts as a "physics server" - it runs MuJoCo WASM and responds to commands from Elixir via Phoenix Channels.

## Installation

Add `bb_mujoco` to your dependencies:

```elixir
def deps do
  [
    {:bb, "~> 0.10"},
    {:bb_mujoco, "~> 0.1"}
  ]
end
```

## Setup

### 1. Download MuJoCo WASM

Download the MuJoCo WASM files and place them in `priv/static/vendor/`:

```bash
# Download from MuJoCo releases
curl -L https://github.com/google-deepmind/mujoco/releases/download/3.2.0/mujoco-3.2.0-wasm.zip -o mujoco-wasm.zip
unzip mujoco-wasm.zip -d priv/static/vendor/
```

### 2. Configure your Phoenix app

Add the socket to your endpoint:

```elixir
# lib/my_app_web/endpoint.ex
socket "/mujoco", BB.Mujoco.Socket,
  websocket: true,
  longpoll: false
```

Add routes:

```elixir
# lib/my_app_web/router.ex
scope "/mujoco" do
  pipe_through :browser
  live "/viewer/:robot", BbMujocoWeb.ViewerLive, :show
end

scope "/api/mujoco" do
  pipe_through :api
  get "/mjcf/:robot", BbMujocoWeb.MjcfController, :show
end
```

### 3. Start your robot with physics

```elixir
# Start with physics simulation
MyRobot.start_link(simulation: :physics)
```

### 4. Open the viewer

Navigate to `http://localhost:4000/mujoco/viewer/MyRobot` in your browser.

## Usage

### Export robot to MJCF

```elixir
# Export to string
mjcf = BB.Mujoco.to_mjcf(MyRobot)

# Export to file
BB.Mujoco.Exporter.to_mjcf_file(MyRobot, "robot.xml")
```

### Check connection status

```elixir
BB.Mujoco.connected?(MyRobot)
#=> true
```

### Direct physics control

```elixir
# Set joint targets
BB.Mujoco.Bridge.set_joints(MyRobot, [0.0, 0.5, -0.3])

# Step physics
{:ok, state} = BB.Mujoco.Bridge.step(MyRobot, 0.02)
#=> %{joints: [...], velocities: [...], time: 1.234}

# Reset simulation
BB.Mujoco.Bridge.reset(MyRobot)
```

## Components

- **BB.Mujoco.Bridge** - GenServer coordinating Elixir ↔ browser physics
- **BB.Mujoco.Channel** - Phoenix Channel for bidirectional communication
- **BB.Mujoco.Exporter** - Converts BB robot definitions to MJCF XML
- **BB.Mujoco.Socket** - Phoenix Socket configuration

## JavaScript Hook

The `MujocoViewer` hook handles:
- Loading MuJoCo WASM
- Building Three.js scene from MuJoCo geoms
- Real-time visualization updates
- Physics command execution

## Development

```bash
# Install dependencies
mix setup

# Start dev server
mix phx.server

# Run tests
mix test
```

## License

Apache-2.0
