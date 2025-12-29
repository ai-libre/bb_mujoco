defmodule BbMujocoWeb.ViewerLive do
  @moduledoc """
  LiveView for MuJoCo robot visualization.

  Provides a full-screen 3D viewer with:
  - MuJoCo WASM physics simulation (server-driven via Simulation GenServer)
  - Three.js rendering
  - Joint control sliders
  - Physics stepping controls
  - Camera presets

  ## Architecture

  This LiveView does NOT send commands directly to the JS hook.
  Instead, it communicates through the Simulation GenServer:

      Slider → ViewerLive → Simulation.set_joint()
                                   ↓
                           (50Hz tick)
                                   ↓
                      Bridge → Channel → Browser WASM
                                   ↓
                      PubSub broadcast
                                   ↓
                      ViewerLive.handle_info() → UI update
  """

  use BbMujocoWeb, :live_view
  require Logger

  alias BB.Mujoco.Simulation

  # Available example robots
  @example_robots [
    {"BbMujoco.Examples.MeshRobot", "Mesh Robot"},
    {"BbMujoco.Examples.SixDofArm", "6-DOF Arm"},
    {"BbMujoco.Examples.DifferentialDrive", "Differential Drive"},
    {"BbMujoco.Examples.LinearActuator", "Linear Actuator"},
    {"BbMujoco.Examples.PanTiltRobot", "Pan-Tilt Camera"}
  ]

  # Camera presets
  @camera_presets [
    {"front", "Front", {0, 0.5, 1.5}},
    {"side", "Side", {1.5, 0.5, 0}},
    {"top", "Top", {0, 2, 0.01}},
    {"iso", "Isometric", {1, 0.8, 1}}
  ]

  @impl Phoenix.LiveView
  def mount(%{"robot" => robot_name}, _session, socket) do
    robot = get_robot_module(robot_name)

    socket =
      socket
      |> assign(:robot, robot)
      |> assign(:robot_name, robot_name)
      |> assign(:example_robots, @example_robots)
      |> assign(:camera_presets, @camera_presets)
      |> assign(:status, :initializing)
      |> assign(:model_info, nil)
      |> assign(:error, nil)
      |> assign(:paused, false)
      |> assign(:panel_open, true)
      |> assign(:joints, [])
      |> assign(:joint_controls, %{})
      |> assign(:joint_positions, [])
      |> assign(:sim_time, 0.0)
      |> assign(:step_count, 0)

    if connected?(socket) do
      # Subscribe to simulation tick broadcasts
      Phoenix.PubSub.subscribe(BbMujoco.PubSub, "bb:mujoco:simulation")
      # Subscribe to bridge connection events
      Phoenix.PubSub.subscribe(BbMujoco.PubSub, "mujoco:#{robot}")

      # Start Simulation GenServer for this robot
      start_simulation(robot)
    end

    {:ok, socket}
  end

  @impl Phoenix.LiveView
  def handle_params(%{"robot" => _robot_name}, _uri, socket) do
    # Just acknowledge params change - actual robot switch is handled in change_robot event
    {:noreply, socket}
  end

  @impl Phoenix.LiveView
  def terminate(_reason, socket) do
    # Stop simulation when LiveView terminates
    if socket.assigns[:robot] do
      Logger.info("[ViewerLive] Terminating, stopping simulation for #{inspect(socket.assigns.robot)}")
      Simulation.stop(socket.assigns.robot)
    end

    :ok
  end

  @impl Phoenix.LiveView
  def render(assigns) do
    ~H"""
    <Layouts.full flash={@flash}>
      <div class="flex h-full">
        <%!-- Side Panel (collapsible) --%>
        <aside class={[
          "shrink-0 flex flex-col bg-slate-800 border-r border-slate-700 transition-all duration-200 overflow-hidden",
          if(@panel_open, do: "w-72", else: "w-0")
        ]}>
          <div class="flex flex-col h-full overflow-y-auto">
            <%!-- Robot Selector --%>
            <div class="p-3 border-b border-slate-700">
              <.form for={%{}} phx-change="change_robot">
                <label class="block text-xs text-slate-400 mb-1">Robot</label>
                <select
                  name="robot"
                  class="w-full px-2 py-1.5 text-sm bg-slate-700 text-white rounded border border-slate-600 focus:border-cyan-500 focus:outline-none"
                >
                  <%= for {module, name} <- @example_robots do %>
                    <option value={module} selected={module == @robot_name}><%= name %></option>
                  <% end %>
                </select>
              </.form>
            </div>

            <%!-- Physics Controls --%>
            <div class="p-3 border-b border-slate-700">
              <h3 class="text-xs font-semibold text-slate-400 uppercase tracking-wide mb-2">Physics</h3>
              <div class="flex gap-2">
                <button
                  phx-click="toggle_pause"
                  class={[
                    "flex-1 px-3 py-1.5 text-xs rounded transition",
                    if(@paused, do: "bg-green-600 hover:bg-green-500", else: "bg-yellow-600 hover:bg-yellow-500"),
                    "text-white"
                  ]}
                >
                  <%= if @paused, do: "▶ Play", else: "⏸ Pause" %>
                </button>
                <button
                  phx-click="step_physics"
                  disabled={not @paused}
                  class="px-3 py-1.5 text-xs bg-slate-600 hover:bg-slate-500 text-white rounded transition disabled:opacity-50"
                >
                  Step
                </button>
                <button
                  phx-click="reset"
                  class="px-3 py-1.5 text-xs bg-red-600 hover:bg-red-500 text-white rounded transition"
                >
                  Reset
                </button>
              </div>
              <div class="mt-2 text-xs text-slate-500">
                Time: <%= Float.round(@sim_time, 2) %>s | Steps: <%= @step_count %>
              </div>
            </div>

            <%!-- Joint Controls --%>
            <div class="p-3 border-b border-slate-700 flex-1 overflow-y-auto">
              <h3 class="text-xs font-semibold text-slate-400 uppercase tracking-wide mb-2">
                Joint Controls
                <%= if length(@joints) > 0 do %>
                  <span class="text-slate-500 font-normal">(<%= length(@joints) %>)</span>
                <% end %>
              </h3>
              <%= if length(@joints) > 0 do %>
                <form phx-change="joint_change" class="space-y-3">
                  <%= for {joint, idx} <- Enum.with_index(@joints) do %>
                    <div class="space-y-1">
                      <div class="flex justify-between text-xs">
                        <span class="text-slate-300"><%= joint.name %></span>
                        <span class="text-slate-500 font-mono">
                          <%= format_angle(Map.get(@joint_controls, idx, joint.value)) %>
                        </span>
                      </div>
                      <input
                        type="range"
                        name={"joint[#{idx}]"}
                        min={joint.min}
                        max={joint.max}
                        step="0.01"
                        value={Map.get(@joint_controls, idx, joint.value)}
                        class="w-full h-1.5 bg-slate-600 rounded-lg appearance-none cursor-pointer accent-cyan-500"
                      />
                    </div>
                  <% end %>
                </form>
              <% else %>
                <p class="text-xs text-slate-500 italic">
                  <%= if @status == :connected, do: "No controllable joints", else: "Loading..." %>
                </p>
              <% end %>
            </div>

            <%!-- Camera Controls --%>
            <div class="p-3">
              <h3 class="text-xs font-semibold text-slate-400 uppercase tracking-wide mb-2">Camera</h3>
              <div class="grid grid-cols-2 gap-1.5">
                <%= for {id, name, _pos} <- @camera_presets do %>
                  <button
                    phx-click="set_camera"
                    phx-value-preset={id}
                    class="px-2 py-1.5 text-xs bg-slate-700 hover:bg-slate-600 text-slate-300 rounded transition"
                  >
                    <%= name %>
                  </button>
                <% end %>
              </div>
            </div>
          </div>
        </aside>

        <%!-- Main Content --%>
        <div class="flex-1 flex flex-col min-w-0">
          <%!-- Header --%>
          <header class="shrink-0 flex items-center justify-between px-4 py-2 bg-slate-800/95 backdrop-blur border-b border-slate-700/50">
            <div class="flex items-center gap-3">
              <button
                phx-click="toggle_panel"
                class="p-1 text-slate-400 hover:text-white transition"
                title={if @panel_open, do: "Hide panel", else: "Show panel"}
              >
                <%= if @panel_open do %>
                  <svg class="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M11 19l-7-7 7-7m8 14l-7-7 7-7" />
                  </svg>
                <% else %>
                  <svg class="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M13 5l7 7-7 7M5 5l7 7-7 7" />
                  </svg>
                <% end %>
              </button>
              <h1 class="text-lg font-semibold text-white">BB MuJoCo</h1>
              <span class="px-2 py-0.5 text-xs font-mono bg-slate-700/80 text-slate-300 rounded">
                <%= get_robot_short_name(@robot_name) %>
              </span>
              <div class="flex items-center gap-1.5">
                <div class={["w-2 h-2 rounded-full", status_color(@status)]} />
                <span class="text-xs text-slate-400"><%= status_text(@status) %></span>
              </div>
            </div>

            <div class="flex items-center gap-4">
              <%= if @model_info do %>
                <div class="flex items-center gap-3 text-xs text-slate-500">
                  <span>nq: <%= @model_info["nq"] || @model_info[:nq] %></span>
                  <span>nv: <%= @model_info["nv"] || @model_info[:nv] %></span>
                  <span>nu: <%= @model_info["nu"] || @model_info[:nu] %></span>
                </div>
              <% end %>
            </div>
          </header>

          <%!-- Viewport --%>
          <main class="flex-1 relative min-h-0 bg-slate-900">
            <div
              id={"mujoco-viewer-#{@robot_name}"}
              phx-hook="MujocoViewer"
              phx-update="ignore"
              data-robot={@robot_name}
              data-paused={to_string(@paused)}
              class="absolute inset-0"
            />

            <%!-- Error overlay --%>
            <%= if @error do %>
              <div class="absolute inset-0 flex items-center justify-center bg-slate-900/90 z-10">
                <div class="bg-red-900/50 border border-red-500 rounded-lg p-6 max-w-md">
                  <h3 class="text-red-400 font-semibold mb-2">Error</h3>
                  <p class="text-red-300 text-sm"><%= @error %></p>
                </div>
              </div>
            <% end %>

            <%!-- Loading overlay --%>
            <%= if @status == :initializing do %>
              <div class="absolute inset-0 flex items-center justify-center bg-slate-900/90 z-10">
                <div class="flex flex-col items-center gap-3">
                  <div class="w-8 h-8 border-2 border-blue-500 border-t-transparent rounded-full animate-spin" />
                  <p class="text-slate-400 text-sm">Loading MuJoCo WASM...</p>
                </div>
              </div>
            <% end %>

            <%!-- Controls hint --%>
            <div class="absolute bottom-3 right-3 text-xs text-slate-500/70 z-10 pointer-events-none">
              Drag to rotate • Scroll to zoom • Right-drag to pan
            </div>
          </main>
        </div>
      </div>
    </Layouts.full>
    """
  end

  # ============================================================================
  # Event Handlers
  # ============================================================================

  @impl Phoenix.LiveView
  def handle_event("mujoco_status", %{"status" => "ready"} = params, socket) do
    model_info = %{
      nbody: params["nbody"],
      njnt: params["njnt"],
      nq: params["nq"],
      nv: params["nv"],
      nu: params["nu"]
    }

    # Build joint info from model
    joints = build_joint_info(params)
    nu = params["nu"] || 0

    # Notify Simulation of joint count
    if socket.assigns.robot && nu > 0 do
      Simulation.set_joint_count(socket.assigns.robot, nu)
      # Also notify backend is ready (WASM loaded)
      Simulation.backend_ready(socket.assigns.robot)
    end

    {:noreply,
     socket
     |> assign(:status, :connected)
     |> assign(:model_info, model_info)
     |> assign(:joints, joints)
     |> assign(:joint_controls, %{})}
  end

  def handle_event("mujoco_status", %{"status" => "error", "message" => msg}, socket) do
    {:noreply,
     socket
     |> assign(:status, :error)
     |> assign(:error, msg)}
  end

  def handle_event("mujoco_status", %{"status" => status}, socket) do
    {:noreply, assign(socket, :status, String.to_atom(status))}
  end

  def handle_event("change_robot", %{"robot" => robot_name}, socket) do
    old_robot = socket.assigns.robot

    # Stop old simulation
    if old_robot do
      Logger.info("[ViewerLive] Stopping simulation for robot change: #{inspect(old_robot)}")
      Simulation.stop(old_robot)
    end

    # Get new robot module
    new_robot = get_robot_module(robot_name)

    # Unsubscribe from old robot's PubSub topic
    if old_robot do
      Phoenix.PubSub.unsubscribe(BbMujoco.PubSub, "mujoco:#{old_robot}")
    end

    # Subscribe to new robot's PubSub topic
    Phoenix.PubSub.subscribe(BbMujoco.PubSub, "mujoco:#{new_robot}")

    # Start new simulation
    start_simulation(new_robot)

    # Push event to JS hook to reload model (instead of full page navigation)
    # This avoids race conditions with hook cleanup
    # Also push_patch to update the URL for bookmarking/refreshing
    {:noreply,
     socket
     |> assign(:robot, new_robot)
     |> assign(:robot_name, robot_name)
     |> assign(:status, :initializing)
     |> assign(:model_info, nil)
     |> assign(:joints, [])
     |> assign(:joint_controls, %{})
     |> assign(:sim_time, 0.0)
     |> assign(:step_count, 0)
     |> push_event("mujoco_load_robot", %{robot: robot_name})
     |> push_patch(to: ~p"/viewer/#{robot_name}", replace: true)}
  end

  def handle_event("toggle_panel", _params, socket) do
    {:noreply, assign(socket, :panel_open, not socket.assigns.panel_open)}
  end

  def handle_event("toggle_pause", _params, socket) do
    new_paused = not socket.assigns.paused

    # Use Simulation GenServer to control pause state
    if socket.assigns.robot do
      Simulation.set_paused(socket.assigns.robot, new_paused)
    end

    {:noreply, assign(socket, :paused, new_paused)}
  end

  def handle_event("step_physics", _params, socket) do
    # Single step not implemented yet - would need to temporarily unpause
    {:noreply, socket}
  end

  def handle_event("reset", _params, socket) do
    # Use Simulation GenServer to reset
    if socket.assigns.robot do
      Simulation.reset(socket.assigns.robot)
    end

    {:noreply,
     socket
     |> assign(:sim_time, 0.0)
     |> assign(:step_count, 0)
     |> assign(:joint_controls, %{})}
  end

  def handle_event("joint_change", %{"joint" => joint_params}, socket) do
    # Parse joint values and update targets via Simulation GenServer
    joint_controls =
      joint_params
      |> Enum.reject(fn {key, _} -> String.starts_with?(key, "_") end)
      |> Enum.map(fn {idx_str, value_str} ->
        {String.to_integer(idx_str), parse_float_safe(value_str)}
      end)
      |> Enum.into(%{})

    # Update each joint target in the Simulation GenServer
    robot = socket.assigns.robot
    Logger.info("[ViewerLive] JOINT_CHANGE: robot=#{inspect(robot)}, controls=#{inspect(joint_controls)}")

    if robot do
      for {idx, value} <- joint_controls do
        result = Simulation.set_joint(robot, idx, value)
        Logger.info("[ViewerLive] SET_JOINT(#{idx}, #{value}) => #{inspect(result)}")
      end
    else
      Logger.warning("[ViewerLive] joint_change but robot is nil!")
    end

    {:noreply, assign(socket, :joint_controls, joint_controls)}
  end

  def handle_event("set_camera", %{"preset" => preset_id}, socket) do
    # Camera control stays as push_event since it's pure visualization
    # not physics state - this is the ONLY push_event we keep
    case Enum.find(@camera_presets, fn {id, _, _} -> id == preset_id end) do
      {_, _, {x, y, z}} ->
        {:noreply, push_event(socket, "mujoco_camera", %{position: [x, y, z]})}

      nil ->
        {:noreply, socket}
    end
  end

  # ============================================================================
  # PubSub Handlers
  # ============================================================================

  @impl Phoenix.LiveView
  def handle_info({:simulation_tick, %{robot: robot_name} = tick_data}, socket) do
    # Only process ticks for our robot
    if robot_name == socket.assigns.robot_name do
      {:noreply,
       socket
       |> assign(:joint_positions, tick_data.joints)
       |> assign(:sim_time, tick_data.time)
       |> assign(:step_count, tick_data.step_count)}
    else
      {:noreply, socket}
    end
  end

  def handle_info({:mujoco_connected, _pid}, socket) do
    {:noreply, assign(socket, :status, :connected)}
  end

  def handle_info(:mujoco_disconnected, socket) do
    {:noreply, assign(socket, :status, :disconnected)}
  end

  def handle_info({:mujoco_ready, params}, socket) do
    # Only update joints if the new params have joint_ranges
    joints =
      case params["joint_ranges"] || params[:joint_ranges] do
        ranges when is_list(ranges) and length(ranges) > 0 ->
          build_joint_info(params)

        _ ->
          socket.assigns.joints
      end

    {:noreply,
     socket
     |> assign(:status, :connected)
     |> assign(:model_info, params)
     |> assign(:joints, joints)}
  end

  def handle_info(_msg, socket) do
    {:noreply, socket}
  end

  # ============================================================================
  # Helpers
  # ============================================================================

  defp start_simulation(robot) when is_atom(robot) and not is_nil(robot) do
    case Simulation.start_link(robot: robot) do
      {:ok, pid} ->
        Logger.info("[ViewerLive] Started simulation for #{inspect(robot)}, pid: #{inspect(pid)}")

      {:error, {:already_started, pid}} ->
        Logger.info("[ViewerLive] Simulation already running for #{inspect(robot)}, pid: #{inspect(pid)}")

      {:error, reason} ->
        Logger.error("[ViewerLive] Failed to start simulation: #{inspect(reason)}")
    end
  end

  defp start_simulation(_robot), do: :ok

  defp get_robot_module(robot_name) do
    String.to_existing_atom("Elixir.#{robot_name}")
  rescue
    ArgumentError -> nil
  end

  defp get_robot_short_name(full_name) do
    full_name
    |> String.split(".")
    |> List.last()
  end

  defp build_joint_info(params) do
    nu = params["nu"] || params[:nu] || 0
    joint_names = params["joint_names"] || []
    joint_ranges = params["joint_ranges"] || []

    Logger.debug("[ViewerLive] build_joint_info: nu=#{nu}, ranges=#{inspect(joint_ranges)}")

    if nu == 0 do
      []
    else
      Enum.map(0..(nu - 1), fn idx ->
        name = Enum.at(joint_names, idx, "joint_#{idx}")
        raw_range = Enum.at(joint_ranges, idx)

        # Joint ranges come from JS as [min, max] arrays
        {min, max} =
          case raw_range do
            [lo, hi] -> {lo, hi}
            {lo, hi} -> {lo, hi}
            _ -> {-3.14, 3.14}
          end

        %{
          name: name,
          value: 0.0,
          min: min,
          max: max
        }
      end)
    end
  end

  defp parse_float_safe(str) when is_binary(str) do
    case Float.parse(str) do
      {float, _} -> float
      :error -> 0.0
    end
  end

  defp parse_float_safe(_), do: 0.0

  defp format_angle(rad) when is_number(rad) do
    deg = rad * 180.0 / :math.pi()
    "#{Float.round(deg, 1)}°"
  end

  defp format_angle(_), do: "0°"

  defp status_color(:initializing), do: "bg-yellow-500 animate-pulse"
  defp status_color(:connected), do: "bg-green-500"
  defp status_color(:disconnected), do: "bg-red-500"
  defp status_color(:error), do: "bg-red-500"
  defp status_color(_), do: "bg-slate-500"

  defp status_text(:initializing), do: "Initializing..."
  defp status_text(:connected), do: "Connected"
  defp status_text(:disconnected), do: "Disconnected"
  defp status_text(:error), do: "Error"
  defp status_text(_), do: "Unknown"
end
