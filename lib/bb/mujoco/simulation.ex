defmodule BB.Mujoco.Simulation do
  @moduledoc """
  50Hz control loop for browser-based MuJoCo simulation.

  This GenServer owns robot state and drives physics through the Bridge/Channel
  to browser WASM. It follows the beachy SimulationController pattern.

  ## State Ownership

  This GenServer is the single source of truth for:
  - Joint targets (what we're commanding)
  - Joint positions (what physics computed)
  - Joint velocities
  - Simulation time

  ## Lifecycle

  Each robot gets its own Simulation GenServer. When switching robots,
  the old Simulation is stopped and a new one started - ensuring clean
  WASM state reset.

  ## Data Flow

      Slider → ViewerLive → Simulation.set_joint()
                                   ↓
                           (50Hz tick)
                                   ↓
                      Bridge.step_with_targets()
                                   ↓
                      Channel → Browser WASM
                                   ↓
                      Response via Channel
                                   ↓
                      Simulation broadcasts via PubSub
                                   ↓
                      ViewerLive.handle_info() → UI update

  ## Example

      # Start simulation for a robot
      {:ok, pid} = BB.Mujoco.Simulation.start_link(robot: MyRobot)

      # Set joint targets
      BB.Mujoco.Simulation.set_joint(MyRobot, 0, 1.5)

      # Get current state
      {:ok, state} = BB.Mujoco.Simulation.get_state(MyRobot)
  """

  use GenServer
  require Logger

  alias BB.Mujoco.Bridge

  # Control loop at 50Hz
  @control_hz 50
  @tick_interval_ms div(1000, @control_hz)
  @step_dt 1.0 / @control_hz

  defstruct [
    :robot_module,
    :robot_name,
    :joint_count,
    :joint_targets,
    :joint_positions,
    :joint_velocities,
    :sim_time,
    :step_count,
    :running,
    :backend_ready,
    :tick_timer
  ]

  # ===========================================================================
  # Client API
  # ===========================================================================

  @doc "Start simulation for a robot module"
  def start_link(opts) do
    robot = Keyword.fetch!(opts, :robot)
    GenServer.start_link(__MODULE__, opts, name: via(robot))
  end

  @doc "Stop simulation (used when switching robots)"
  def stop(robot) do
    GenServer.stop(via(robot), :normal)
  catch
    :exit, _ -> :ok
  end

  @doc "Set target position for a single joint"
  def set_joint(robot, index, value) when is_integer(index) and is_number(value) do
    GenServer.call(via(robot), {:set_joint, index, value})
  catch
    :exit, _ -> {:error, :not_running}
  end

  @doc "Set all joint targets at once"
  def set_joints(robot, targets) when is_list(targets) do
    GenServer.call(via(robot), {:set_joints, targets})
  catch
    :exit, _ -> {:error, :not_running}
  end

  @doc "Get current joint positions (from physics)"
  def get_joints(robot) do
    GenServer.call(via(robot), :get_joints)
  catch
    :exit, _ -> {:error, :not_running}
  end

  @doc "Get current joint targets"
  def get_targets(robot) do
    GenServer.call(via(robot), :get_targets)
  catch
    :exit, _ -> {:error, :not_running}
  end

  @doc "Get full simulation state"
  def get_state(robot) do
    GenServer.call(via(robot), :get_state)
  catch
    :exit, _ -> {:error, :not_running}
  end

  @doc "Pause/resume simulation"
  def set_paused(robot, paused) when is_boolean(paused) do
    GenServer.call(via(robot), {:set_paused, paused})
  catch
    :exit, _ -> {:error, :not_running}
  end

  @doc "Reset simulation to initial state"
  def reset(robot) do
    GenServer.call(via(robot), :reset)
  catch
    :exit, _ -> {:error, :not_running}
  end

  @doc "Check if simulation is running"
  def running?(robot) do
    GenServer.call(via(robot), :running?)
  catch
    :exit, _ -> false
  end

  @doc "Notify that backend is ready (called when browser connects)"
  def backend_ready(robot) do
    GenServer.cast(via(robot), :backend_ready)
  catch
    :exit, _ -> :ok
  end

  @doc "Update joint count (called when WASM reports model info)"
  def set_joint_count(robot, count) when is_integer(count) and count > 0 do
    GenServer.cast(via(robot), {:set_joint_count, count})
  catch
    :exit, _ -> :ok
  end

  # ===========================================================================
  # Server Implementation
  # ===========================================================================

  @impl GenServer
  def init(opts) do
    robot = Keyword.fetch!(opts, :robot)
    robot_name = robot_to_name(robot)

    # Default joint count, will be updated when WASM reports model info
    joint_count = Keyword.get(opts, :joint_count, 6)

    Logger.info("[Simulation] Starting for #{robot_name} with #{joint_count} joints")

    # Start Bridge for this robot (needed for Channel communication)
    case Bridge.start_link(robot: robot) do
      {:ok, pid} ->
        Logger.info("[Simulation] Started Bridge for #{robot_name}, pid: #{inspect(pid)}")

      {:error, {:already_started, pid}} ->
        Logger.info("[Simulation] Bridge already running for #{robot_name}, pid: #{inspect(pid)}")

      {:error, reason} ->
        Logger.warning("[Simulation] Failed to start Bridge: #{inspect(reason)}")
    end

    state = %__MODULE__{
      robot_module: robot,
      robot_name: robot_name,
      joint_count: joint_count,
      joint_targets: List.duplicate(0.0, joint_count),
      joint_positions: List.duplicate(0.0, joint_count),
      joint_velocities: List.duplicate(0.0, joint_count),
      sim_time: 0.0,
      step_count: 0,
      running: true,
      backend_ready: false,
      tick_timer: nil
    }

    # Subscribe to bridge connection events
    Phoenix.PubSub.subscribe(BbMujoco.PubSub, "mujoco:#{robot}")

    {:ok, state, {:continue, :check_backend}}
  end

  @impl GenServer
  def handle_continue(:check_backend, state) do
    # Check if bridge is already connected
    if Bridge.connected?(state.robot_module) do
      Logger.info("[Simulation] Backend already connected, starting control loop")
      timer = schedule_tick()
      {:noreply, %{state | backend_ready: true, tick_timer: timer}}
    else
      Logger.info("[Simulation] Waiting for browser connection...")
      {:noreply, state}
    end
  end

  @impl GenServer
  def handle_call({:set_joint, index, value}, _from, state) do
    if index >= 0 and index < state.joint_count do
      targets = List.replace_at(state.joint_targets, index, value * 1.0)
      # INFO level so we can see it without restarting
      Logger.info("[Simulation] SET_JOINT(#{index}, #{value}) => targets=#{inspect(targets)}")
      {:reply, :ok, %{state | joint_targets: targets}}
    else
      Logger.warning("[Simulation] set_joint(#{index}, #{value}) INVALID - joint_count=#{state.joint_count}")
      {:reply, {:error, :invalid_index}, state}
    end
  end

  def handle_call({:set_joints, targets}, _from, state) do
    normalized = normalize_targets(targets, state.joint_count)
    {:reply, :ok, %{state | joint_targets: normalized}}
  end

  def handle_call(:get_joints, _from, state) do
    {:reply, {:ok, state.joint_positions}, state}
  end

  def handle_call(:get_targets, _from, state) do
    {:reply, {:ok, state.joint_targets}, state}
  end

  def handle_call(:get_state, _from, state) do
    result = %{
      joints: state.joint_positions,
      velocities: state.joint_velocities,
      targets: state.joint_targets,
      time: state.sim_time,
      step_count: state.step_count,
      running: state.running,
      backend_ready: state.backend_ready
    }

    {:reply, {:ok, result}, state}
  end

  def handle_call({:set_paused, paused}, _from, state) do
    new_running = not paused

    # Start/stop tick timer based on pause state
    new_timer =
      cond do
        new_running and state.backend_ready and is_nil(state.tick_timer) ->
          schedule_tick()

        not new_running and not is_nil(state.tick_timer) ->
          Process.cancel_timer(state.tick_timer)
          nil

        true ->
          state.tick_timer
      end

    {:reply, :ok, %{state | running: new_running, tick_timer: new_timer}}
  end

  def handle_call(:reset, _from, state) do
    case Bridge.reset(state.robot_module) do
      {:ok, result} ->
        joints = Map.get(result, :joints, List.duplicate(0.0, state.joint_count))
        velocities = Map.get(result, :velocities, List.duplicate(0.0, state.joint_count))

        new_state = %{
          state
          | joint_positions: joints,
            joint_velocities: velocities,
            joint_targets: List.duplicate(0.0, state.joint_count),
            sim_time: 0.0,
            step_count: 0
        }

        broadcast_tick(new_state)
        {:reply, :ok, new_state}

      {:error, reason} ->
        {:reply, {:error, reason}, state}
    end
  end

  def handle_call(:running?, _from, state) do
    {:reply, state.running and state.backend_ready, state}
  end

  @impl GenServer
  def handle_cast(:backend_ready, state) do
    Logger.info("[Simulation] Backend connected, starting control loop")
    timer = if state.running, do: schedule_tick(), else: nil
    {:noreply, %{state | backend_ready: true, tick_timer: timer}}
  end

  def handle_cast({:set_joint_count, count}, state) do
    if count != state.joint_count do
      Logger.info("[Simulation] Updating joint count: #{state.joint_count} -> #{count}")

      {:noreply,
       %{
         state
         | joint_count: count,
           joint_targets: normalize_targets(state.joint_targets, count),
           joint_positions: normalize_targets(state.joint_positions, count),
           joint_velocities: normalize_targets(state.joint_velocities, count)
       }}
    else
      {:noreply, state}
    end
  end

  @impl GenServer
  def handle_info(:tick, state) do
    new_state =
      if state.running and state.backend_ready do
        run_control_tick(state)
      else
        state
      end

    # Schedule next tick if still running
    timer =
      if new_state.running and new_state.backend_ready do
        schedule_tick()
      else
        nil
      end

    {:noreply, %{new_state | tick_timer: timer}}
  end

  # Handle bridge connection events via PubSub
  def handle_info({:mujoco_connected, _pid}, state) do
    Logger.info("[Simulation] Browser connected via PubSub")

    timer =
      if state.running and is_nil(state.tick_timer) do
        schedule_tick()
      else
        state.tick_timer
      end

    {:noreply, %{state | backend_ready: true, tick_timer: timer}}
  end

  def handle_info(:mujoco_disconnected, state) do
    Logger.warning("[Simulation] Browser disconnected")
    if state.tick_timer, do: Process.cancel_timer(state.tick_timer)
    {:noreply, %{state | backend_ready: false, tick_timer: nil}}
  end

  def handle_info(_msg, state) do
    {:noreply, state}
  end

  @impl GenServer
  def terminate(reason, state) do
    Logger.info("[Simulation] Terminating for #{state.robot_name}: #{inspect(reason)}")
    if state.tick_timer, do: Process.cancel_timer(state.tick_timer)
    :ok
  end

  # ===========================================================================
  # Private Functions
  # ===========================================================================

  defp run_control_tick(state) do
    # Log every 50th step (1 second) at INFO level for visibility
    if rem(state.step_count, 50) == 0 do
      Logger.info("[Simulation] TICK #{state.step_count}: targets=#{inspect(state.joint_targets)}")
    end

    # Send targets to WASM and step physics
    case Bridge.step_with_targets(state.robot_module, state.joint_targets, @step_dt) do
      {:ok, result} ->
        joints = Map.get(result, :joints, state.joint_positions)
        velocities = Map.get(result, :velocities, state.joint_velocities)
        time = Map.get(result, :time, state.sim_time + @step_dt)

        # Log position changes at INFO level
        if rem(state.step_count, 50) == 0 do
          Logger.info("[Simulation] TICK #{state.step_count}: positions=#{inspect(joints)}")
        end

        new_state = %{
          state
          | joint_positions: joints,
            joint_velocities: velocities,
            sim_time: time,
            step_count: state.step_count + 1
        }

        # Broadcast state via PubSub
        broadcast_tick(new_state)

        new_state

      {:error, reason} ->
        Logger.warning("[Simulation] Step failed: #{inspect(reason)}")
        state
    end
  end

  defp broadcast_tick(state) do
    Phoenix.PubSub.broadcast(
      BbMujoco.PubSub,
      "bb:mujoco:simulation",
      {:simulation_tick,
       %{
         robot: state.robot_name,
         joints: state.joint_positions,
         velocities: state.joint_velocities,
         targets: state.joint_targets,
         time: state.sim_time,
         step_count: state.step_count
       }}
    )
  end

  defp schedule_tick do
    Process.send_after(self(), :tick, @tick_interval_ms)
  end

  defp via(robot) do
    {:via, Registry, {BB.Mujoco.Registry, {robot, :simulation}}}
  end

  defp robot_to_name(robot) when is_atom(robot) do
    robot
    |> Atom.to_string()
    |> String.replace_prefix("Elixir.", "")
  end

  defp robot_to_name(robot) when is_binary(robot), do: robot

  defp normalize_targets(targets, count) when length(targets) == count do
    Enum.map(targets, &(&1 * 1.0))
  end

  defp normalize_targets(targets, count) when length(targets) < count do
    targets = Enum.map(targets, &(&1 * 1.0))
    targets ++ List.duplicate(0.0, count - length(targets))
  end

  defp normalize_targets(targets, count) when length(targets) > count do
    targets
    |> Enum.take(count)
    |> Enum.map(&(&1 * 1.0))
  end
end
