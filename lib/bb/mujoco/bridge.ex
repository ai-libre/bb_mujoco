defmodule BB.Mujoco.Bridge do
  @moduledoc """
  GenServer that bridges BB robot state with browser-based MuJoCo physics.

  The browser acts as a physics server - it runs MuJoCo WASM and responds
  to commands from Elixir. This inverts the typical client/server relationship
  but provides excellent performance (physics runs at 60fps in browser).

  ## States

  - `:disconnected` - No browser connected
  - `:connecting` - Browser is loading MuJoCo WASM
  - `:connected` - Ready for physics commands
  - `:degraded` - Browser slow/unresponsive, using cached state

  ## Example

      # Start bridge for a robot
      {:ok, _pid} = BB.Mujoco.Bridge.start_link(robot: MyRobot)

      # Check connection
      BB.Mujoco.Bridge.connected?(MyRobot)
      #=> true

      # Step physics (called by simulation controller)
      {:ok, state} = BB.Mujoco.Bridge.step(MyRobot, 0.02)
      #=> %{joints: [...], velocities: [...], time: 1.234}
  """

  use GenServer
  require Logger

  @type state :: :disconnected | :connecting | :connected | :degraded
  @command_timeout 100

  # Client API

  @doc "Start the bridge for a robot"
  @spec start_link(keyword()) :: GenServer.on_start()
  def start_link(opts) do
    robot = Keyword.fetch!(opts, :robot)
    GenServer.start_link(__MODULE__, opts, name: via(robot))
  end

  @doc "Register a channel connection from the browser"
  @spec register_channel(module(), pid()) :: :ok
  def register_channel(robot, channel_pid) do
    GenServer.call(via(robot), {:register_channel, channel_pid})
  end

  @doc "Set joint targets (ctrl array in MuJoCo)"
  @spec set_joints(module(), [float()]) :: :ok | {:error, term()}
  def set_joints(robot, joints) when is_list(joints) do
    GenServer.call(via(robot), {:set_joints, joints}, @command_timeout * 2)
  end

  @doc "Step physics simulation by dt seconds"
  @spec step(module(), float()) :: {:ok, map()} | {:error, term()}
  def step(robot, dt) when is_float(dt) do
    GenServer.call(via(robot), {:step, dt}, @command_timeout * 2)
  end

  @doc "Step physics with targets in one call (more efficient)"
  @spec step_with_targets(module(), [float()], float()) :: {:ok, map()} | {:error, term()}
  def step_with_targets(robot, targets, dt) do
    GenServer.call(via(robot), {:step_with_targets, targets, dt}, @command_timeout * 2)
  end

  @doc "Get current joint positions"
  @spec get_joints(module()) :: {:ok, [float()]} | {:error, term()}
  def get_joints(robot) do
    GenServer.call(via(robot), :get_joints, @command_timeout * 2)
  end

  @doc "Get current velocities"
  @spec get_velocities(module()) :: {:ok, [float()]} | {:error, term()}
  def get_velocities(robot) do
    GenServer.call(via(robot), :get_velocities, @command_timeout * 2)
  end

  @doc "Reset simulation to initial state"
  @spec reset(module()) :: :ok | {:error, term()}
  def reset(robot) do
    GenServer.call(via(robot), :reset, @command_timeout * 2)
  end

  @doc "Check if browser physics is connected"
  @spec connected?(module()) :: boolean()
  def connected?(robot) do
    GenServer.call(via(robot), :connected?)
  catch
    :exit, _ -> false
  end

  @doc "Get current state (joints + velocities)"
  @spec get_state(module()) :: {:ok, map()} | {:error, term()}
  def get_state(robot) do
    GenServer.call(via(robot), :get_state, @command_timeout * 2)
  end

  @doc "Handle response from browser channel"
  @spec handle_response(module(), map()) :: :ok
  def handle_response(robot, response) do
    GenServer.cast(via(robot), {:response, response})
  end

  # Server Implementation

  @impl GenServer
  def init(opts) do
    robot = Keyword.fetch!(opts, :robot)

    state = %{
      robot: robot,
      status: :disconnected,
      channel_pid: nil,
      pending_calls: %{},
      call_counter: 0,
      last_state: nil
    }

    Logger.info("[BB.Mujoco.Bridge] Started for #{inspect(robot)}")
    {:ok, state}
  end

  @impl GenServer
  def handle_call({:register_channel, channel_pid}, _from, state) do
    Process.monitor(channel_pid)
    Logger.info("[BB.Mujoco.Bridge] Browser connected")

    # Broadcast connection event
    Phoenix.PubSub.broadcast(
      BbMujoco.PubSub,
      "mujoco:#{state.robot}",
      {:mujoco_connected, channel_pid}
    )

    # Notify Simulation GenServer if running
    try do
      BB.Mujoco.Simulation.backend_ready(state.robot)
    catch
      :exit, _ -> :ok  # Simulation might not be started yet
    end

    {:reply, :ok, %{state | status: :connected, channel_pid: channel_pid}}
  end

  def handle_call({:set_joints, joints}, from, state) do
    send_command(state, "set_joints", %{joints: joints}, from)
  end

  def handle_call({:step, dt}, from, state) do
    send_command(state, "step", %{dt: dt}, from)
  end

  def handle_call({:step_with_targets, targets, dt}, from, state) do
    send_command(state, "step_with_targets", %{targets: targets, dt: dt}, from)
  end

  def handle_call(:get_joints, from, state) do
    send_command(state, "get_joints", %{}, from)
  end

  def handle_call(:get_velocities, from, state) do
    send_command(state, "get_velocities", %{}, from)
  end

  def handle_call(:reset, from, state) do
    send_command(state, "reset", %{}, from)
  end

  def handle_call(:connected?, _from, state) do
    {:reply, state.status == :connected, state}
  end

  def handle_call(:get_state, from, state) do
    if state.last_state do
      {:reply, {:ok, state.last_state}, state}
    else
      send_command(state, "get_state", %{}, from)
    end
  end

  @impl GenServer
  def handle_cast({:response, %{"id" => id} = response}, state) do
    case Map.pop(state.pending_calls, id) do
      {{from, cmd}, pending_calls} ->
        result = parse_response(cmd, response)

        # Cache state for degraded mode
        new_last_state =
          case result do
            {:ok, %{joints: _, velocities: _} = s} -> s
            _ -> state.last_state
          end

        GenServer.reply(from, result)
        {:noreply, %{state | pending_calls: pending_calls, last_state: new_last_state}}

      {nil, _} ->
        Logger.warning("[BB.Mujoco.Bridge] Unknown response id: #{id}")
        {:noreply, state}
    end
  end

  @impl GenServer
  def handle_info({:DOWN, _ref, :process, pid, _reason}, %{channel_pid: pid} = state) do
    Logger.warning("[BB.Mujoco.Bridge] Browser disconnected")

    # Reply to all pending calls with error
    for {_id, {from, _cmd}} <- state.pending_calls do
      GenServer.reply(from, {:error, :disconnected})
    end

    # Broadcast disconnection
    Phoenix.PubSub.broadcast(
      BbMujoco.PubSub,
      "mujoco:#{state.robot}",
      :mujoco_disconnected
    )

    {:noreply, %{state | status: :disconnected, channel_pid: nil, pending_calls: %{}}}
  end

  def handle_info({:timeout, id}, state) do
    case Map.pop(state.pending_calls, id) do
      {{from, _cmd}, pending_calls} ->
        GenServer.reply(from, {:error, :timeout})
        {:noreply, %{state | pending_calls: pending_calls}}

      {nil, _} ->
        {:noreply, state}
    end
  end

  # Private helpers

  defp send_command(%{status: :disconnected} = state, _cmd, _params, _from) do
    {:reply, {:error, :disconnected}, state}
  end

  defp send_command(state, cmd, params, from) do
    id = state.call_counter + 1

    message = %{
      cmd: cmd,
      id: id,
      params: params
    }

    # Send to channel
    send(state.channel_pid, {:push, "physics:command", message})

    # Set timeout
    Process.send_after(self(), {:timeout, id}, @command_timeout)

    # Track pending call
    pending_calls = Map.put(state.pending_calls, id, {from, cmd})

    {:noreply, %{state | pending_calls: pending_calls, call_counter: id}}
  end

  defp parse_response(_cmd, %{"status" => "ok"} = response) do
    result =
      response
      |> Map.drop(["id", "status"])
      |> Enum.map(fn {k, v} -> {String.to_atom(k), v} end)
      |> Map.new()

    {:ok, result}
  end

  defp parse_response(_cmd, %{"status" => "error", "message" => msg}) do
    {:error, msg}
  end

  defp parse_response(_cmd, response) do
    {:error, {:unexpected_response, response}}
  end

  defp via(robot) do
    {:via, Registry, {BB.Mujoco.Registry, {robot, :bridge}}}
  end
end
