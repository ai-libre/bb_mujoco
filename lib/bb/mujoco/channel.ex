defmodule BB.Mujoco.Channel do
  @moduledoc """
  Phoenix Channel handler for bidirectional physics communication.

  ## Message Flow

  ```
  Browser                           Elixir Backend
     │                                   │
     ├── physics:ready ───────────────→ │
     │   (MuJoCo loaded)                │
     │                                   │
     │ ← ─ physics:command ─────────────┤
     │   (set_joints, step, etc.)        │
     │                                   │
     ├── physics:response ───────────→  │
     │   (response with joints/vels)     │
     │                                   │
     └── physics:error ────────────────→ │
         (error notification)             │
  ```
  """

  use Phoenix.Channel
  require Logger

  @impl Phoenix.Channel
  def join("mujoco:physics:" <> robot_name, _params, socket) do
    robot = String.to_existing_atom("Elixir.#{robot_name}")

    Logger.info("[BB.Mujoco.Channel] Browser joining for #{robot_name}")

    # Register with bridge after successful join
    send(self(), :register_with_bridge)

    {:ok, assign(socket, :robot, robot)}
  rescue
    ArgumentError ->
      {:error, %{reason: "unknown robot: #{robot_name}"}}
  end

  @impl Phoenix.Channel
  def handle_info(:register_with_bridge, socket) do
    # Try to register with bridge if it's running (optional for viewer-only mode)
    try do
      BB.Mujoco.Bridge.register_channel(socket.assigns.robot, self())
    catch
      :exit, _ ->
        Logger.debug("[BB.Mujoco.Channel] Bridge not running, viewer-only mode")
    end
    {:noreply, socket}
  end

  def handle_info({:push, event, payload}, socket) do
    push(socket, event, payload)
    {:noreply, socket}
  end

  @impl Phoenix.Channel
  def handle_in("physics:ready", params, socket) do
    Logger.info("[BB.Mujoco.Channel] MuJoCo ready: #{inspect(params)}")

    # Retry registration with Bridge (it may have started since initial join)
    try do
      BB.Mujoco.Bridge.register_channel(socket.assigns.robot, self())
      Logger.info("[BB.Mujoco.Channel] Registered with Bridge")
    catch
      :exit, _ ->
        Logger.debug("[BB.Mujoco.Channel] Bridge still not available")
    end

    # Broadcast readiness
    Phoenix.PubSub.broadcast(
      BbMujoco.PubSub,
      "mujoco:#{socket.assigns.robot}",
      {:mujoco_ready, params}
    )

    {:reply, :ok, socket}
  end

  def handle_in("physics:response", response, socket) do
    # Forward to bridge if running (optional for viewer-only mode)
    try do
      BB.Mujoco.Bridge.handle_response(socket.assigns.robot, response)
    catch
      :exit, _ -> :ok
    end
    {:reply, :ok, socket}
  end

  def handle_in("physics:error", %{"message" => msg}, socket) do
    Logger.error("[BB.Mujoco.Channel] Browser error: #{msg}")
    {:noreply, socket}
  end

  @impl Phoenix.Channel
  def terminate(_reason, socket) do
    Logger.info("[BB.Mujoco.Channel] Browser disconnected for #{socket.assigns.robot}")
    :ok
  end
end
