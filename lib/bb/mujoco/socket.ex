defmodule BB.Mujoco.Socket do
  @moduledoc """
  Phoenix Socket for MuJoCo physics communication.

  Configure in your endpoint:

      socket "/mujoco", BB.Mujoco.Socket,
        websocket: true,
        longpoll: false
  """

  use Phoenix.Socket

  channel "mujoco:physics:*", BB.Mujoco.Channel

  @impl Phoenix.Socket
  def connect(_params, socket, _connect_info) do
    {:ok, socket}
  end

  @impl Phoenix.Socket
  def id(_socket), do: nil
end
