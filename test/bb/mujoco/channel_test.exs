defmodule BB.Mujoco.ChannelTest do
  use BbMujocoWeb.ChannelCase

  alias BB.Mujoco.Channel

  describe "join/3" do
    test "joins successfully with valid robot name" do
      # Start a bridge for the robot
      {:ok, _bridge} = start_supervised({BB.Mujoco.Bridge, robot: BbMujoco.TestRobot})

      {:ok, _, socket} =
        socket(BB.Mujoco.Socket, nil, %{})
        |> subscribe_and_join(Channel, "mujoco:physics:BbMujoco.TestRobot")

      assert socket.assigns.robot == BbMujoco.TestRobot
    end

    test "fails with unknown robot" do
      {:error, %{reason: reason}} =
        socket(BB.Mujoco.Socket, nil, %{})
        |> subscribe_and_join(Channel, "mujoco:physics:NonExistentRobot")

      assert reason =~ "unknown robot"
    end
  end

  describe "handle_in physics:ready" do
    setup do
      {:ok, _bridge} = start_supervised({BB.Mujoco.Bridge, robot: BbMujoco.TestRobot})

      {:ok, _, socket} =
        socket(BB.Mujoco.Socket, nil, %{})
        |> subscribe_and_join(Channel, "mujoco:physics:BbMujoco.TestRobot")

      {:ok, socket: socket}
    end

    test "broadcasts ready event", %{socket: socket} do
      ref = push(socket, "physics:ready", %{"nq" => 2, "nv" => 2, "nu" => 2})
      assert_reply ref, :ok
    end
  end

  describe "handle_in physics:response" do
    setup do
      {:ok, _bridge} = start_supervised({BB.Mujoco.Bridge, robot: BbMujoco.TestRobot})

      {:ok, _, socket} =
        socket(BB.Mujoco.Socket, nil, %{})
        |> subscribe_and_join(Channel, "mujoco:physics:BbMujoco.TestRobot")

      {:ok, socket: socket}
    end

    test "forwards response to bridge", %{socket: socket} do
      # This would normally be called in response to a command
      ref = push(socket, "physics:response", %{"id" => 1, "status" => "ok", "joints" => [0.0, 0.0]})
      assert_reply ref, :ok
    end
  end
end
