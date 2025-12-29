defmodule BB.Mujoco.BridgeTest do
  use ExUnit.Case, async: true

  alias BB.Mujoco.Bridge

  describe "start_link/1" do
    test "starts with robot option" do
      {:ok, pid} = Bridge.start_link(robot: TestRobot)
      assert Process.alive?(pid)
      GenServer.stop(pid)
    end

    test "raises without robot option" do
      assert_raise KeyError, fn ->
        Bridge.start_link([])
      end
    end
  end

  describe "connected?/1" do
    setup do
      {:ok, pid} = start_supervised({Bridge, robot: TestRobotConnected})
      {:ok, pid: pid}
    end

    test "returns false when no browser connected" do
      refute Bridge.connected?(TestRobotConnected)
    end
  end

  describe "commands when disconnected" do
    setup do
      {:ok, _pid} = start_supervised({Bridge, robot: TestRobotDisconnected})
      :ok
    end

    test "set_joints returns error" do
      assert {:error, :disconnected} = Bridge.set_joints(TestRobotDisconnected, [0.0, 0.0])
    end

    test "get_joints returns error" do
      assert {:error, :disconnected} = Bridge.get_joints(TestRobotDisconnected)
    end

    test "step returns error" do
      assert {:error, :disconnected} = Bridge.step(TestRobotDisconnected, 0.02)
    end

    test "reset returns error" do
      assert {:error, :disconnected} = Bridge.reset(TestRobotDisconnected)
    end
  end

  describe "register_channel/2" do
    setup do
      {:ok, _pid} = start_supervised({Bridge, robot: TestRobotChannel})
      :ok
    end

    test "registers channel and sets connected state" do
      refute Bridge.connected?(TestRobotChannel)

      # Simulate channel registration
      :ok = Bridge.register_channel(TestRobotChannel, self())

      assert Bridge.connected?(TestRobotChannel)
    end
  end
end
