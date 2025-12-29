defmodule BB.Mujoco.Supervisor do
  @moduledoc """
  Supervisor for BB.Mujoco components.

  Starts:
  - Registry for process registration
  - Bridge GenServer for physics coordination
  - SimulationController for control loop (when connected)
  """

  use Supervisor

  def start_link(opts) do
    robot = Keyword.fetch!(opts, :robot)
    Supervisor.start_link(__MODULE__, opts, name: via(robot))
  end

  @impl Supervisor
  def init(opts) do
    robot = Keyword.fetch!(opts, :robot)

    children = [
      {BB.Mujoco.Bridge, robot: robot}
    ]

    Supervisor.init(children, strategy: :one_for_one)
  end

  defp via(robot) do
    {:via, Registry, {BB.Mujoco.Registry, {robot, :supervisor}}}
  end
end
