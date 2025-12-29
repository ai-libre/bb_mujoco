defmodule BB.Mujoco.Exporter do
  @moduledoc """
  Exports BB robot definitions to MuJoCo MJCF XML format.

  BB's topology DSL maps naturally to MJCF:
  - BB links → MJCF bodies
  - BB joints → MJCF joints
  - BB visuals → MJCF geoms
  - BB actuators → MJCF actuators

  ## Example

      # Export robot to MJCF string
      mjcf = BB.Mujoco.Exporter.to_mjcf(MyRobot)

      # Export with custom physics parameters
      mjcf = BB.Mujoco.Exporter.to_mjcf(MyRobot,
        timestep: 0.002,
        gravity: [0, 0, -9.81]
      )

      # Write to file
      BB.Mujoco.Exporter.to_mjcf_file(MyRobot, "robot.xml")
  """

  @default_opts [
    timestep: 0.002,
    gravity: [0, 0, -9.81],
    integrator: "implicitfast",
    cone: "pyramidal"
  ]

  @doc "Convert BB robot to MJCF XML string"
  @spec to_mjcf(module(), keyword()) :: String.t()
  def to_mjcf(robot_module, opts \\ []) do
    opts = Keyword.merge(@default_opts, opts)
    robot = get_robot!(robot_module)
    model_name = format_model_name(robot.name)

    """
    <?xml version="1.0" encoding="utf-8"?>
    <mujoco model="#{model_name}">
      #{compiler_section()}
      #{option_section(opts)}
      #{default_section()}
      #{asset_section(robot)}
      #{worldbody_section(robot)}
      #{actuator_section(robot)}
    </mujoco>
    """
  end

  @doc "Export robot to MJCF file"
  @spec to_mjcf_file(module(), Path.t(), keyword()) :: :ok | {:error, term()}
  def to_mjcf_file(robot_module, path, opts \\ []) do
    mjcf = to_mjcf(robot_module, opts)
    File.write(path, mjcf)
  end

  # Private helpers

  defp compiler_section do
    """
      <compiler angle="radian" meshdir="assets" autolimits="true"/>
    """
  end

  defp option_section(opts) do
    [gx, gy, gz] = opts[:gravity]
    timestep = opts[:timestep]
    integrator = opts[:integrator]
    cone = opts[:cone]

    """
      <option timestep="#{timestep}" integrator="#{integrator}" cone="#{cone}" gravity="#{gx} #{gy} #{gz}"/>
    """
  end

  defp default_section do
    """
      <default>
        <joint damping="0.5" armature="0.01"/>
        <geom contype="1" conaffinity="1" condim="3" friction="1 0.5 0.001"/>
        <position kp="100" kv="10"/>
      </default>
    """
  end

  defp asset_section(robot) do
    meshes = collect_meshes(robot)

    mesh_elements =
      meshes
      |> Enum.map(fn {filename, scale} ->
        name = Path.basename(filename, Path.extname(filename))
        scale_attr = if scale && scale != 1, do: ~s( scale="#{scale} #{scale} #{scale}"), else: ""
        ~s(    <mesh name="#{name}" file="#{Path.basename(filename)}"#{scale_attr}/>)
      end)
      |> Enum.join("\n")

    """
      <asset>
    #{mesh_elements}
      </asset>
    """
  end

  defp worldbody_section(robot) do
    # Start from root link using BB's flat structure
    root_link_name = robot.root_link
    body_xml = link_to_body(robot, root_link_name, nil, 0)

    """
      <worldbody>
        <light name="top" pos="0 0 2" dir="0 0 -1"/>
        <geom name="floor" type="plane" size="2 2 0.1" rgba="0.8 0.9 0.8 1" pos="0 0 0"/>
    #{body_xml}
      </worldbody>
    """
  end

  defp actuator_section(robot) do
    actuators = collect_actuators(robot)

    actuator_elements =
      actuators
      |> Enum.map(fn {name, joint_name} ->
        # Use position actuators for servo-like control (matches Dynamixel behavior)
        # ctrl = target position, MuJoCo's PD controller handles the rest
        ~s(    <position name="#{name}" joint="#{joint_name}"/>)
      end)
      |> Enum.join("\n")

    """
      <actuator>
    #{actuator_elements}
      </actuator>
    """
  end

  # Build MJCF body from BB link using flat structure
  # In MuJoCo, a joint must be inside the body it moves (the child body),
  # not in the parent body. The joint defines how the body moves relative to its parent.
  defp link_to_body(robot, link_name, parent_joint, depth) do
    indent = String.duplicate("  ", depth + 2)
    link = Map.fetch!(robot.links, link_name)

    # Get position from parent joint (if any)
    pos = if parent_joint, do: format_joint_pos(parent_joint.origin), else: "0 0 0"
    quat = if parent_joint, do: format_joint_quat(parent_joint.origin), else: "1 0 0 0"

    body_attrs = ~s(name="#{link_name}" pos="#{pos}" quat="#{quat}")

    # Output the joint INSIDE this body (if we have a parent joint connecting us)
    # This is the correct MuJoCo structure: joint inside the body it moves
    joint_xml = if parent_joint, do: joint_to_xml(parent_joint, depth), else: ""

    # Build geom from visual (BB has single visual, not list)
    geom_xml = if link.visual, do: visual_to_geom(link_name, link.visual, 0, depth + 1), else: ""

    # Build child bodies - joints are output inside each child body, not here
    children_xml =
      link.child_joints
      |> Enum.map(fn joint_name ->
        joint = Map.fetch!(robot.joints, joint_name)
        link_to_body(robot, joint.child_link, joint, depth + 1)
      end)
      |> Enum.join("\n")

    """
    #{indent}<body #{body_attrs}>
    #{joint_xml}
    #{geom_xml}
    #{children_xml}
    #{indent}</body>
    """
  end

  defp visual_to_geom(link_name, visual, index, depth) do
    indent = String.duplicate("  ", depth + 3)
    geom_name = "#{link_name}_visual_#{index}"

    # BB visual has geometry as tuple {type, params}
    geom_attrs = geometry_to_attrs(visual.geometry)
    rgba = format_rgba(visual.material)

    # Apply visual origin transform if present
    origin_attrs = format_visual_origin(visual.origin)

    ~s(#{indent}<geom name="#{geom_name}" #{geom_attrs}#{origin_attrs} rgba="#{rgba}"/>)
  end

  # Visual origin in BB is a tuple {{x, y, z}, {roll, pitch, yaw}}
  defp format_visual_origin(nil), do: ""

  defp format_visual_origin({{x, y, z}, {r, p, yaw}}) do
    pos = if x != 0 or y != 0 or z != 0, do: ~s( pos="#{x} #{y} #{z}"), else: ""

    quat =
      if r != 0 or p != 0 or yaw != 0 do
        {w, qx, qy, qz} = rpy_to_quaternion(r, p, yaw)
        ~s( quat="#{w} #{qx} #{qy} #{qz}")
      else
        ""
      end

    "#{pos}#{quat}"
  end

  defp format_visual_origin(_), do: ""

  defp joint_to_xml(joint, depth) do
    indent = String.duplicate("  ", depth + 3)

    type = joint_type(joint.type)

    if type do
      axis = format_axis(joint.axis)
      range = format_range(joint.limits)

      range_attr = if range, do: ~s( range="#{range}"), else: ""

      ~s(#{indent}<joint name="#{joint.name}" type="#{type}" axis="#{axis}"#{range_attr}/>)
    else
      ""
    end
  end

  # BB geometry is a tuple {type, params}
  defp geometry_to_attrs({:box, %{x: x, y: y, z: z}}) do
    ~s(type="box" size="#{x / 2} #{y / 2} #{z / 2}")
  end

  # Support both :height and :length for cylinders (BB uses height, but be defensive)
  defp geometry_to_attrs({:cylinder, %{radius: r, height: h}}) do
    ~s(type="cylinder" size="#{r} #{h / 2}")
  end

  defp geometry_to_attrs({:cylinder, %{radius: r, length: l}}) do
    ~s(type="cylinder" size="#{r} #{l / 2}")
  end

  defp geometry_to_attrs({:sphere, %{radius: r}}) do
    ~s(type="sphere" size="#{r}")
  end

  defp geometry_to_attrs({:mesh, %{filename: f}}) do
    mesh_name = Path.basename(f, Path.extname(f))
    ~s(type="mesh" mesh="#{mesh_name}")
  end

  defp geometry_to_attrs(_), do: ~s(type="sphere" size="0.01")

  defp joint_type(:revolute), do: "hinge"
  defp joint_type(:prismatic), do: "slide"
  defp joint_type(:continuous), do: "hinge"
  defp joint_type(:fixed), do: nil
  defp joint_type(:floating), do: "free"
  defp joint_type(:planar), do: "slide"
  defp joint_type(_), do: nil

  # BB joint origin is %{position: {x, y, z}, orientation: {r, p, y}}
  defp format_joint_pos(nil), do: "0 0 0"
  defp format_joint_pos(%{position: {x, y, z}}), do: "#{x} #{y} #{z}"
  defp format_joint_pos(%{position: nil}), do: "0 0 0"

  # Convert RPY orientation to quaternion
  defp format_joint_quat(nil), do: "1 0 0 0"
  defp format_joint_quat(%{orientation: nil}), do: "1 0 0 0"
  defp format_joint_quat(%{orientation: {r, p, y}}) when r == 0 and p == 0 and y == 0 do
    "1 0 0 0"
  end

  defp format_joint_quat(%{orientation: {roll, pitch, yaw}}) do
    # Convert RPY to quaternion (ZYX convention)
    {w, x, y, z} = rpy_to_quaternion(roll, pitch, yaw)
    "#{w} #{x} #{y} #{z}"
  end

  defp rpy_to_quaternion(roll, pitch, yaw) do
    cr = :math.cos(roll / 2)
    sr = :math.sin(roll / 2)
    cp = :math.cos(pitch / 2)
    sp = :math.sin(pitch / 2)
    cy = :math.cos(yaw / 2)
    sy = :math.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    {w, x, y, z}
  end

  # BB axis is a tuple {x, y, z}
  defp format_axis(nil), do: "0 0 1"
  defp format_axis({x, y, z}), do: "#{x} #{y} #{z}"

  # BB limits is %{lower: _, upper: _, effort: _, velocity: _}
  defp format_range(nil), do: nil
  defp format_range(%{lower: lower, upper: upper}), do: "#{lower} #{upper}"

  # BB material has color: %{red: _, green: _, blue: _, alpha: _}
  defp format_rgba(nil), do: "0.7 0.7 0.7 1"

  defp format_rgba(%{color: %{red: r, green: g, blue: b, alpha: a}}) do
    "#{r} #{g} #{b} #{a}"
  end

  defp format_rgba(%{color: %{red: r, green: g, blue: b}}) do
    "#{r} #{g} #{b} 1"
  end

  defp format_rgba(_), do: "0.7 0.7 0.7 1"

  defp collect_meshes(robot) do
    # Walk links and collect mesh paths with scales from visuals
    robot.links
    |> Map.values()
    |> Enum.flat_map(fn link ->
      case link.visual do
        %{geometry: {:mesh, %{filename: f, scale: s}}} -> [{f, s}]
        %{geometry: {:mesh, %{filename: f}}} -> [{f, 1}]
        _ -> []
      end
    end)
    |> Enum.uniq_by(fn {f, _} -> f end)
  end

  defp collect_actuators(robot) do
    # Collect actuated joints (all non-fixed joints for now)
    # Position actuators provide servo-like behavior matching Dynamixel hardware
    robot.joints
    |> Map.values()
    |> Enum.filter(fn joint -> joint.type != :fixed end)
    |> Enum.map(fn joint -> {"#{joint.name}_pos", joint.name} end)
  end

  defp get_robot!(robot_module) do
    # Ensure module is compiled before checking exports
    case Code.ensure_compiled(robot_module) do
      {:module, _} ->
        if function_exported?(robot_module, :robot, 0) do
          robot_module.robot()
        else
          raise ArgumentError,
                "#{inspect(robot_module)} is not a BB robot module (missing robot/0 function)"
        end

      {:error, reason} ->
        raise ArgumentError, "Could not load module #{inspect(robot_module)}: #{inspect(reason)}"
    end
  end

  # Format model name: extract readable name from module atom
  defp format_model_name(name) when is_atom(name) do
    name
    |> Atom.to_string()
    |> String.replace_prefix("Elixir.", "")
    |> String.split(".")
    |> List.last()
    |> Macro.underscore()
  end

  defp format_model_name(name), do: to_string(name)
end
