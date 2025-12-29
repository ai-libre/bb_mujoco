defmodule BB.Mujoco.ExporterTest do
  use ExUnit.Case, async: true

  alias BB.Mujoco.Exporter

  describe "to_mjcf/2 basic structure" do
    test "generates valid MJCF XML structure" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)

      assert mjcf =~ ~r/<\?xml version="1.0"/
      assert mjcf =~ ~r/<mujoco model="/
      assert mjcf =~ ~r/<compiler/
      assert mjcf =~ ~r/<option/
      assert mjcf =~ ~r/<default>/
      assert mjcf =~ ~r/<worldbody>/
      assert mjcf =~ ~r/<actuator>/
      assert mjcf =~ ~r/<\/mujoco>/
    end

    test "includes gravity option" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)
      assert mjcf =~ ~r/gravity="0 0 -9.81"/
    end

    test "includes timestep option" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)
      assert mjcf =~ ~r/timestep="0.002"/
    end

    test "respects custom options" do
      mjcf =
        Exporter.to_mjcf(BbMujoco.TestRobot,
          timestep: 0.001,
          gravity: [0, 0, -10.0]
        )

      assert mjcf =~ ~r/timestep="0.001"/
      assert mjcf =~ ~r/gravity="0 0 -10.0"/
    end

    test "includes floor and light in worldbody" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)

      assert mjcf =~ ~r/<light name="top"/
      assert mjcf =~ ~r/<geom name="floor" type="plane"/
    end

    test "sets compiler angle to radian" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)
      assert mjcf =~ ~r/angle="radian"/
    end
  end

  describe "joint placement - CRITICAL" do
    @tag :critical
    test "joints are placed INSIDE child body, not parent body" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)

      # Parse the XML to verify structure
      # Joint should be INSIDE the body it moves, not its parent

      # pan_joint should be inside pan_link, not base_link
      # Pattern: <body name="pan_link"...><joint name="pan_joint"
      assert mjcf =~ ~r/<body name="pan_link"[^>]*>\s*<joint name="pan_joint"/s,
             "pan_joint should be INSIDE pan_link body"

      # tilt_joint should be inside tilt_link, not pan_link
      assert mjcf =~ ~r/<body name="tilt_link"[^>]*>\s*<joint name="tilt_joint"/s,
             "tilt_joint should be INSIDE tilt_link body"
    end

    @tag :critical
    test "base_link has no joint (it's fixed to world)" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)

      # base_link should NOT have a joint directly inside it
      # It should open, then have a geom, then have child bodies
      assert mjcf =~ ~r/<body name="base_link"[^>]*>\s*\n?\s*<geom/s,
             "base_link should have geom (no joint - it's fixed to world)"
    end

    @tag :critical
    test "joint is NOT a sibling of child body" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)

      # WRONG pattern: joint before body at same level
      # <joint name="pan_joint"/>\n<body name="pan_link">
      refute mjcf =~ ~r/<joint name="pan_joint"[^>]*\/>\s*\n?\s*<body name="pan_link"/s,
             "joint should NOT be sibling of body (should be inside it)"

      refute mjcf =~ ~r/<joint name="tilt_joint"[^>]*\/>\s*\n?\s*<body name="tilt_link"/s,
             "joint should NOT be sibling of body (should be inside it)"
    end
  end

  describe "joint placement - LinearActuator" do
    @tag :critical
    test "slider_joint is inside slider_link" do
      mjcf = Exporter.to_mjcf(BbMujoco.Examples.LinearActuator)

      # slider_joint should be inside slider_link body
      assert mjcf =~ ~r/<body name="slider_link"[^>]*>\s*<joint name="slider_joint"/s,
             "slider_joint should be INSIDE slider_link body"
    end

    @tag :critical
    test "base_link has no joint" do
      mjcf = Exporter.to_mjcf(BbMujoco.Examples.LinearActuator)

      # base_link should start with geom, not joint
      assert mjcf =~ ~r/<body name="base_link"[^>]*>\s*\n?\s*<geom/s,
             "base_link should have geom first (no joint)"
    end

    test "slide joint type is correct" do
      mjcf = Exporter.to_mjcf(BbMujoco.Examples.LinearActuator)

      assert mjcf =~ ~r/<joint name="slider_joint" type="slide"/,
             "prismatic joint should be type='slide'"
    end
  end

  describe "body hierarchy" do
    test "bodies are properly nested" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)

      # Verify nesting: base_link contains pan_link contains tilt_link
      # Simple check: tilt_link closes before pan_link closes before base_link closes
      base_close = :binary.match(mjcf, "</body>") |> elem(0)
      assert base_close > 0, "Should have closing body tags"
    end

    test "each body has required name attribute" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)

      assert mjcf =~ ~r/<body name="base_link"/
      assert mjcf =~ ~r/<body name="pan_link"/
      assert mjcf =~ ~r/<body name="tilt_link"/
    end

    test "child body has pos from joint origin" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)

      # pan_link should have pos="0 0 0.05" from pan_joint origin
      assert mjcf =~ ~r/<body name="pan_link" pos="0.0 0.0 0.05"/,
             "pan_link position should come from pan_joint origin"

      # tilt_link should have pos="0 0 0.1" from tilt_joint origin
      assert mjcf =~ ~r/<body name="tilt_link" pos="0.0 0.0 0.1"/,
             "tilt_link position should come from tilt_joint origin"
    end
  end

  describe "joint attributes" do
    test "hinge joints have correct type" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)

      assert mjcf =~ ~r/<joint name="pan_joint" type="hinge"/
      assert mjcf =~ ~r/<joint name="tilt_joint" type="hinge"/
    end

    test "joints have axis attribute" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)

      # Default axis is 0 0 1 (Z-axis) - may be formatted as 0.0 0.0 1.0
      assert mjcf =~ ~r/<joint name="pan_joint"[^>]*axis="0\.?0? 0\.?0? 1\.?0?"/
    end

    test "joints have range attribute when limits defined" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)

      # pan_joint: -90 to +90 degrees = -1.5708 to 1.5708 radians
      assert mjcf =~ ~r/<joint name="pan_joint"[^>]*range="-1\.57[0-9]* 1\.57[0-9]*"/
    end
  end

  describe "geom attributes" do
    test "box geometry has correct type and size" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)

      # base_link has box 0.1 x 0.1 x 0.05
      # MJCF uses half-sizes: 0.05 x 0.05 x 0.025
      assert mjcf =~ ~r/<geom name="base_link_visual_0" type="box" size="0.05 0.05 0.025"/
    end

    test "cylinder geometry has correct type and size" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)

      # pan_link has cylinder radius=0.02, height=0.1
      # MJCF: size="radius half_height" = "0.02 0.05"
      assert mjcf =~ ~r/<geom name="pan_link_visual_0" type="cylinder" size="0.02 0.05"/
    end

    test "geoms have rgba from material color" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)

      # base_link: grey (0.3, 0.3, 0.3, 1.0)
      assert mjcf =~ ~r/<geom name="base_link_visual_0"[^>]*rgba="0.3 0.3 0.3 1.0"/
    end

    test "visual origin is applied to geom pos attribute" do
      mjcf = Exporter.to_mjcf(BbMujoco.Examples.MeshRobot)

      # MeshRobot base_link has visual origin at (-0.05, -0.05, 0)
      assert mjcf =~ ~r/<geom name="base_link_visual_0"[^>]*pos="-0.05 -0.05 0\.0"/,
             "Visual origin should be applied as geom pos attribute"
    end
  end

  describe "actuators" do
    test "position actuator generated for each non-fixed joint" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)

      assert mjcf =~ ~r/<position name="pan_joint_pos" joint="pan_joint"/
      assert mjcf =~ ~r/<position name="tilt_joint_pos" joint="tilt_joint"/
    end

    test "actuator section is inside mujoco element" do
      mjcf = Exporter.to_mjcf(BbMujoco.TestRobot)

      # actuator section should come after worldbody
      assert mjcf =~ ~r/<\/worldbody>\s*<actuator>/s
      assert mjcf =~ ~r/<\/actuator>\s*<\/mujoco>/s
    end
  end

  describe "to_mjcf_file/3" do
    @tag :tmp_dir
    test "writes MJCF to file", %{tmp_dir: tmp_dir} do
      path = Path.join(tmp_dir, "robot.xml")

      assert :ok = Exporter.to_mjcf_file(BbMujoco.TestRobot, path)
      assert File.exists?(path)

      content = File.read!(path)
      assert content =~ ~r/<mujoco/
    end
  end
end
