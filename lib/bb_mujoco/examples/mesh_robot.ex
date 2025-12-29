defmodule BbMujoco.Examples.MeshRobot do
  @moduledoc """
  A robot demonstrating mesh (STL) geometry support.

  This robot uses STL mesh files for visual geometry instead of
  primitive shapes (box, cylinder, sphere).

  Structure:
  - Base link with cube mesh
  - Pan joint (yaw rotation)
  - Head link with head mesh

  Mesh files are stored in priv/assets/:
  - cube.stl - Unit cube for the base
  - head.stl - Rectangular head shape
  """

  use BB
  import BB.Unit

  settings do
    name(:mesh_robot)
  end

  topology do
    link :base_link do
      inertial do
        mass(~u(2 kilogram))

        inertia do
          ixx(~u(0.02 kilogram_square_meter))
          iyy(~u(0.02 kilogram_square_meter))
          izz(~u(0.02 kilogram_square_meter))
          ixy(~u(0 kilogram_square_meter))
          ixz(~u(0 kilogram_square_meter))
          iyz(~u(0 kilogram_square_meter))
        end
      end

      # Using mesh geometry for visual
      visual do
        mesh do
          filename("cube.stl")
          scale(0.1)
        end

        origin do
          x(~u(-0.05 meter))
          y(~u(-0.05 meter))
          z(~u(0 meter))
        end

        material do
          name(:base_metal)

          color do
            red(0.4)
            green(0.4)
            blue(0.5)
            alpha(1.0)
          end
        end
      end

      joint :pan_joint do
        type(:revolute)

        origin do
          x(~u(0 meter))
          y(~u(0 meter))
          z(~u(0.1 meter))
        end

        axis do
          yaw(~u(0 degree))
        end

        limit do
          lower(~u(-180 degree))
          upper(~u(180 degree))
          effort(~u(2 newton_meter))
          velocity(~u(2 radian_per_second))
        end

        link :head_link do
          inertial do
            mass(~u(0.5 kilogram))

            inertia do
              ixx(~u(0.005 kilogram_square_meter))
              iyy(~u(0.005 kilogram_square_meter))
              izz(~u(0.005 kilogram_square_meter))
              ixy(~u(0 kilogram_square_meter))
              ixz(~u(0 kilogram_square_meter))
              iyz(~u(0 kilogram_square_meter))
            end
          end

          # Using mesh geometry for head visual
          visual do
            mesh do
              filename("head.stl")
              scale(1.0)
            end

            material do
              name(:head_orange)

              color do
                red(0.9)
                green(0.5)
                blue(0.2)
                alpha(1.0)
              end
            end
          end
        end
      end
    end
  end
end
