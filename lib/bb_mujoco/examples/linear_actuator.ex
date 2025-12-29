defmodule BbMujoco.Examples.LinearActuator do
  @moduledoc """
  A simple linear actuator (prismatic joint example).

  Structure:
  - base_link
    - slider_joint (prismatic, Z-axis)
      - slider_link
  """
  use BB
  import BB.Unit

  settings do
    name(:linear_actuator)
  end

  topology do
    link :base_link do
      inertial do
        mass(~u(2 kilogram))

        inertia do
          ixx(~u(0.01 kilogram_square_meter))
          iyy(~u(0.01 kilogram_square_meter))
          izz(~u(0.005 kilogram_square_meter))
          ixy(~u(0 kilogram_square_meter))
          ixz(~u(0 kilogram_square_meter))
          iyz(~u(0 kilogram_square_meter))
        end
      end

      visual do
        box do
          x(~u(0.1 meter))
          y(~u(0.1 meter))
          z(~u(0.3 meter))
        end

        # Offset to sit on floor (half-height of box)
        origin do
          z(~u(0.15 meter))
        end

        material do
          name(:actuator_blue)

          color do
            red(0.2)
            green(0.3)
            blue(0.7)
            alpha(1.0)
          end
        end
      end

      joint :slider_joint do
        type(:prismatic)

        # Joint origin at top of base box (0.3m = box height)
        origin do
          z(~u(0.3 meter))
        end

        limit do
          lower(~u(0 meter))
          upper(~u(0.2 meter))
          effort(~u(100 newton))
          velocity(~u(0.1 meter_per_second))
        end

        link :slider_link do
          inertial do
            mass(~u(0.5 kilogram))

            inertia do
              ixx(~u(0.001 kilogram_square_meter))
              iyy(~u(0.001 kilogram_square_meter))
              izz(~u(0.001 kilogram_square_meter))
              ixy(~u(0 kilogram_square_meter))
              ixz(~u(0 kilogram_square_meter))
              iyz(~u(0 kilogram_square_meter))
            end
          end

          visual do
            cylinder do
              radius(~u(0.03 meter))
              height(~u(0.15 meter))
            end

            # Offset cylinder to extend upward from joint origin
            origin do
              z(~u(0.075 meter))
            end

            material do
              name(:slider_silver)

              color do
                red(0.8)
                green(0.8)
                blue(0.8)
                alpha(1.0)
              end
            end
          end
        end
      end
    end
  end
end
