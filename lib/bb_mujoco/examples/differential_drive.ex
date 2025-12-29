defmodule BbMujoco.Examples.DifferentialDrive do
  @moduledoc """
  A simple two-wheeled differential drive robot with a caster.

  Structure:
  - base_link (main chassis)
    - left_wheel (continuous joint)
    - right_wheel (continuous joint)
    - caster_wheel (fixed)
  """
  use BB
  import BB.Unit

  settings do
    name(:differential_drive)
  end

  topology do
    link :base_link do
      inertial do
        mass(~u(5 kilogram))

        inertia do
          ixx(~u(0.05 kilogram_square_meter))
          iyy(~u(0.05 kilogram_square_meter))
          izz(~u(0.08 kilogram_square_meter))
          ixy(~u(0 kilogram_square_meter))
          ixz(~u(0 kilogram_square_meter))
          iyz(~u(0 kilogram_square_meter))
        end
      end

      visual do
        box do
          x(~u(0.3 meter))
          y(~u(0.2 meter))
          z(~u(0.1 meter))
        end

        # Offset chassis to sit above floor (wheel_radius + half_chassis_height)
        origin do
          z(~u(0.1 meter))
        end

        material do
          name(:chassis_grey)

          color do
            red(0.5)
            green(0.5)
            blue(0.5)
            alpha(1.0)
          end
        end
      end

      joint :left_wheel_joint do
        type(:continuous)

        # Position wheel so it touches floor (z = wheel_radius = 0.05)
        origin do
          x(~u(0 meter))
          y(~u(0.12 meter))
          z(~u(0.05 meter))
        end

        axis do
          roll(~u(-90 degree))
        end

        limit do
          effort(~u(10 newton_meter))
          velocity(~u(360 degree_per_second))
        end

        link :left_wheel do
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
            origin do
              roll(~u(90 degree))
            end

            cylinder do
              radius(~u(0.05 meter))
              height(~u(0.02 meter))
            end

            material do
              name(:left_wheel_black)

              color do
                red(0.1)
                green(0.1)
                blue(0.1)
                alpha(1.0)
              end
            end
          end
        end
      end

      joint :right_wheel_joint do
        type(:continuous)

        origin do
          x(~u(0 meter))
          y(~u(-0.12 meter))
          z(~u(-0.03 meter))
        end

        axis do
          roll(~u(-90 degree))
        end

        limit do
          effort(~u(10 newton_meter))
          velocity(~u(360 degree_per_second))
        end

        link :right_wheel do
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
            origin do
              roll(~u(90 degree))
            end

            cylinder do
              radius(~u(0.05 meter))
              height(~u(0.02 meter))
            end

            material do
              name(:right_wheel_black)

              color do
                red(0.1)
                green(0.1)
                blue(0.1)
                alpha(1.0)
              end
            end
          end
        end
      end

      joint :caster_joint do
        type(:fixed)

        origin do
          x(~u(-0.1 meter))
          z(~u(-0.04 meter))
        end

        link :caster_wheel do
          visual do
            sphere do
              radius(~u(0.02 meter))
            end

            material do
              name(:caster_grey)

              color do
                red(0.3)
                green(0.3)
                blue(0.3)
                alpha(1.0)
              end
            end
          end
        end
      end
    end
  end
end
