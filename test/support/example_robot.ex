defmodule BbMujoco.TestRobot do
  @moduledoc """
  A simple test robot for BB.Mujoco testing.

  This is a minimal 2-DOF robot with:
  - Base link (fixed to world)
  - Pan joint (revolute, yaw)
  - Pan link
  - Tilt joint (revolute, pitch)
  - Tilt link (the "head")
  """

  use BB
  import BB.Unit

  settings do
    name(:test_robot)
  end

  topology do
    link :base_link do
      inertial do
        mass(~u(1 kilogram))

        inertia do
          ixx(~u(0.01 kilogram_square_meter))
          iyy(~u(0.01 kilogram_square_meter))
          izz(~u(0.01 kilogram_square_meter))
          ixy(~u(0 kilogram_square_meter))
          ixz(~u(0 kilogram_square_meter))
          iyz(~u(0 kilogram_square_meter))
        end
      end

      visual do
        box do
          x(~u(0.1 meter))
          y(~u(0.1 meter))
          z(~u(0.05 meter))
        end

        material do
          name(:base_grey)

          color do
            red(0.3)
            green(0.3)
            blue(0.3)
            alpha(1.0)
          end
        end
      end

      joint :pan_joint do
        type(:revolute)

        origin do
          x(~u(0 meter))
          y(~u(0 meter))
          z(~u(0.05 meter))
        end

        axis do
          yaw(~u(0 degree))
        end

        limit do
          lower(~u(-90 degree))
          upper(~u(90 degree))
          effort(~u(1 newton_meter))
          velocity(~u(1 radian_per_second))
        end

        link :pan_link do
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

          visual do
            cylinder do
              radius(~u(0.02 meter))
              height(~u(0.1 meter))
            end

            material do
              name(:pan_blue)

              color do
                red(0.6)
                green(0.6)
                blue(0.8)
                alpha(1.0)
              end
            end
          end

          joint :tilt_joint do
            type(:revolute)

            origin do
              x(~u(0 meter))
              y(~u(0 meter))
              z(~u(0.1 meter))
            end

            axis do
              pitch(~u(0 degree))
            end

            limit do
              lower(~u(-45 degree))
              upper(~u(45 degree))
              effort(~u(1 newton_meter))
              velocity(~u(1 radian_per_second))
            end

            link :tilt_link do
              inertial do
                mass(~u(0.3 kilogram))

                inertia do
                  ixx(~u(0.003 kilogram_square_meter))
                  iyy(~u(0.003 kilogram_square_meter))
                  izz(~u(0.003 kilogram_square_meter))
                  ixy(~u(0 kilogram_square_meter))
                  ixz(~u(0 kilogram_square_meter))
                  iyz(~u(0 kilogram_square_meter))
                end
              end

              visual do
                box do
                  x(~u(0.08 meter))
                  y(~u(0.06 meter))
                  z(~u(0.04 meter))
                end

                material do
                  name(:tilt_orange)

                  color do
                    red(0.8)
                    green(0.4)
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
  end
end
