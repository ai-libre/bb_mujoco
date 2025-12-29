defmodule BbMujoco.Examples.PanTiltRobot do
  @moduledoc """
  A simple pan-tilt camera mount.

  Structure:
  - base_link
    - pan_joint (revolute, Z-axis)
      - pan_link
        - tilt_joint (revolute, Y-axis)
          - camera_link
  """
  use BB
  import BB.Unit

  settings do
    name(:pan_tilt_camera)
  end

  topology do
    link :base_link do
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
          height(~u(0.02 meter))
        end

        material do
          name(:base_black)

          color do
            red(0.1)
            green(0.1)
            blue(0.1)
            alpha(1.0)
          end
        end
      end

      joint :pan_joint do
        type(:revolute)

        origin do
          z(~u(0.015 meter))
        end

        limit do
          lower(~u(-170 degree))
          upper(~u(170 degree))
          effort(~u(2 newton_meter))
          velocity(~u(90 degree_per_second))
        end

        link :pan_link do
          inertial do
            mass(~u(0.1 kilogram))

            inertia do
              ixx(~u(0.0001 kilogram_square_meter))
              iyy(~u(0.0001 kilogram_square_meter))
              izz(~u(0.0001 kilogram_square_meter))
              ixy(~u(0 kilogram_square_meter))
              ixz(~u(0 kilogram_square_meter))
              iyz(~u(0 kilogram_square_meter))
            end
          end

          visual do
            origin do
              z(~u(0.015 meter))
            end

            box do
              x(~u(0.04 meter))
              y(~u(0.04 meter))
              z(~u(0.03 meter))
            end

            material do
              name(:pan_black)

              color do
                red(0.1)
                green(0.1)
                blue(0.1)
                alpha(1.0)
              end
            end
          end

          joint :tilt_joint do
            type(:revolute)

            origin do
              z(~u(0.035 meter))
            end

            axis do
              roll(~u(-90 degree))
            end

            limit do
              lower(~u(-45 degree))
              upper(~u(90 degree))
              effort(~u(1 newton_meter))
              velocity(~u(60 degree_per_second))
            end

            link :camera_link do
              inertial do
                mass(~u(0.15 kilogram))

                origin do
                  x(~u(0.02 meter))
                end

                inertia do
                  ixx(~u(0.0001 kilogram_square_meter))
                  iyy(~u(0.0001 kilogram_square_meter))
                  izz(~u(0.0001 kilogram_square_meter))
                  ixy(~u(0 kilogram_square_meter))
                  ixz(~u(0 kilogram_square_meter))
                  iyz(~u(0 kilogram_square_meter))
                end
              end

              visual do
                origin do
                  x(~u(0.02 meter))
                end

                box do
                  x(~u(0.04 meter))
                  y(~u(0.06 meter))
                  z(~u(0.03 meter))
                end

                material do
                  name(:camera_silver)

                  color do
                    red(0.7)
                    green(0.7)
                    blue(0.7)
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
