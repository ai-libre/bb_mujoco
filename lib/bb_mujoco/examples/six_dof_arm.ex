defmodule BbMujoco.Examples.SixDofArm do
  @moduledoc """
  A 6 degree-of-freedom industrial robot arm.

  Structure:
  - base_link
    - shoulder_pan (revolute, Z-axis)
      - shoulder_lift (revolute, Y-axis)
        - elbow (revolute, Y-axis)
          - wrist_1 (revolute, Y-axis)
            - wrist_2 (revolute, Z-axis)
              - wrist_3 (revolute, Y-axis)
                - tool0 (fixed)
  """
  use BB
  import BB.Unit

  settings do
    name(:six_dof_arm)
  end

  topology do
    link :base_link do
      inertial do
        mass(~u(4 kilogram))

        inertia do
          ixx(~u(0.02 kilogram_square_meter))
          iyy(~u(0.02 kilogram_square_meter))
          izz(~u(0.02 kilogram_square_meter))
          ixy(~u(0 kilogram_square_meter))
          ixz(~u(0 kilogram_square_meter))
          iyz(~u(0 kilogram_square_meter))
        end
      end

      visual do
        cylinder do
          radius(~u(0.075 meter))
          height(~u(0.05 meter))
        end

        material do
          name(:arm_blue)

          color do
            red(0.2)
            green(0.4)
            blue(0.8)
            alpha(1.0)
          end
        end
      end

      joint :shoulder_pan_joint do
        type(:revolute)

        origin do
          z(~u(0.089 meter))
        end

        limit do
          lower(~u(-180 degree))
          upper(~u(180 degree))
          effort(~u(150 newton_meter))
          velocity(~u(180 degree_per_second))
        end

        link :shoulder_link do
          inertial do
            mass(~u(3.7 kilogram))

            origin do
              z(~u(0.05 meter))
            end

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
            origin do
              z(~u(0.05 meter))
            end

            cylinder do
              radius(~u(0.06 meter))
              height(~u(0.1 meter))
            end

            material do
              name(:shoulder_blue)

              color do
                red(0.2)
                green(0.4)
                blue(0.8)
                alpha(1.0)
              end
            end
          end

          joint :shoulder_lift_joint do
            type(:revolute)

            origin do
              y(~u(0.135 meter))
              z(~u(0.089 meter))
            end

            axis do
              roll(~u(-90 degree))
            end

            limit do
              lower(~u(-180 degree))
              upper(~u(180 degree))
              effort(~u(150 newton_meter))
              velocity(~u(180 degree_per_second))
            end

            link :upper_arm_link do
              inertial do
                mass(~u(8.4 kilogram))

                origin do
                  z(~u(0.2125 meter))
                end

                inertia do
                  ixx(~u(0.13 kilogram_square_meter))
                  iyy(~u(0.13 kilogram_square_meter))
                  izz(~u(0.02 kilogram_square_meter))
                  ixy(~u(0 kilogram_square_meter))
                  ixz(~u(0 kilogram_square_meter))
                  iyz(~u(0 kilogram_square_meter))
                end
              end

              visual do
                origin do
                  z(~u(0.2125 meter))
                end

                box do
                  x(~u(0.08 meter))
                  y(~u(0.08 meter))
                  z(~u(0.425 meter))
                end

                material do
                  name(:upper_arm_blue)

                  color do
                    red(0.2)
                    green(0.4)
                    blue(0.8)
                    alpha(1.0)
                  end
                end
              end

              joint :elbow_joint do
                type(:revolute)

                origin do
                  z(~u(0.425 meter))
                end

                axis do
                  roll(~u(-90 degree))
                end

                limit do
                  lower(~u(-180 degree))
                  upper(~u(180 degree))
                  effort(~u(28 newton_meter))
                  velocity(~u(180 degree_per_second))
                end

                link :forearm_link do
                  inertial do
                    mass(~u(2.3 kilogram))

                    origin do
                      z(~u(0.196 meter))
                    end

                    inertia do
                      ixx(~u(0.03 kilogram_square_meter))
                      iyy(~u(0.03 kilogram_square_meter))
                      izz(~u(0.004 kilogram_square_meter))
                      ixy(~u(0 kilogram_square_meter))
                      ixz(~u(0 kilogram_square_meter))
                      iyz(~u(0 kilogram_square_meter))
                    end
                  end

                  visual do
                    origin do
                      z(~u(0.196 meter))
                    end

                    box do
                      x(~u(0.06 meter))
                      y(~u(0.06 meter))
                      z(~u(0.392 meter))
                    end

                    material do
                      name(:forearm_blue)

                      color do
                        red(0.2)
                        green(0.4)
                        blue(0.8)
                        alpha(1.0)
                      end
                    end
                  end

                  joint :wrist_1_joint do
                    type(:revolute)

                    origin do
                      y(~u(0.093 meter))
                      z(~u(0.392 meter))
                    end

                    axis do
                      roll(~u(-90 degree))
                    end

                    limit do
                      lower(~u(-180 degree))
                      upper(~u(180 degree))
                      effort(~u(12 newton_meter))
                      velocity(~u(180 degree_per_second))
                    end

                    link :wrist_1_link do
                      inertial do
                        mass(~u(1.2 kilogram))

                        inertia do
                          ixx(~u(0.002 kilogram_square_meter))
                          iyy(~u(0.002 kilogram_square_meter))
                          izz(~u(0.002 kilogram_square_meter))
                          ixy(~u(0 kilogram_square_meter))
                          ixz(~u(0 kilogram_square_meter))
                          iyz(~u(0 kilogram_square_meter))
                        end
                      end

                      visual do
                        cylinder do
                          radius(~u(0.04 meter))
                          height(~u(0.08 meter))
                        end

                        material do
                          name(:wrist_grey)

                          color do
                            red(0.6)
                            green(0.6)
                            blue(0.6)
                            alpha(1.0)
                          end
                        end
                      end

                      joint :wrist_2_joint do
                        type(:revolute)

                        origin do
                          z(~u(0.093 meter))
                        end

                        limit do
                          lower(~u(-180 degree))
                          upper(~u(180 degree))
                          effort(~u(12 newton_meter))
                          velocity(~u(180 degree_per_second))
                        end

                        link :wrist_2_link do
                          inertial do
                            mass(~u(1.2 kilogram))

                            inertia do
                              ixx(~u(0.002 kilogram_square_meter))
                              iyy(~u(0.002 kilogram_square_meter))
                              izz(~u(0.002 kilogram_square_meter))
                              ixy(~u(0 kilogram_square_meter))
                              ixz(~u(0 kilogram_square_meter))
                              iyz(~u(0 kilogram_square_meter))
                            end
                          end

                          visual do
                            cylinder do
                              radius(~u(0.04 meter))
                              height(~u(0.08 meter))
                            end

                            material do
                              name(:wrist_2_grey)

                              color do
                                red(0.6)
                                green(0.6)
                                blue(0.6)
                                alpha(1.0)
                              end
                            end
                          end

                          joint :wrist_3_joint do
                            type(:revolute)

                            origin do
                              y(~u(0.093 meter))
                            end

                            axis do
                              roll(~u(-90 degree))
                            end

                            limit do
                              lower(~u(-180 degree))
                              upper(~u(180 degree))
                              effort(~u(12 newton_meter))
                              velocity(~u(180 degree_per_second))
                            end

                            link :wrist_3_link do
                              inertial do
                                mass(~u(0.2 kilogram))

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
                                cylinder do
                                  radius(~u(0.03 meter))
                                  height(~u(0.04 meter))
                                end

                                material do
                                  name(:wrist_3_grey)

                                  color do
                                    red(0.6)
                                    green(0.6)
                                    blue(0.6)
                                    alpha(1.0)
                                  end
                                end
                              end

                              joint :tool0_joint do
                                type(:fixed)

                                origin do
                                  z(~u(0.082 meter))
                                end

                                link :tool0 do
                                  visual do
                                    sphere do
                                      radius(~u(0.015 meter))
                                    end

                                    material do
                                      name(:tool_red)

                                      color do
                                        red(0.8)
                                        green(0.2)
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
