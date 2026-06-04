idf_component_register(SRCS "main.c"
                              "User/servo_init.c"
                              "User/servo_set.c"
                              "User/servo_util.c"
                              "User/kinematics.c"
                              "User/arm_control.c"
                       INCLUDE_DIRS "."
                                     "User")
