#!/bin/bash

cp -r ./generated_mpc ~/ros/catkin_ws/src/lmpcc/src/

mv ~/ros/catkin_ws/src/lmpcc/src/generated_mpc/acado_common.h ~/ros/catkin_ws/src/lmpcc/include/lmpcc/generated_mpc/
mv ~/ros/catkin_ws/src/lmpcc/src/generated_mpc/acado_auxiliary_functions.h ~/ros/catkin_ws/src/lmpcc/include/lmpcc/generated_mpc/
mv ~/ros/catkin_ws/src/lmpcc/src/generated_mpc/acado_qpoases_interface.hpp ~/ros/catkin_ws/src/lmpcc/include/lmpcc/generated_mpc/

