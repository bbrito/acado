#!/bin/bash

cp -r ./generated_mpc ~/catkin_ws/src/lmpcc/src/

mv ~/catkin_ws/src/lmpcc/src/generated_mpc/acado_common.h ~/catkin_ws/src/lmpcc/include/lmpcc/generated_mpc/
mv ~/catkin_ws/src/lmpcc/src/generated_mpc/acado_auxiliary_functions.h ~/catkin_ws/src/lmpcc/include/lmpcc/generated_mpc/
mv ~/catkin_ws/src/lmpcc/src/generated_mpc/acado_qpoases_interface.hpp ~/catkin_ws/src/lmpcc/include/lmpcc/generated_mpc/

