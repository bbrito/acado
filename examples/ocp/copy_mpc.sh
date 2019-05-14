#!/bin/bash

cp -r ./ocp ~/thesis/workspace/src/lmpcc/src/

mv ~/thesis/workspace/src/lmpcc/src/ocp/acado_common.h ~/thesis/workspace/src/lmpcc/include/lmpcc/ocp/
mv ~/thesis/workspace/src/lmpcc/src/ocp/acado_auxiliary_functions.h ~/thesis/workspace/src/lmpcc/include/lmpcc/ocp/
mv ~/thesis/workspace/src/lmpcc/src/ocp/acado_qpoases_interface.hpp ~/thesis/workspace/src/lmpcc/include/lmpcc/ocp/
