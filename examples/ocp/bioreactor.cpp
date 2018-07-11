/*
s file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


 /**
  *    \file   examples/ocp/bioreactor.cpp
  *    \author Boris Houska, Filip Logist, Rien Quirynen
  *    \date   2014
  */
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <acado_code_generation.hpp>
#include <cmath>

/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState     x,y,theta;
    Control               v,w   ;
    DifferentialEquation  f    ;

    OnlineData goal_x;
    OnlineData goal_y;
    OnlineData goal_theta;

    OnlineData wX;
    OnlineData wY;
    OnlineData wTheta;
    OnlineData wV;
    OnlineData wW;

    OnlineData wX_T;
    OnlineData wY_T;
    OnlineData wTheta_T;

    OnlineData r_disc;
    OnlineData disc_pos;

    OnlineData obst1_x;
    OnlineData obst1_y;
    OnlineData obst1_theta;
    OnlineData obst1_major;
    OnlineData obst1_minor;

    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    f << dot(x) == v*cos(theta);
    f << dot(y) == v*sin(theta);
    f << dot(theta) == w;

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( 0.0, 5, 100.0 );

    ocp.setNOD(18);

    ocp.minimizeLagrangeTerm(wX*(x-goal_x)*(x-goal_x)+ wY*(y-goal_y)*(y-goal_y)+ wTheta*(theta-goal_theta)*(theta-goal_theta)+wV*v*v+wW*w*w );  // weigh this with the physical cost!!!

    ocp.minimizeMayerTerm(wX_T*(x-goal_x)*(x-goal_x)+ wY_T*(y-goal_y)*(y-goal_y)+ wTheta_T*(theta-goal_theta)*(theta-goal_theta));
    
ocp.subjectTo( f );

    ocp.subjectTo( -1.0 <= v <= 1.0 );
    ocp.subjectTo( -1.0 <= w <= 1.0 );

    // DEFINE COLLISION CONSTRAINTS:
    // ---------------------------------------

    Expression ab(2,2);
    ab(0,0) = 1/((obst1_major + r_disc)*(obst1_major + r_disc));
    ab(0,1) = 0;
    ab(1,1) = 1/((obst1_minor + r_disc)*(obst1_minor + r_disc));
    ab(1,0) = 0;

    Expression R_obst(2,2);
    R_obst(0,0) = cos(obst1_theta);
    R_obst(0,1) = -sin(obst1_theta);
    R_obst(1,0) = sin(obst1_theta);
    R_obst(1,1) = cos(obst1_theta);

    Expression deltaPos_disc1(2,1);
    deltaPos_disc1(0) =  x - cos(obst1_theta)*disc_pos - obst1_x;
    deltaPos_disc1(1) =  y - sin(obst1_theta)*disc_pos - obst1_y;

    Expression deltaPos_disc2(2,1);
    deltaPos_disc2(0) = x + cos(obst1_theta)*disc_pos - obst1_x;
    deltaPos_disc2(1) = y + sin(obst1_theta)*disc_pos - obst1_y;

    Expression c_obst1_1, c_obst1_2;
    c_obst1_1 = deltaPos_disc1.transpose() * R_obst.transpose() * ab * R_obst * deltaPos_disc1;
    c_obst1_2 = deltaPos_disc2.transpose() * R_obst.transpose() * ab * R_obst * deltaPos_disc2;

    ocp.subjectTo(c_obst1_1 >= 1 );
    //ocp.subjectTo(c_obst1_2 >= 1);

	// DEFINE AN MPC EXPORT MODULE AND GENERATE THE CODE:
	// ----------------------------------------------------------
	OCPexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION,       EXACT_HESSIAN  		);
	mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING 	);
	mpc.set( INTEGRATOR_TYPE,             INT_RK4			);
	mpc.set( NUM_INTEGRATOR_STEPS,        100            		);
	mpc.set( QP_SOLVER,                   QP_QPOASES3    		);
	mpc.set( HOTSTART_QP,                 NO             		);
	mpc.set( GENERATE_TEST_FILE,          YES            		);
	mpc.set( GENERATE_MAKE_FILE,          YES            		);
	mpc.set( GENERATE_MATLAB_INTERFACE,   NO            		);
	mpc.set( SPARSE_QP_SOLUTION, 		  FULL_CONDENSING_N2	);
	mpc.set( DYNAMIC_SENSITIVITY, 		  SYMMETRIC				);
	mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO 					);
	mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, YES 				);

	mpc.exportCode( "generated_mpc" );
	mpc.printDimensionsQP( );
	// ----------------------------------------------------------
    return 0;
}
