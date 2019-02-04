/*
s file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free s0ftware; you can redistribute it and/or
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

/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState     x,y,theta,dummy1,dummy2;
    Control               v,w,sv1,sv2;
    DifferentialEquation  f;

	OnlineData x_goal;
	OnlineData y_goal;
	OnlineData theta_goal;

	OnlineData Wx;
	OnlineData Wy;
	OnlineData Wtheta;
	OnlineData Wv;
	OnlineData Ww;

	OnlineData vref;

	OnlineData ws;
	OnlineData wP;

	OnlineData r_disc;
	OnlineData disc_pos;

	OnlineData obst1_x;
	OnlineData obst1_y;
	OnlineData obst1_theta;
	OnlineData obst1_major;
	OnlineData obst1_minor;

	OnlineData obst2_x;
	OnlineData obst2_y;
	OnlineData obst2_theta;
	OnlineData obst2_major;
	OnlineData obst2_minor;

	OnlineData collision_free_xmin;
	OnlineData collision_free_xmax;
	OnlineData collision_free_ymin;
	OnlineData collision_free_ymax;

	OnlineData collision_free_a1x;
	OnlineData collision_free_a2x;
	OnlineData collision_free_a3x;
	OnlineData collision_free_a4x;

	OnlineData collision_free_a1y;
	OnlineData collision_free_a2y;
	OnlineData collision_free_a3y;
	OnlineData collision_free_a4y;

	OnlineData collision_free_C1;
	OnlineData collision_free_C2;
	OnlineData collision_free_C3;
	OnlineData collision_free_C4;

    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    
    f << dot(x) == v*cos(theta);
    f << dot(y) == v*sin(theta);
    f << dot(theta) == w;
	f << dot(dummy1) == sv1;
    f << dot(dummy2) == sv2;

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( 0.0, 5, 25.0 );

    // Need to set the number of online variables!
    ocp.setNOD(39);

	ocp.minimizeLagrangeTerm(Wx*(x-x_goal)*(x-x_goal) + ws*sv1*sv1 + ws*sv2*sv2 + Wy*(y-y_goal)*(y-y_goal) + Wtheta*(theta-theta_goal)*(theta-theta_goal)+ Ww*w*w +Wv*(v-vref)*(v-vref) + wP*((1/((x-obst1_x)*(x-obst1_x)+(y-obst1_y)*(y-obst1_y)+0.0001)) + (1/((x-obst2_x)*(x-obst2_x)+(y-obst2_y)*(y-obst2_y)+0.0001)))); // weight this with the physical cost!!!
	ocp.setModel(f);

    ocp.subjectTo( -2.0 <= v <= 2.0 );
    ocp.subjectTo( -1.0 <= w <= 1.0 );

    // DEFINE COLLISION CONSTRAINTS:
	// ---------------------------------------

	Expression ab_1(2,2);
	ab_1(0,0) = 1/((obst1_major + r_disc)*(obst1_major + r_disc));
	ab_1(0,1) = 0;
	ab_1(1,1) = 1/((obst1_minor + r_disc)*(obst1_minor + r_disc));
	ab_1(1,0) = 0;

	Expression ab_2(2,2);
	ab_2(0,0) = 1/((obst2_major + r_disc)*(obst2_major + r_disc));
	ab_2(0,1) = 0;
	ab_2(1,1) = 1/((obst2_minor + r_disc)*(obst2_minor + r_disc));
	ab_2(1,0) = 0;

	Expression R_obst_1(2,2);
	R_obst_1(0,0) = cos(obst1_theta);
	R_obst_1(0,1) = -sin(obst1_theta);
	R_obst_1(1,0) = sin(obst1_theta);
	R_obst_1(1,1) = cos(obst1_theta);

	Expression R_obst_2(2,2);
	R_obst_2(0,0) = cos(obst2_theta);
	R_obst_2(0,1) = -sin(obst2_theta);
	R_obst_2(1,0) = sin(obst2_theta);
	R_obst_2(1,1) = cos(obst2_theta);

	Expression deltaPos_disc_1(2,1);
	deltaPos_disc_1(0) =  x - obst1_x;
	deltaPos_disc_1(1) =  y - obst1_y;

	Expression deltaPos_disc_2(2,1);
	deltaPos_disc_2(0) =  x - obst2_x;
	deltaPos_disc_2(1) =  y - obst2_y;

	Expression c_obst_1, c_obst_2;
	c_obst_1 = deltaPos_disc_1.transpose() * R_obst_1.transpose() * ab_1 * R_obst_1 * deltaPos_disc_1;
	c_obst_2 = deltaPos_disc_2.transpose() * R_obst_2.transpose() * ab_2 * R_obst_2 * deltaPos_disc_2;

	ocp.subjectTo(c_obst_1 + sv1 >= 1);
	ocp.subjectTo(c_obst_2 + sv1 >= 1);

    ocp.subjectTo( x*collision_free_a1x + y*collision_free_a1y - collision_free_C1 + sv2 >= 0 );
    ocp.subjectTo( x*collision_free_a2x + y*collision_free_a2y - collision_free_C2 + sv2 >= 0 );
    ocp.subjectTo( x*collision_free_a3x + y*collision_free_a3y - collision_free_C3 + sv2 >= 0 );
    ocp.subjectTo( x*collision_free_a4x + y*collision_free_a4y - collision_free_C4 + sv2 >= 0 );

    // DEFINE AN MPC EXPORT MODULE AND GENERATE THE CODE:
	// ----------------------------------------------------------
	OCPexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION,            EXACT_HESSIAN  		);
	mpc.set( DISCRETIZATION_TYPE,              MULTIPLE_SHOOTING 	);
	mpc.set( INTEGRATOR_TYPE,                  INT_RK4			    );
	mpc.set( NUM_INTEGRATOR_STEPS,             25            		);
	mpc.set( QP_SOLVER,                        QP_QPOASES    		);
	mpc.set( HOTSTART_QP,                      NO             		);
	mpc.set( GENERATE_TEST_FILE,               YES            		);
	mpc.set( GENERATE_MAKE_FILE,               YES            		);
	mpc.set( GENERATE_MATLAB_INTERFACE,        NO            		);
	mpc.set( SPARSE_QP_SOLUTION, 		       FULL_CONDENSING_N2	);
	mpc.set( DYNAMIC_SENSITIVITY, 		       SYMMETRIC			);
	mpc.set( CG_HARDCODE_CONSTRAINT_VALUES,    NO 					);
	mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, YES 					);

	mpc.exportCode( "ocp" );
	mpc.printDimensionsQP( );
	// ----------------------------------------------------------


    return 0;
}
