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
    DifferentialState     x,y,psi,v;//, dummy;
    Control               a,delta;//, sv;
    DifferentialEquation  f    ;

    OnlineData goal_x;
    OnlineData goal_y;
    OnlineData goal_psi;

    OnlineData wX;
    OnlineData wY;
    OnlineData wPsi;
    OnlineData wa;
    OnlineData wdelta;

    OnlineData wX_T;
    OnlineData wY_T;
    OnlineData wPsi_T;

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

/*	double goal_x = 5.0;
	double goal_y =2.0;
	double goal_psi =0;

	double wX =1;
	double wY =1;
	double wPsi=1;
	double wa=1;
	double wdelta=1;

	double wX_T=1;
	double wY_T=1;
	double wPsi_T=1;
*/

	double lr=1.123; // distance from center of mass of the vehicle to the rear
	double lf=1.577; // distance from center of mass of the vehicle to the front
	double ratio =lr/(lf+lr);
	IntermediateState beta;
	beta = atan(ratio*tan(delta));
    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    f << dot(x) == v*cos(psi+beta);
    f << dot(y) == v*sin(psi+beta);
    f << dot(psi) == v/lr*sin(beta);
    f << dot(v) == a;

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( 0.0, 10.0, 25.0 );

    ocp.setNOD(25);

	ocp.minimizeLagrangeTerm(wX*(x-goal_x)*(x-goal_x)+ wY*(y-goal_y)*(y-goal_y)+ wPsi*(psi-goal_psi)*(psi-goal_psi)+wa*a*a+wdelta*delta*delta);//+ wP*((1/((x-obst1_x)*(x-obst1_x)+(y-obst1_y)*(y-obst1_y)+0.0001)) + (1/((x-obst2_x)*(x-obst2_x)+(y-obst2_y)*(y-obst2_y)+0.0001))));  // weigh this with the physical cost!!!
    ocp.minimizeMayerTerm(wX_T*(x-goal_x)*(x-goal_x)+ wY_T*(y-goal_y)*(y-goal_y)+ wPsi_T*(psi-goal_psi)*(psi-goal_psi));
    
    ocp.subjectTo( f );
	//only for simulation
	//ocp.subjectTo( AT_START, x ==  0.0 );
	//ocp.subjectTo( AT_START, v ==  0.0 );
	//ocp.subjectTo( AT_START, y ==  0.0 );
    ocp.subjectTo( -4 <= a <= 1.5 );
    ocp.subjectTo( -0.52 <= delta <= 0.52 );
	ocp.subjectTo( 0 <= v <= 13.8 );

    // DEFINE COLLISION CONSTRAINTS:
    // ---------------------------------------
	/*Expression ab_1(2,2);
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

	ocp.subjectTo(c_obst_1 >= 1);
	ocp.subjectTo(c_obst_2 >= 1);
*/
	// DEFINE A PLOT WINDOW:
	// ---------------------
	/*GnuplotWindow window;
		 window.addSubplot( x ,"X"  );
		 window.addSubplot( y ,"Y"  );
		 window.addSubplot( psi ,"Psi"  );
		 window.addSubplot( v,"V" );
		 window.addSubplot( a,"A" );
		 window.addSubplot( delta,"Delta" );
	 // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
	 // ---------------------------------------------------
	 OptimizationAlgorithm algorithm(ocp);
	 //RealTimeAlgorithm algorithm(ocp);
	 algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
	 algorithm.set(PRINTLEVEL, NONE);                       // default MEDIUM (NONE, MEDIUM, HIGH)
	 algorithm.set(PRINT_SCP_METHOD_PROFILE, false);        // default false
	 algorithm.set(PRINT_COPYRIGHT, false);                 // default true
	 algorithm.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
	 Grid t(0,10.0,25);
	 VariablesGrid s2(4,0,10.0,25),c2(2,0,10.0,25);
	 algorithm.initializeDifferentialStates(s2);
	 algorithm.initializeControls          (c2);

	 algorithm.set( MAX_NUM_ITERATIONS, 100 );
	 algorithm.set( KKT_TOLERANCE, 1e-3 );
	 algorithm << window;
	 //algorithm.solve(0.0,state_ini);
	 algorithm.solve();
	 VariablesGrid s3,c3;
	 algorithm.getDifferentialStates(s3);
	 algorithm.getControls          (c3);

*/
	// DEFINE AN MPC EXPORT MODULE AND GENERATE THE CODE:
	// ----------------------------------------------------------
	OCPexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION,       EXACT_HESSIAN  		);
    mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING 	);
	mpc.set( INTEGRATOR_TYPE,             INT_RK4			    );
//    mpc.set( INTEGRATOR_TYPE,             INT_IRK_GL4			);
	mpc.set( NUM_INTEGRATOR_STEPS,        25            		);
	mpc.set( QP_SOLVER,                   QP_QPOASES    		);
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
