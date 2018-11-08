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
  *    \file   examples/ocp/lmpcc.cpp
  *    \author Bruno Brito, Laura Ferranti, Boaz Floor, Javier Alonso-Mora
  *    \date   November 2018
  */
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <acado_code_generation.hpp>

/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState     x,y,psi,v,s, dummy;
    Control               a,delta, sv;
    DifferentialEquation  f    ;
    
    // Parameters required for the spline
	OnlineData a_X1;
	OnlineData b_X1;
	OnlineData c_X1;
	OnlineData d_X1;
	OnlineData a_Y1;
	OnlineData b_Y1;
	OnlineData c_Y1;
	OnlineData d_Y1;

	OnlineData a_X2;
	OnlineData b_X2;
	OnlineData c_X2;
	OnlineData d_X2;
	OnlineData a_Y2;
	OnlineData b_Y2;
	OnlineData c_Y2;
	OnlineData d_Y2;
	
	OnlineData s01;
	OnlineData s02;
	
	// d: used to evaluate lambda (equivalent of delta in the Jackal version)
	OnlineData d;

    // Wx: weight of contour error 
    OnlineData Wx;
    // Wy: weight of lag error
	OnlineData Wy;
	// Wv: weight on the desired acceleration
	OnlineData Wa;
	// Wdelta: weight on the delta command
	OnlineData Wdelta;
	
	// vref1, vref2: to define the desired reference speed
	OnlineData vref1;
	OnlineData vref2;
	
    // ws: weight on the slack variable sv
	OnlineData ws;
	// Wp: for the potential field associated with the obstacles in the cost
	OnlineData wP;

	// Wv: weight on the desired acceleration
	OnlineData Wv;

	Expression lambda = 1/(1 + exp((s - d)/0.1));

	Expression x_path1 = (a_X1*(s-s01)*(s-s01)*(s-s01) + b_X1*(s-s01)*(s-s01) + c_X1*(s-s01) + d_X1) ;
	Expression y_path1 = (a_Y1*(s-s01)*(s-s01)*(s-s01) + b_Y1*(s-s01)*(s-s01) + c_Y1*(s-s01) + d_Y1) ;
	Expression dx_path1 = (3*a_X1*(s-s01)*(s-s01) + 2*b_X1*(s-s01) + c_X1) ;
	Expression dy_path1 = (3*a_Y1*(s-s01)*(s-s01) + 2*b_Y1*(s-s01) + c_Y1) ;

	Expression x_path2 = (a_X2*(s-s02)*(s-s02)*(s-s02) + b_X2*(s-s02)*(s-s02) + c_X2*(s-s02) + d_X2) ;
	Expression y_path2 = (a_Y2*(s-s02)*(s-s02)*(s-s02) + b_Y2*(s-s02)*(s-s02) + c_Y2*(s-s02) + d_Y2) ;
	Expression dx_path2 = (3*a_X2*(s-s02)*(s-s02) + 2*b_X2*(s-s02) + c_X2) ;
	Expression dy_path2 = (3*a_Y2*(s-s02)*(s-s02) + 2*b_Y2*(s-s02) + c_Y2) ;

	Expression x_path = lambda*x_path1 + (1 - lambda)*x_path2;
	Expression y_path = lambda*y_path1 + (1 - lambda)*y_path2;
	Expression dx_path = lambda*dx_path1 + (1 - lambda)*dx_path2;
	Expression dy_path = lambda*dy_path1 + (1 - lambda)*dy_path2;

	Expression abs_grad = sqrt(dx_path.getPowInt(2) + dy_path.getPowInt(2));
	Expression dx_path_norm = dx_path/abs_grad;
	Expression dy_path_norm =  dy_path/abs_grad;

	Expression vref = lambda*vref1 + (1 - lambda)*vref2;
	
	double lr=1.123; // distance from center of mass of the vehicle to the rear
	double lf=1.577; // distance from center of mass of the vehicle to the front
	double ratio =lr/(lf+lr);
	double length = 4.540; // car length in m
	double width = 1.760; // car width in m
	
	IntermediateState beta;
	beta = atan(ratio*tan(delta));

    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    
    f << dot(x) == v*cos(psi+beta);
    f << dot(y) == v*sin(psi+beta);
    f << dot(psi) == v/lr*sin(beta);
    f << dot(v) == a;
    f << dot(s) == v;
	f << dot(dummy) == sv;

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    // OCP ocp( 0.0, 5.0, 25.0 );
    OCP ocp( 0.0, 2, 25.0 );

    // Need to set the number of online variables!
    ocp.setNOD(44);

	Expression error_contour   = dy_path_norm * (x - x_path) - dx_path_norm * (y - y_path);

	Expression error_lag       = -dx_path_norm * (x - x_path) - dy_path_norm * (y - y_path);
	
	Expression road_boundary = -dy_path_norm * (x - x_path) + dx_path_norm * (y - y_path);


    ocp.subjectTo( -6 <= a <= 1.5 );
    ocp.subjectTo( -0.52 <= delta <= 0.52 );
	ocp.subjectTo( 0 <= v <= 13.8 );
	// to test if the car stops with stricter road boundaries uncomment
	//ocp.subjectTo(road_boundary  <= .75+.88);
	ocp.subjectTo(road_boundary -sv<= 6);
	ocp.subjectTo(-road_boundary-sv<= 6);


	ocp.subjectTo(sv >= 0);
	
	//	ocp.minimizeLagrangeTerm(Wx*error_contour*error_contour + Wy*error_lag*error_lag +Wv*(v-vref)*(v-vref) +Wa*a*a+ Wdelta*(delta)*(delta)+ws*sv+wP*((1/(deltaPos_disc_1_obstacle_1.transpose()*deltaPos_disc_1_obstacle_1+0.01))+((1/(deltaPos_disc_2_obstacle_1.transpose()*deltaPos_disc_2_obstacle_1+0.01)))+((1/(deltaPos_disc_3_obstacle_1.transpose()*deltaPos_disc_3_obstacle_1+0.01)))+(1/(deltaPos_disc_1_obstacle_2.transpose()*deltaPos_disc_1_obstacle_2+0.01))+((1/(deltaPos_disc_2_obstacle_2.transpose()*deltaPos_disc_2_obstacle_2+0.01)))+((1/(deltaPos_disc_3_obstacle_2.transpose()*deltaPos_disc_3_obstacle_2+0.01))))); // weight this with the physical cost!!!
	ocp.minimizeLagrangeTerm(Wx*error_contour*error_contour + Wy*error_lag*error_lag +Wv*(v-vref)*(v-vref) +Wa*a*a+ Wdelta*(delta)*(delta)+ws*sv); // weight this with the physical cost!!!
	ocp.setModel(f);


    // DEFINE AN MPC EXPORT MODULE AND GENERATE THE CODE:
	// ----------------------------------------------------------
	OCPexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION,       EXACT_HESSIAN  		);
	mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING 	);
	mpc.set( INTEGRATOR_TYPE,             INT_RK4			);
	mpc.set( NUM_INTEGRATOR_STEPS,        16            		);
	mpc.set( QP_SOLVER,                   QP_QPOASES    		);
	mpc.set( HOTSTART_QP,                 NO             		);
	mpc.set( GENERATE_TEST_FILE,          YES            		);
	mpc.set( GENERATE_MAKE_FILE,          YES            		);
	mpc.set( GENERATE_MATLAB_INTERFACE,   NO            		);
	mpc.set( SPARSE_QP_SOLUTION, 		  FULL_CONDENSING_N2	);
	mpc.set( DYNAMIC_SENSITIVITY, 		  SYMMETRIC				);
	mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO 					);
	mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, YES 				);

	mpc.exportCode( "generated_lmpcc" );
	mpc.printDimensionsQP( );
	// ----------------------------------------------------------


    return 0;
}
