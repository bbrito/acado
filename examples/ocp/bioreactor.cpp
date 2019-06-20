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
	OnlineData vref;

	// ws: weight on the slack variable sv
	OnlineData ws;
	// Wp: for the potential field associated with the obstacles in the cost
	OnlineData wP;
	OnlineData r_disc;
	OnlineData disc_pos_0;
	//Description of Obstacle 1
	OnlineData obst1_x;
	OnlineData obst1_y;
	OnlineData obst1_theta;
	// obst1_major:  major axis of the ellipse defining the obstacle
	OnlineData obst1_major;
	// obst1_minor:  minor axis of the ellipse defining the obstacle
	OnlineData obst1_minor;

	//Description of Obstacle 1
	OnlineData obst2_x;
	OnlineData obst2_y;
	OnlineData obst2_theta;
	OnlineData obst2_major;
	OnlineData obst2_minor;
	// Wv: weight on the desired acceleration
	OnlineData Wv;
	OnlineData disc_pos_1;
	OnlineData disc_pos_2;
	OnlineData right_offset;
	OnlineData left_offset;

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

	double lf=1.123; // distance from center of mass of the vehicle to the front
	double lr=1.577; // distance from center of mass of the vehicle to the rear
	double ratio = lr/(lf+lr); // 1

	// not used
	//double length = 4.540; // car length in m
	//double width = 1.760; // car width in m

	// DEFINE A DIFFERENTIAL EQUATION:
	// -------------------------------

	f << dot(x) == v*cos(psi+delta);
	f << dot(y) == v*sin(psi+delta);
	f << dot(psi) == v/lr*sin(delta);
	f << dot(v) == a;
	f << dot(s) == v;
	f << dot(dummy) == sv;

	// DEFINE AN OPTIMAL CONTROL PROBLEM:
	// ----------------------------------
	// OCP ocp( 0.0, 5.0, 25.0 );
	OCP ocp( 0.0, 4.0, 20.0 );

	// Need to set the number of online variables!
	ocp.setNOD(43);

	Expression error_contour   = dy_path_norm * (x - x_path) - dx_path_norm * (y - y_path);

	Expression error_lag       = -dx_path_norm * (x - x_path) - dy_path_norm * (y - y_path);

	Expression road_boundary = -dy_path_norm * (x - x_path) + dx_path_norm * (y - y_path);

	Expression R_car(2,2);
	R_car(0,0) = cos(psi);
	R_car(0,1) = -sin(psi);
	R_car(1,0) = sin(psi);
	R_car(1,1) = cos(psi);

	Expression CoG(2,1);
	CoG(0) = x;
	CoG(1) = y;

	Expression shift_1(2,1);
	shift_1(0) = disc_pos_0;//-0.255;
	shift_1(1) = 0;
	Expression shift_2(2,1);
	shift_2(0) = disc_pos_1;//2.64;
	shift_2(1) = 0;
	Expression shift_3(2,1);
	shift_3(0) = disc_pos_2;//4.4;
	shift_3(1) = 0;


	Expression position_disc_1(2,1);
	position_disc_1 = CoG+R_car*shift_1;

	Expression position_disc_2(2,1);
	position_disc_2 = CoG+R_car*shift_2;

	Expression position_disc_3(2,1);
	position_disc_3 = CoG+R_car*shift_3;



	// For Obstacle 1

	Expression CoG_obst1(2,1);
	CoG_obst1(0) = obst1_x;
	CoG_obst1(1) = obst1_y;

	Expression deltaPos_disc_1_obstacle_1(2,1);
	deltaPos_disc_1_obstacle_1 =  position_disc_1 - CoG_obst1;

	Expression deltaPos_disc_2_obstacle_1(2,1);
	deltaPos_disc_2_obstacle_1 =  position_disc_2 - CoG_obst1;

	Expression deltaPos_disc_3_obstacle_1(2,1);
	deltaPos_disc_3_obstacle_1 =  position_disc_3 - CoG_obst1;


	// For Obstacle 2

	Expression CoG_obst2(2,1);
	CoG_obst2(0) = obst2_x;
	CoG_obst2(1) = obst2_y;

	Expression deltaPos_disc_1_obstacle_2(2,1);
	deltaPos_disc_1_obstacle_2 =  position_disc_1 - CoG_obst2;

	Expression deltaPos_disc_2_obstacle_2(2,1);
	deltaPos_disc_2_obstacle_2 =  position_disc_2 - CoG_obst2;

	Expression deltaPos_disc_3_obstacle_2(2,1);
	deltaPos_disc_3_obstacle_2 =  position_disc_3 - CoG_obst2;





	ocp.subjectTo( -6 <= a <= 1 );
	ocp.subjectTo( -0.52 <= delta <= 0.52 );
	ocp.subjectTo( 0 <= v <= 13.8 );
	//ocp.subjectTo( -10 <= jerk <= 10 );
	//ocp.subjectTo( -0.5 <= delta_rate <= 0.5 );
	// to test if the car stops with stricter road boundaries uncomment
	//ocp.subjectTo(road_boundary  <= 1.5+.88);
	ocp.subjectTo(road_boundary -left_offset<= 4.24);
	ocp.subjectTo(-road_boundary-right_offset<= 1.88);

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


	Expression c_disc_1_obst_1, c_disc_2_obst_1, c_disc_3_obst_1, c_disc_1_obst_2, c_disc_2_obst_2, c_disc_3_obst_2;
	c_disc_1_obst_1 = deltaPos_disc_1_obstacle_1.transpose() * R_obst_1.transpose() * ab_1 * R_obst_1 * deltaPos_disc_1_obstacle_1;
	c_disc_2_obst_1 = deltaPos_disc_2_obstacle_1.transpose() * R_obst_1.transpose() * ab_1 * R_obst_1 * deltaPos_disc_2_obstacle_1;
	c_disc_3_obst_1 = deltaPos_disc_3_obstacle_1.transpose() * R_obst_1.transpose() * ab_1 * R_obst_1 * deltaPos_disc_3_obstacle_1;

	c_disc_1_obst_2 = deltaPos_disc_1_obstacle_2.transpose() * R_obst_2.transpose() * ab_2 * R_obst_2 * deltaPos_disc_1_obstacle_2;
	c_disc_2_obst_2 = deltaPos_disc_2_obstacle_2.transpose() * R_obst_2.transpose() * ab_2 * R_obst_2 * deltaPos_disc_2_obstacle_2;
	c_disc_3_obst_2 = deltaPos_disc_3_obstacle_2.transpose() * R_obst_2.transpose() * ab_2 * R_obst_2 * deltaPos_disc_3_obstacle_2;

	//ocp.subjectTo(c_disc_1_obst_1 + sv >= 1);
	//ocp.subjectTo(c_disc_2_obst_1 + sv >= 1);
	//ocp.subjectTo(c_disc_3_obst_1 + sv >= 1);
	//ocp.subjectTo(c_disc_1_obst_2 + sv >= 1);
	//ocp.subjectTo(c_disc_2_obst_2 + sv >= 1);
	//ocp.subjectTo(c_disc_3_obst_2 + sv >= 1);
	ocp.subjectTo(c_disc_1_obst_1 + sv >= 1);
	ocp.subjectTo(c_disc_2_obst_1 + sv >= 1);
	ocp.subjectTo(c_disc_3_obst_1 + sv >= 1);
	//ocp.subjectTo(c_disc_4_obst_1 + sv >= 1);
	//ocp.subjectTo(c_disc_5_obst_1 + sv >= 1);
	ocp.subjectTo(c_disc_1_obst_2 + sv >= 1);
	ocp.subjectTo(c_disc_2_obst_2 + sv >= 1);
	ocp.subjectTo(c_disc_3_obst_2 + sv >= 1);
	//ocp.subjectTo(c_disc_4_obst_2 + sv >= 1);
	//ocp.subjectTo(c_disc_5_obst_2 + sv >= 1);
	ocp.subjectTo(sv >= 0);

	// road boundary cost
	float road_offset_left = -4.24;
	float road_offset_right = 1.88;
	int steepness=5;
    int steepness_right=5;
    int intersect_cost=100;
    int exp_addition=log(intersect_cost)/steepness;
    int exp_addition_right=log(intersect_cost)/steepness_right;
	//	ocp.minimizeLagrangeTerm(Wx*error_contour*error_contour + Wy*error_lag*error_lag +Wv*(v-vref)*(v-vref) +Wa*a*a+ Wdelta*(delta)*(delta)+ws*sv+wP*((1/(deltaPos_disc_1_obstacle_1.transpose()*deltaPos_disc_1_obstacle_1+0.01))+((1/(deltaPos_disc_2_obstacle_1.transpose()*deltaPos_disc_2_obstacle_1+0.01)))+((1/(deltaPos_disc_3_obstacle_1.transpose()*deltaPos_disc_3_obstacle_1+0.01)))+(1/(deltaPos_disc_1_obstacle_2.transpose()*deltaPos_disc_1_obstacle_2+0.01))+((1/(deltaPos_disc_2_obstacle_2.transpose()*deltaPos_disc_2_obstacle_2+0.01)))+((1/(deltaPos_disc_3_obstacle_2.transpose()*deltaPos_disc_3_obstacle_2+0.01))))); // weight this with the physical cost!!!
	//ocp.minimizeLagrangeTerm(Wx*error_contour*error_contour + Wy*error_lag*error_lag +Wv*(v-vref)*(v-vref) +Wa*a*a+ Wdelta*(delta)*(delta)+ws*sv+wP*(1/((1-c_disc_1_obst_1)*(1-c_disc_1_obst_1)+.001))+wP*(1/((1-c_disc_2_obst_1)*(1-c_disc_2_obst_1)+.001))+wP*(1/((1-c_disc_3_obst_1)*(1-c_disc_3_obst_1)+.001))+wP*(1/((1-c_disc_1_obst_2)*(1-c_disc_1_obst_2)+.001))+wP*(1/((1-c_disc_2_obst_2)*(1-c_disc_2_obst_2)+.001))+wP*(1/((1-c_disc_3_obst_2)*(1-c_disc_3_obst_2)+.001))+10000*exp((road_boundary-2.15-1)/0.1)+10000*exp(-(road_boundary+5.12)/0.1));//+0*exp((-road_boundary+1)/0.1)+dot(a)*dot(a) // weight this with the physical cost!!!
	ocp.minimizeLagrangeTerm(
			Wx*error_contour*error_contour +
			Wy*error_lag*error_lag +
			Wv*(v-vref)*(v-vref) +
			Wa*a*a+ Wdelta*(delta)*(delta)+
			ws*sv+
			wP*(1/((1-c_disc_1_obst_1)*(1-c_disc_1_obst_1)+.001))+
			wP*(1/((1-c_disc_2_obst_1)*(1-c_disc_2_obst_1)+.001))+
			wP*(1/((1-c_disc_3_obst_1)*(1-c_disc_3_obst_1)+.001))+
			wP*(1/((1-c_disc_1_obst_2)*(1-c_disc_1_obst_2)+.001))+
			wP*(1/((1-c_disc_2_obst_2)*(1-c_disc_2_obst_2)+.001))+
			wP*(1/((1-c_disc_3_obst_2)*(1-c_disc_3_obst_2)+.001))+
			exp((error_contour-road_offset_right+exp_addition_right-right_offset)*steepness_right)+
			exp(-(error_contour-road_offset_left-exp_addition+left_offset)*steepness)
	);//+0*exp((-road_boundary+1)/0.1)+dot(a)*dot(a) // weight this with the physical cost!!!

	ocp.setModel(f);


	// DEFINE AN MPC EXPORT MODULE AND GENERATE THE CODE:
	// ----------------------------------------------------------
	OCPexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION,       EXACT_HESSIAN  		);
	mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING 	);
	mpc.set( INTEGRATOR_TYPE,             INT_RK4				);
	mpc.set( NUM_INTEGRATOR_STEPS,        15            		);
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
