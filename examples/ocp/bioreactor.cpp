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
    DifferentialState     x,y,theta;//,v;
    Control               v,w;
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

    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    
    f << dot(x) == v*cos(theta);
    f << dot(y) == v*sin(theta);
    f << dot(theta) == w;
    //f << dot(v) == a;
	//f << dot(dummy1) == sv1;
    //f << dot(dummy2) == sv2;

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( 0.0, 20, 100.0 );

    // Need to set the number of online variables!
    ocp.setNOD(35);

	ocp.minimizeLagrangeTerm(Wx*(x-x_goal)*(x-x_goal) +Wy*(y-y_goal)*(y-y_goal)+ Wtheta*(theta-theta_goal)*(theta-theta_goal)); //
    ocp.minimizeMayerTerm(Wx*(x-x_goal)*(x-x_goal) +Wy*(y-y_goal)*(y-y_goal)+ Wtheta*(theta-theta_goal)*(theta-theta_goal)); //

    ocp.setModel(f);

    ocp.subjectTo( 0 <= v <= 0.2 );
    ocp.subjectTo( -0.5 <= w <= 0.5 );

    //ocp.subjectTo( AT_END, x-x_goal  == 0.0 );
    //ocp.subjectTo( AT_END, y-y_goal == 0.0 );
    //ocp.subjectTo( AT_END, theta-theta_goal == 0.0 );


    // DEFINE AN MPC EXPORT MODULE AND GENERATE THE CODE:
	// ----------------------------------------------------------
/*
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
*/
    ocp.subjectTo( AT_START, x  == 0.9 );
    ocp.subjectTo( AT_START, y  == 0.3 );
    ocp.subjectTo( AT_START, theta  == -1.57 );
    //ocp.subjectTo( AT_START, v  == 0.0 );
    //ocp.subjectTo( AT_START, w == 0.0 );
    double wx = 100;
    double wy = 100;
    double wtheta = 100;
    double xgoal = 0.5;
    double ygoal = 0.5;
    double thetagoal = 0;
    ocp.minimizeLagrangeTerm(v*v+w*w);//wx*(x-xgoal)*(x-xgoal) +wy*(y-ygoal)*(y-ygoal)+ wtheta*(theta-thetagoal)*(theta-thetagoal)); //
    ocp.minimizeMayerTerm(wx*(x-xgoal)*(x-xgoal) +wy*(y-ygoal)*(y-ygoal)+ wtheta*(theta-thetagoal)*(theta-thetagoal)); //
    // Additionally, flush a plotting object

    GnuplotWindow window1;//( PLOT_AT_EACH_ITERATION );
    window1.addSubplot( x,y, " X [m]" );
    window1.addSubplot( y," Y [m]" );
    window1.addSubplot( theta," Theta [m]" );
    window1.addSubplot( v, "Velocity [m/s]" );
    //window1.addSubplot( a, "Velocity [m/s^2]" );
    window1.addSubplot( w,"Angular Velocity [m/s]" );
    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // --------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);

    algorithm << window1;

    // algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
    //  algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    //algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
    algorithm.set( DISCRETIZATION_TYPE,              MULTIPLE_SHOOTING 	);
   // algorithm.set( INTEGRATOR_TYPE,                  INT_RK4			    );
    //algorithm.set( NUM_INTEGRATOR_STEPS,             25            		);
    algorithm.set( QP_SOLVER,                        QP_QPOASES3    		);
    //algorithm.set( HOTSTART_QP,                      NO             		);
    //algorithm.set( SPARSE_QP_SOLUTION, 		       FULL_CONDENSING_N2	);
    //algorithm.set( INTEGRATOR_TOLERANCE, 1e-8 );
    algorithm.set( KKT_TOLERANCE, 1e-9 );
    //algorithm.set( GLOBALIZATION_STRATEGY, GS_FULLSTEP );
    //algorithm.set( MAX_NUM_ITERATIONS, 1 );

    algorithm.solve();

	// ----------------------------------------------------------


    return 0;
}
