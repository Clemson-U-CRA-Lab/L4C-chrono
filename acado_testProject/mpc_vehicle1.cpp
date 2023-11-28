
 /**
  *    \author Prakhar Gupta
  *    \date   2023
  */

#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>

int main( ){

    USING_NAMESPACE_ACADO

    // INTRODUCE THE VARIABLES:
    // -------------------------
    int EXPORT = 1;

    const double t_start =  0.0;
    const double t_end   = 10.0;

    const double L = 2.75; // Wheelbase
    const double m = 1814; // CX-7 curb weight

    const double Ts = 0.5;
    const double N = 10;

    DifferentialState x, y, phi, theta, v;
    // AlgebraicState delta_d THWi delta_v ttci
    OnlineData xo1, yo1, xo2, yo2; 
    Control d_theta, ax;
    Control s1, s2;
    DifferentialEquation f;
    IntermediateState ay, uo1, uo2; 

    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    f << dot(x) == v*cos(phi);
    f << dot(y) == v*sin(phi);
    f << dot(phi) == v*tan(theta)/L;
    f << dot(theta) == d_theta;
    f << dot(v) == ax;

    ay = v*v / L * tan(theta);
    uo1 = (x - xo1)*(x - xo1) + (y - yo1)*(y - yo1);
    uo2 = (x - xo1)*(x - xo1) + (y - yo2)*(y - yo2);
    // double uo2 = exp(-((x - xo2)**2/(2 * 100) + (y - yo2)**2/(2 * 1)));

    // Define LSQ function
    Function h, hN;
    h << x - 1.0;
    h << y - 1.0;
    h << phi;
    h << theta;
    h << v;
    h << uo1;
    h << uo2;
    h << d_theta;
    h << ax;
    h << s1;
    h << s2;

    hN << x;
    hN << y;
    hN << phi;
    hN << theta;
    hN << v;
    hN << uo1;
    hN << uo2;

    // Define weights
    DMatrix W = eye<double>( h.getDim() );
	DMatrix WN = eye<double>( hN.getDim() ) * 10;


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------

    OCP ocp( 0.0, N*Ts, N );

    ocp.minimizeLSQ( W, h );
    ocp.minimizeLSQEndTerm( WN, hN );

    ocp.subjectTo( f );

    ocp.subjectTo( 0 <= v );
    ocp.subjectTo(-5 <= ay + s1);
    ocp.subjectTo(ay - s1 <= 5);
    ocp.subjectTo( -5 <= ax + s1 ); 
    ocp.subjectTo( ax - s1 <= 5 ); 

    ocp.subjectTo( -0.570 <= theta + s2);
    ocp.subjectTo(theta - s2 <= 0.570 );
    ocp.subjectTo(-0.2 <= d_theta );
    ocp.subjectTo(d_theta <= 0.2 );     
    ocp.subjectTo(s1 >= 0.01 );  
    // ocp.subjectTo(s2 >= 0 );      


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    // OptimizationAlgorithm algorithm(ocp);
    // RealTimeAlgorithm algorithm(ocp,1/20.0);
    OCPexport algorithm (ocp);
    algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    // algorithm.set( MAX_NUM_ITERATIONS, 20 );
    // algorithm.set( KKT_TOLERANCE, 1e-10 );
    algorithm.set( DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING );
    
    algorithm.set( INTEGRATOR_TYPE,       INT_RK4       );
    // algorithm.set( NUM_INTEGRATOR_STEPS,        10*N         );

    algorithm.set( SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);
    algorithm.set( QP_SOLVER, QP_QPOASES );   

    // algorithm.set( HOTSTART_QP,            NO             	);
    algorithm.set( LEVENBERG_MARQUARDT,    1e-10    		);
    algorithm.set(GENERATE_TEST_FILE, YES);
	algorithm.set(GENERATE_MAKE_FILE, YES);
	algorithm.set(GENERATE_MATLAB_INTERFACE, YES);
    algorithm.set(GENERATE_SIMULINK_INTERFACE, YES);

	if (algorithm.exportCode( "mpc_export" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	algorithm.printDimensionsQP( );

	return EXIT_SUCCESS;

}


