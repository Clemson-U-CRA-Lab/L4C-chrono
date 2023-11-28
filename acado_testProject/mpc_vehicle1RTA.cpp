
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
    int EXPORT = 0;

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
    DifferentialEquation f(0.0, Ts*N);
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
    DMatrix W(11,11); // LSQ coefficient matrix
    W(1,1) = 10.0;
    W(2,2) = 10.0;
    W(3,3) = 2.0;
    W(4,4) = 1.0;
    W(5,5) = 2.0;
    W(6,6) = 1.0;
    W(7,7) = 1.0;
    W(8,8) = 0.0;
    W(9,9) = 0.0;
    W.setIdentity();
    DMatrix WN(7,7);
    // WN(1,1) = 1.0;
    // WN(2,2) = 1.0;
    // WN(3,3) = 1.0;
    // WN(4,4) = 1.0;
    // WN(5,5) = 1.0;
    // WN(6,6) = 1.0;
    // WN(7,7) = 1.0;
    WN.setIdentity();

    DVector r(11);
    r.setAll(0.0);
    r(0) = 0.0;
    // printf("Reference states: ---------------- \n");
    // r.print();

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------

    OCP ocp( 0.0, N*Ts, N );

    ocp.minimizeLSQ( W, h, r );
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
    ocp.subjectTo(s1 >= 0 );  
    ocp.subjectTo(s2 >= 0 );      
    // ocp.subjectTo( AT_START, x == 0.0 );
    // ocp.subjectTo( AT_END, x == 2.0 );

    // DEFINE A PLOT WINDOW:
    // ---------------------
    GnuplotWindow window;
        window.addSubplot( x,"DifferentialState x" );
        window.addSubplot( v,"DifferentialState v" );
        window.addSubplot( y,"DifferentialState y" );
        window.addSubplot( phi,"DifferentialState phi" );
        window.addSubplot( theta,"DifferentialState theta" );
        window.addSubplot( ax,"Control ax" );
        window.addSubplot( PLOT_KKT_TOLERANCE,"KKT Tolerance" );

    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    // OptimizationAlgorithm algorithm(ocp);
    RealTimeAlgorithm algorithm(ocp,1/20.0);
    algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    algorithm.set( MAX_NUM_ITERATIONS, 20 );
    algorithm.set( KKT_TOLERANCE, 1e-10 );
    algorithm.set( DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING );
    // only for export : algorithm.set( SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);
    algorithm.set( INTEGRATOR_TYPE,       INT_RK45       );
    //  only for export : algorithm.set( NUM_INTEGRATOR_STEPS,        10*N         );
    //  only for export : algorithm.set( QP_SOLVER, QP_QPOASES );   
    algorithm.set( HOTSTART_QP,            NO             	);
    algorithm.set( LEVENBERG_MARQUARDT,    1e-10    		);
    algorithm << window;

    printf("Set up - Ready to solve  \n");

    // Init the x0
    DVector x0(5);
    x0(0) = -1;
    x0(1) = 0;
    x0(2) = 0;
    x0(3) = 0;
    x0(4) = 0;


    // Define path to follow
    StaticReferenceTrajectory zeroReference( "ref1.txt" );
    // FILE* pFile = fopen("ref1.txt","r");
    // VariablesGrid refGrid(FILE * pFile);
    VariablesGrid refGrid( 5,0,30,4 );
    printf("VarGrid is: ----------------  \n");
    refGrid.init();
    double p0[5] = {0,0,0,0,0};
    double p1[5] = {30,0,0,5,0};
    double p2[5] = {55,3.7,0,5,0};
    double p3[5] = {90,3.7,0,5,0};
    DVector rv0(5, p0);
    DVector rv1(5, p1);
    DVector rv2(5, p2);
    DVector rv3(5, p3);
    refGrid.addVector(rv0, 0.0);
    refGrid.addVector(rv1, 3.0);
    refGrid.addVector(rv2, 5.0);
    refGrid.addVector(rv3, 10.0);
    // refGrid.setVector(2, p2);
    refGrid.print();
    refGrid.refineGrid(5);
    refGrid.print();

    // algorithm.setReference(zeroReference.getReference(2.0,4.0));
    algorithm.solve(0.0, x0);
    
    // VariablesGird xy = zeroReference.getReference(2.0,4.0, refGrid);

    VariablesGrid statesOut, controlsOut;
    algorithm.getDifferentialStates(statesOut);
    algorithm.getControls(controlsOut);

    // printf("States: ----------------  \n");
    // statesOut.print();
    // printf("Controls: ---------------- \n");
    // controlsOut.print();
    std::ofstream stream1( "out.txt" );
    statesOut.print(stream1, 0," "," ", 16, 16, " ", "\n" );
    controlsOut.print(stream1, 0," "," ", 16, 16, " ", "\n" );
    stream1.close();

    return 0;
}


