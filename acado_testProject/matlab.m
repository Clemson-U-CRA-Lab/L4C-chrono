clc;
clear all;
close all;

EXPORT = 1;

DifferentialState x y phi theta v
% AlgebraicState delta_d THWi delta_v ttci
OnlineData xo1 yo1 xo2 yo2 
Control d_theta ax
Control s1 s2

%% Differential Equation

L = 2.75; % Wheelbase
m = 1814; % CX-7 curb weight

f = acado.DifferentialEquation();
f.ODE(dot(x) == v * cos(phi));
f.ODE(dot(y) == v * sin(phi));
f.ODE(dot(phi) == v * tan(theta)/L);
f.ODE(dot(theta) == d_theta);
f.ODE(dot(v) == ax);

ay = v ^ 2 / L * tan(theta);

uo1 = exp(-((x - xo1)^2/(2 * 100) + (y - yo1)^2/(2 * 1)));
uo2 = exp(-((x - xo2)^2/(2 * 100) + (y - yo2)^2/(2 * 1)));

h = [x, y, phi,  theta, v, uo1, uo2, d_theta, ax, s1, s2];
hN = [x, y, phi, theta, v, uo1, uo2,];

n_XD = length(hN);
n_U = length(controls);


%% MPCexport
acadoSet('problemname', 'mpc');

Ts = 0.5;
N = 10;
ocp = acado.OCP( 0.0, N*Ts, N );

W_mat = eye(n_XD+n_U,n_XD+n_U);
WN_mat = eye(n_XD,n_XD);
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);

ocp.minimizeLSQ( W, h );
ocp.minimizeLSQEndTerm( WN, hN );



% Mazda CX-7 steering ratio 15.8
% Steering wheel lock to lock 2.9 turns
% Max steering angle 33 degrees (0.575 rad)
ocp.subjectTo( 0 <= v );  % Constraints
ocp.subjectTo(-5 <= ay + s1);
ocp.subjectTo(ay - s1 <= 5);
ocp.subjectTo( -5 <= ax + s1 );  % Constraints
ocp.subjectTo( ax - s1 <= 5 ); 

ocp.subjectTo( -0.570 <= theta + s2);  % Constraints
ocp.subjectTo(theta - s2 <= 0.570 );  % Constraints
ocp.subjectTo(-0.2 <= d_theta );      % Constraints
ocp.subjectTo(d_theta <= 0.2 );      % Constraints
ocp.subjectTo(s1 >= 0 );      % Constraints
ocp.subjectTo(s2 >= 0 );      % Constraints
ocp.subjectTo( f );

mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',        10*N                 );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
mpc.set( 'HOTSTART_QP',                 'NO'             	);
mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-10				);
mpc.set( 'GENERATE_SIMULINK_INTERFACE', 'YES'               );

if EXPORT
    mpc.exportCode( 'Mazda_LaneChange_MPC' );
%     copyfile('/home/cralab/Downloads/ACADOtoolkit/external_packages/qpoases', 'Mazda_LaneChange_MPC/qpoases', 'f')
    
    cd Mazda_LaneChange_MPC
    make_acado_solver('../Mazda_LaneChange_MPC')
%     make_acado_solver_sfunction
%     copyfile('acado_solver_sfun.mex*', '../')
    cd ..
end
