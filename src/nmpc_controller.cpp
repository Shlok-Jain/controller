#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

USING_NAMESPACE_ACADO

int main() {
    // Define system states (positions, velocities)
    DifferentialState x;       // X position
    DifferentialState y;       // Y position
    DifferentialState theta;   // Orientation angle
    DifferentialState vx;      // Linear velocity in x-direction
    DifferentialState vy;      // Linear velocity in y-direction
    DifferentialState omega;   // Angular velocity

    // Control inputs (wheel velocities)
    Control u1, u2, u3, u4;

    // Robot parameters (wheel radius and distance to center)
    const double R = 0.05;
    const double d = 0.1;

    // Differential equations describing the robot's motion
    DifferentialEquation f;
    f << dot(x) == vx;
    f << dot(y) == vy;
    f << dot(theta) == omega;
    f << dot(vx) == (R / 4.0) * (u1 + u2 + u3 + u4);
    f << dot(vy) == (R / 4.0) * (-u1 + u2 + u3 - u4);
    f << dot(omega) == (R / (4.0 * d)) * (-u1 + u2 - u3 + u4);

    // Define the reference trajectory
    double x_ref = 5.0, y_ref = 5.0, theta_ref = 0.0;

    // Define the output function to track the reference trajectory
    Function h;
    h << x << y << theta << vx << vy << omega;

    // Define cost matrix for least-squares optimization
    DMatrix Q(6,6);
    Q.setIdentity();
    Q(0,0) = 100.0;  // High weight for x position
    Q(1,1) = 100.0;  // High weight for y position
    Q(2,2) = 10.0;   // Lower weight for orientation
    Q(3,3) = 1.0;    // Lower weight for velocities
    Q(4,4) = 1.0;
    Q(5,5) = 1.0;

    // Reference state vector
    DVector r(6);
    r(0) = x_ref;
    r(1) = y_ref;
    r(2) = theta_ref;
    r(3) = 0.0;
    r(4) = 0.0;
    r(5) = 0.0;

    // Define the OCP (time horizon and steps)
    const double t_start = 0.0;
    const double t_end = 10.0;
    OCP ocp(t_start, t_end, 20);

    // Minimize least squares error and apply dynamics
    ocp.minimizeLSQ(Q, h, r);
    ocp.subjectTo(f);
    ocp.subjectTo(-5.0 <= u1 <= 5.0);
    ocp.subjectTo(-5.0 <= u2 <= 5.0);
    ocp.subjectTo(-5.0 <= u3 <= 5.0);
    ocp.subjectTo(-5.0 <= u4 <= 5.0);

    // Real-time algorithm for MPC
    RealTimeAlgorithm alg(ocp, 0.1);
    alg.set(MAX_NUM_ITERATIONS, 2);

    // ** Define the Process: Wrap the system dynamics (f) into a Process **
    Process process(f);

    // Simulation setup
    StaticReferenceTrajectory zeroReference;
    Controller controller(alg, zeroReference);

    // ** Update SimulationEnvironment: Use the process and controller **
    SimulationEnvironment sim(0.0, 10.0, process, controller);

    // Initial state
    DVector x0(6);
    x0(0) = 0.0; x0(1) = 0.0; x0(2) = 0.0;
    x0(3) = 0.0; x0(4) = 0.0; x0(5) = 0.0;

    sim.init(x0);
    sim.run();

    // Plot the results
    VariablesGrid output;
    sim.getSampledProcessOutput(output);
    GnuplotWindow window;
    window.addSubplot(output(0), "X Position");
    window.addSubplot(output(1), "Y Position");
    window.addSubplot(output(2), "Orientation");
    window.addSubplot(output(3), "X Velocity");
    window.addSubplot(output(4), "Y Velocity");
    window.addSubplot(output(5), "Angular Velocity");
    window.plot();

    return 0;
}

