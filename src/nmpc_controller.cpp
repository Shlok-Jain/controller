#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <iostream>  // For printing the values

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

    // Define the Process: Wrap the system dynamics (f) into a Process
    Process process(f);

    // Simulation setup
    StaticReferenceTrajectory zeroReference;
    Controller controller(alg, zeroReference);
    SimulationEnvironment sim(0.0, 10.0, process, controller);

    // Initial state
    DVector x0(6);
    x0(0) = 0.0; x0(1) = 0.0; x0(2) = 0.0;
    x0(3) = 0.0; x0(4) = 0.0; x0(5) = 0.0;

    sim.init(x0);
    
    // Run the simulation
    if (sim.run() == SUCCESSFUL_RETURN) {
        // Store the output data
        VariablesGrid output;
        VariablesGrid control;
        
        // Get the sampled process output (state variables)
        sim.getSampledProcessOutput(output);

        // Get the feedback control (wheel velocities)
        sim.getFeedbackControl(control);

        // Print output values at every 5th iteration
        for (int i = 0; i < output.getNumPoints(); ++i) {
            if (i % 5 == 0) {  // Print every 5th iteration
                double time = output.getTime(i);
                DVector state = output.getVector(i);
                DVector ctrl = control.getVector(i);

                std::cout << "Iteration: " << i << " Time: " << time << "s\n";
                std::cout << "State Values:\n";
                std::cout << "  X Position: " << state(0) << "\n";
                std::cout << "  Y Position: " << state(1) << "\n";
                std::cout << "  Orientation (Theta): " << state(2) << "\n";
                std::cout << "  X Velocity: " << state(3) << "\n";
                std::cout << "  Y Velocity: " << state(4) << "\n";
                std::cout << "  Angular Velocity: " << state(5) << "\n";

                std::cout << "Control Inputs (Wheel Velocities):\n";
                std::cout << "  u1: " << ctrl(0) << "\n";
                std::cout << "  u2: " << ctrl(1) << "\n";
                std::cout << "  u3: " << ctrl(2) << "\n";
                std::cout << "  u4: " << ctrl(3) << "\n";
                std::cout << "----------------------------------------\n";
            }
        }
    } else {
        std::cerr << "Simulation failed.\n";
    }

    return 0;
}

