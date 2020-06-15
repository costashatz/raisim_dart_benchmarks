// DART related
#include <dart/config.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/urdf/urdf.hpp>

// RaiSim related
#include <raisim/World.hpp>

// std
#include <chrono>
#include <iostream>

int main()
{
    ///// Generic params
    double sim_time = 10.;
    double dt = 0.001;

    double Kp = 400.;
    double Kd = 1.;

    std::string model_file = "/home/kchatzil/Workspaces/git/raisim/benchmarks/robots/iiwa14.urdf";

    ///// Benchmark-related
    double raisim_time = 0.;
    double dart_time = 0.;

    size_t repeats = 10;

    //// RaiSim
    {
        for (size_t repeat = 0; repeat < repeats; repeat++) {
            // create raisim world
            raisim::World world;

            // set time step
            world.setTimeStep(dt);

            // set gravity
            world.setGravity({0., 0., -9.81});

            // create raisim objects
            auto iiwa = world.addArticulatedSystem(model_file);

            Eigen::VectorXd target_pos(7);
            target_pos << 1., 2., 0., -1., -1.5, 0.6, 0.2;

            // set iiwa properties
            iiwa->setGeneralizedCoordinate(Eigen::VectorXd::Zero(7));
            iiwa->setGeneralizedForce(Eigen::VectorXd::Zero(iiwa->getDOF()));
            iiwa->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
            iiwa->setName("iiwa");

            // Main control loop
            size_t loopN = sim_time / dt;
            auto start = std::chrono::steady_clock::now();
            for (size_t i = 0; i < loopN; i++) {
                Eigen::VectorXd pos = iiwa->getGeneralizedCoordinate().e();
                Eigen::VectorXd vel = iiwa->getGeneralizedVelocity().e();

                Eigen::VectorXd command = Kp * (target_pos - pos) - Kd * vel;
                iiwa->setGeneralizedForce(command);
                world.integrate();
            }
            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            raisim_time += elapsed_seconds.count();
        }
        raisim_time /= repeats;
        std::cout << "RaiSim time: " << raisim_time << "s" << std::endl;
        std::cout << "   real-time factor: " << (sim_time / raisim_time) << std::endl;
    }

    //// DART
    {
        for (size_t repeat = 0; repeat < repeats; repeat++) {
            // Create DART world
            auto world = std::make_shared<dart::simulation::World>();

            // set time step
            world->setTimeStep(dt);

            // set gravity
            world->setGravity(Eigen::Vector3d(0., 0., -9.81));

            // load URDF
            dart::utils::DartLoader loader;
            auto iiwa = loader.parseSkeleton(model_file);

            // dart::dynamics::WeldJoint::Properties properties;
            // iiwa->getRootBodyNode()->changeParentJointType<dart::dynamics::WeldJoint>(properties);

            // Enforce limits and add iiwa
            for (size_t i = 0; i < iiwa->getNumDofs(); ++i) {
#if DART_VERSION_AT_LEAST(6, 10, 0)
                iiwa->getDof(i)->getJoint()->setLimitEnforcement(true);
#else
                iiwa->getDof(i)->getJoint()->setPositionLimitEnforced(true);
#endif
            }

            world->addSkeleton(iiwa);

            // Main control loop
            Eigen::VectorXd target_pos(7);
            target_pos << 1., 2., 0., -1., -1.5, 0.6, 0.2;

            size_t loopN = sim_time / dt;
            auto start = std::chrono::steady_clock::now();
            for (size_t i = 0; i < loopN; i++) {
                Eigen::VectorXd positions = iiwa->getPositions();
                Eigen::VectorXd velocities = iiwa->getVelocities();

                Eigen::VectorXd commands = Kp * (target_pos - positions) - Kd * velocities;
                iiwa->setCommands(commands);
                world->step();
            }
            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            dart_time += elapsed_seconds.count();
        }
        dart_time /= repeats;
        std::cout << "DART time: " << dart_time << "s" << std::endl;
        std::cout << "   real-time factor: " << (sim_time / dart_time) << std::endl;
    }

    std::cout << "Ratio (DART/RaiSim): " << (dart_time / raisim_time) << std::endl;

    return 0;
}