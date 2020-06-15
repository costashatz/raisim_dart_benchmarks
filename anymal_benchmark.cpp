// DART related
#include <dart/config.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/SkelParser.hpp>
#include <dart/utils/sdf/SdfParser.hpp>
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

    double Kp = 300.;
    double Kd = 50.;

    std::string model_file = "/home/kchatzil/Workspaces/git/raisim/benchmarks/robots/anymal_fixed.urdf";

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

            // create raisim objects
            auto anymal = world.addArticulatedSystem(model_file);

            Eigen::VectorXd target_pos(19);
            target_pos << 0, 0, 0, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
            Eigen::VectorXd target_vel = Eigen::VectorXd::Zero(18);

            // set anymal properties
            anymal->setGeneralizedCoordinate(target_pos);
            anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));
            anymal->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
            anymal->setName("anymal");

            // Main control loop
            size_t loopN = sim_time / dt;
            auto start = std::chrono::steady_clock::now();
            for (size_t i = 0; i < loopN; i++) {
                Eigen::VectorXd pos = anymal->getGeneralizedCoordinate().e().tail(12);
                Eigen::VectorXd vel = anymal->getGeneralizedVelocity().e().tail(12);

                Eigen::VectorXd command = Kp * (target_pos.tail(12) - pos) - Kd * vel;
                Eigen::VectorXd cmd(command.size() + 6);
                cmd.setZero();
                cmd.tail(12) = command;
                anymal->setGeneralizedForce(cmd);
                world.integrate();
            }
            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            raisim_time += elapsed_seconds.count();
        }
        raisim_time /= repeats;
        std::cout << "RaiSim time: " << raisim_time << "s\n";
    }

    //// DART
    {
        for (size_t repeat = 0; repeat < repeats; repeat++) {
            // Create DART world
            auto world = std::make_shared<dart::simulation::World>();

            // set time step
            world->setTimeStep(dt);

            // load URDF
            dart::utils::DartLoader loader;
            auto anymal = loader.parseSkeleton(model_file);

            // dart::dynamics::WeldJoint::Properties properties;
            // anymal->getRootBodyNode()->changeParentJointType<dart::dynamics::WeldJoint>(properties);

            // Enforce limits and add anymal
            for (size_t i = 0; i < anymal->getNumDofs(); ++i) {
#if DART_VERSION_AT_LEAST(6, 10, 0)
                anymal->getDof(i)->getJoint()->setLimitEnforcement(true);
#else
                anymal->getDof(i)->getJoint()->setPositionLimitEnforced(true);
#endif
            }

            world->addSkeleton(anymal);

            // Main control loop
            Eigen::VectorXd target_pos(12);
            target_pos << 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
            std::vector<std::string> joint_names = {"LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE", "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
            std::vector<int> indices;

            // Get indices and set init position
            for (size_t i = 0; i < joint_names.size(); i++) {
                indices.push_back(anymal->getJoint(joint_names[i])->getDof(0)->getIndexInSkeleton());
                auto dof = anymal->getDof(indices[i]);
                dof->setPosition(target_pos(i));
            }

            size_t loopN = sim_time / dt;
            auto start = std::chrono::steady_clock::now();
            for (size_t i = 0; i < loopN; i++) {
                Eigen::VectorXd positions = anymal->getPositions();
                Eigen::VectorXd velocities = anymal->getVelocities();
                for (size_t j = 0; j < 12; j++) {
                    auto dof = anymal->getDof(indices[j]); //anymal->getJoint(joint_names[j])->getDof(0);
                    dof->setCommand(Kp * (target_pos(j) - positions[indices[j]]) - Kd * velocities[indices[j]]);
                }
                world->step();
            }
            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            dart_time += elapsed_seconds.count();
        }
        dart_time /= repeats;
        std::cout << "DART time: " << dart_time << "s\n";
    }

    std::cout << "Ratio (DART/RaiSim): " << (dart_time / raisim_time) << std::endl;

    return 0;
}