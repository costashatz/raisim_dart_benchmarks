// DART related
#include <dart/config.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/urdf/urdf.hpp>

// RaiSim related
#include <raisim/World.hpp>

// std
#include <chrono>
#include <iostream>

#define MAKE_STR(x) _MAKE_STR(x)
#define _MAKE_STR(x) #x

int main()
{
    ///// Generic params
    double sim_time = 10.;
    double dt = 0.001;

    double Kp = 1.;
    double Kd = 0.1;

    std::string model_file = std::string(MAKE_STR(EXAMPLE_ROBOT_RESOURCE_DIR)) + "/pexod.urdf";

    ///// Benchmark-related
    double raisim_time = 0.;
    double dart_time = 0.;

    size_t repeats = 10;

    size_t robot_grid_size = 2; // 2x2 robots by default

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
            auto ground = world.addGround();
            ground->setName("checkerboard");

            std::vector<raisim::ArticulatedSystem*> pexods;
            std::vector<Eigen::VectorXd> target_positions;
            for (size_t r = 0; r < robot_grid_size; r++) {
                for (size_t c = 0; c < robot_grid_size; c++) {
                    auto pexod = world.addArticulatedSystem(model_file);

                    Eigen::VectorXd target_pos = Eigen::VectorXd::Zero(18 + 7);
                    target_pos.head(7) << r * 2., c * 2., 0.2, 1.0, 0.0, 0.0, 0.0;

                    // set pexod properties
                    pexod->setGeneralizedCoordinate(target_pos);
                    pexod->setGeneralizedForce(Eigen::VectorXd::Zero(pexod->getDOF()));
                    pexod->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
                    pexod->setName("pexod");

                    pexods.push_back(pexod);
                    target_positions.push_back(target_pos);
                }
            }

            // Main control loop
            size_t loopN = sim_time / dt;
            auto start = std::chrono::steady_clock::now();
            for (size_t i = 0; i < loopN; i++) {
                for (size_t r = 0; r < pexods.size(); r++) {
                    auto pexod = pexods[r];

                    Eigen::VectorXd pos = pexod->getGeneralizedCoordinate().e().tail(18);
                    Eigen::VectorXd vel = pexod->getGeneralizedVelocity().e().tail(18);

                    Eigen::VectorXd command = Kp * (target_positions[r].tail(18) - pos) - Kd * vel;
                    Eigen::VectorXd cmd(command.size() + 6);
                    cmd.setZero();
                    cmd.tail(18) = command;
                    pexod->setGeneralizedForce(cmd);
                }
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
            auto root_pexod = loader.parseSkeleton(model_file);

            // Enforce limits
            for (size_t i = 0; i < root_pexod->getNumDofs(); ++i) {
#if DART_VERSION_AT_LEAST(6, 10, 0)
                root_pexod->getDof(i)->getJoint()->setLimitEnforcement(true);
#else
                root_pexod->getDof(i)->getJoint()->setPositionLimitEnforced(true);
#endif
            }

            // Create and add floor
            double floor_width = 100.0;
            double floor_height = 0.1;
            Eigen::Vector6d pose = Eigen::Vector6d::Zero();

            dart::dynamics::SkeletonPtr floor_skel = dart::dynamics::Skeleton::create("floor");
            // Give the floor a body
            dart::dynamics::BodyNodePtr body = floor_skel->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
            // Give the body a shape
            auto box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(floor_width, floor_width, floor_height));
            body->createShapeNodeWith<dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);
            // Put the body into position
            Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
            // tf.translation() = Eigen::Vector3d(x, y, -floor_height / 2.0);
            tf.linear() = dart::math::eulerXYZToMatrix(pose.head(3));
            tf.translation() = pose.tail(3);
            tf.translation()[2] -= floor_height / 2.0;
            body->getParentJoint()->setTransformFromParentBodyNode(tf);

            world->addSkeleton(floor_skel);

            // Main control loop
            Eigen::VectorXd target_pos = Eigen::VectorXd::Zero(18);

            // add pexods
            std::vector<dart::dynamics::SkeletonPtr> pexods;
            std::vector<Eigen::VectorXd> target_positions;
            size_t id = 0;
            for (size_t r = 0; r < robot_grid_size; r++) {
                for (size_t c = 0; c < robot_grid_size; c++) {
#if DART_VERSION_AT_LEAST(6, 7, 2)
                    auto pexod = root_pexod->cloneSkeleton();
#else
                    auto pexod = root_pexod->clone();
#endif

                    pexod->setPosition(3, r * 2.);
                    pexod->setPosition(4, c * 2.);
                    pexod->setPosition(5, 0.2);
                    pexod->setName("pexod_" + std::to_string(id++));

                    pexods.push_back(pexod);
                    target_positions.push_back(target_pos);

                    // Add animal to the world
                    world->addSkeleton(pexod);
                }
            }

            size_t loopN = sim_time / dt;
            auto start = std::chrono::steady_clock::now();
            for (size_t i = 0; i < loopN; i++) {
                for (size_t r = 0; r < pexods.size(); r++) {
                    auto pexod = pexods[r];

                    Eigen::VectorXd positions = pexod->getPositions();
                    Eigen::VectorXd velocities = pexod->getVelocities();

                    Eigen::VectorXd cmd = Kp * (target_pos - positions.tail(18)) - Kd * velocities.tail(18);
                    Eigen::VectorXd commands = Eigen::VectorXd::Zero(pexod->getNumDofs());
                    commands.tail(18) = cmd;
                    pexod->setCommands(commands);
                }
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