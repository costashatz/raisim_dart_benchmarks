// DART related
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
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

int main()
{
    ///// Generic params
    double sim_time = 10.;
    double dt = 0.001;

    double Kp = 400.;
    double Kd = 1.;

    std::string model_file = "/home/kchatzil/Workspaces/git/raisim/benchmarks/robots/anymal.urdf";

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

            std::vector<raisim::ArticulatedSystem*> anymals;
            std::vector<Eigen::VectorXd> target_positions;
            for (size_t r = 0; r < robot_grid_size; r++) {
                for (size_t c = 0; c < robot_grid_size; c++) {
                    auto anymal = world.addArticulatedSystem(model_file);

                    Eigen::VectorXd target_pos(19);
                    target_pos << r * 2., c * 2., 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;

                    // set anymal properties
                    anymal->setGeneralizedCoordinate(target_pos);
                    anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));
                    anymal->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
                    anymal->setName("anymal");

                    anymals.push_back(anymal);
                    target_positions.push_back(target_pos);
                }
            }

            // Main control loop
            size_t loopN = sim_time / dt;
            auto start = std::chrono::steady_clock::now();
            for (size_t i = 0; i < loopN; i++) {
                for (size_t r = 0; r < anymals.size(); r++) {
                    auto anymal = anymals[r];

                    Eigen::VectorXd pos = anymal->getGeneralizedCoordinate().e().tail(12);
                    Eigen::VectorXd vel = anymal->getGeneralizedVelocity().e().tail(12);

                    Eigen::VectorXd command = Kp * (target_positions[r].tail(12) - pos) - Kd * vel;
                    Eigen::VectorXd cmd(command.size() + 6);
                    cmd.setZero();
                    cmd.tail(12) = command;
                    anymal->setGeneralizedForce(cmd);
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

            // change collision detector to Bullet (produces less contact points, closer to the collision detector of RaiSim)
            world->getConstraintSolver()->setCollisionDetector(dart::collision::BulletCollisionDetector::create());

            // load URDF
            dart::utils::DartLoader loader;
            auto root_anymal = loader.parseSkeleton(model_file);

            // Enforce limits
            for (size_t i = 0; i < root_anymal->getNumDofs(); ++i) {
#if DART_VERSION_AT_LEAST(6, 10, 0)
                root_anymal->getDof(i)->getJoint()->setLimitEnforcement(true);
#else
                root_anymal->getDof(i)->getJoint()->setPositionLimitEnforced(true);
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
            Eigen::VectorXd target_pos(12);
            target_pos << 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
            std::vector<std::string> joint_names = {"LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE", "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
            std::vector<int> indices;

            // Get indices and set init position
            for (size_t i = 0; i < joint_names.size(); i++) {
                indices.push_back(root_anymal->getJoint(joint_names[i])->getDof(0)->getIndexInSkeleton());
            }

            // add anymals
            std::vector<dart::dynamics::SkeletonPtr> anymals;
            std::vector<Eigen::VectorXd> target_positions;
            size_t id = 0;
            for (size_t r = 0; r < robot_grid_size; r++) {
                for (size_t c = 0; c < robot_grid_size; c++) {
#if DART_VERSION_AT_LEAST(6, 7, 2)
                    auto anymal = root_anymal->cloneSkeleton();
#else
                    auto anymal = root_anymal->clone();
#endif

                    anymal->setPosition(3, r * 2.);
                    anymal->setPosition(4, c * 2.);
                    anymal->setPosition(5, 0.54);
                    anymal->setName("anymal_" + std::to_string(id++));

                    for (size_t i = 0; i < joint_names.size(); i++) {
                        auto dof = anymal->getDof(indices[i]);
                        dof->setPosition(target_pos(i));
                    }

                    anymals.push_back(anymal);
                    target_positions.push_back(target_pos);

                    // Add animal to the world
                    world->addSkeleton(anymal);
                }
            }

            size_t loopN = sim_time / dt;
            auto start = std::chrono::steady_clock::now();
            for (size_t i = 0; i < loopN; i++) {
                for (size_t r = 0; r < anymals.size(); r++) {
                    auto anymal = anymals[r];

                    Eigen::VectorXd positions = anymal->getPositions();
                    Eigen::VectorXd velocities = anymal->getVelocities();

                    for (size_t j = 0; j < 12; j++) {
                        auto dof = anymal->getDof(indices[j]);
                        dof->setCommand(Kp * (target_pos(j) - positions[indices[j]]) - Kd * velocities[indices[j]]);
                    }
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