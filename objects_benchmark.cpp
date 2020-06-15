// DART related
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/config.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/urdf/urdf.hpp>

// RaiSim related
#include <raisim/World.hpp>

// std
#include <chrono>
#include <iostream>

dart::dynamics::SkeletonPtr dart_create_box(const Eigen::Vector3d& dims, double mass, const std::string& box_name)
{
    dart::dynamics::SkeletonPtr box_skel = dart::dynamics::Skeleton::create(box_name);

    // Give the box a body
    dart::dynamics::BodyNodePtr body;
    body = box_skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(nullptr).second;
    body->setName(box_name);

    // Give the body a shape
    auto box = std::make_shared<dart::dynamics::BoxShape>(dims);
    auto box_node = body->createShapeNodeWith<dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);
    // Set up inertia
    dart::dynamics::Inertia inertia;
    inertia.setMass(mass);
    inertia.setMoment(box->computeInertia(mass));
    body->setInertia(inertia);

    return box_skel;
}

dart::dynamics::SkeletonPtr dart_create_ellipsoid(const Eigen::Vector3d& dims, double mass, const std::string& ellipsoid_name)
{
    dart::dynamics::SkeletonPtr ellipsoid_skel = dart::dynamics::Skeleton::create(ellipsoid_name);

    // Give the ellipsoid a body
    dart::dynamics::BodyNodePtr body;
    body = ellipsoid_skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(nullptr).second;
    body->setName(ellipsoid_name);

    // Give the body a shape
    auto ellipsoid = std::make_shared<dart::dynamics::EllipsoidShape>(dims);
    auto ellipsoid_node = body->createShapeNodeWith<dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(ellipsoid);
    // Set up inertia
    dart::dynamics::Inertia inertia;
    inertia.setMass(mass);
    inertia.setMoment(ellipsoid->computeInertia(mass));
    body->setInertia(inertia);

    return ellipsoid_skel;
}

dart::dynamics::SkeletonPtr dart_create_floor(double floor_width, double floor_height, const std::string& floor_name)
{
    Eigen::Vector6d pose = Eigen::Vector6d::Zero();

    dart::dynamics::SkeletonPtr floor_skel = dart::dynamics::Skeleton::create(floor_name);
    // Give the floor a body
    dart::dynamics::BodyNodePtr body = floor_skel->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
    // Give the body a shape
    auto box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(floor_width, floor_width, floor_height));
    auto box_node = body->createShapeNodeWith<dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);
    // Put the body into position
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    // tf.translation() = Eigen::Vector3d(x, y, -floor_height / 2.0);
    tf.linear() = dart::math::eulerXYZToMatrix(pose.head(3));
    tf.translation() = pose.tail(3);
    tf.translation()[2] -= floor_height / 2.0;
    body->getParentJoint()->setTransformFromParentBodyNode(tf);

    return floor_skel;
}

int main()
{
    ///// Generic params
    double sim_time = 10.;
    double dt = 0.001;

    ///// Benchmark-related
    double raisim_time = 0.;
    double dart_time = 0.;

    size_t repeats = 10;
    size_t sphere_grid_size = 5;
    size_t box_grid_size = 5;

    double init_height = 0.5;

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

            for (size_t r = 0; r < sphere_grid_size; r++) {
                for (size_t c = 0; c < sphere_grid_size; c++) {
                    auto sphere = world.addSphere(0.1, 1.);

                    sphere->setPosition(r * 0.3, c * 0.3, init_height);

                    sphere = world.addSphere(0.1, 1.);

                    sphere->setPosition(r * 0.3 + 0.05, c * 0.3 + 0.05, init_height + 0.3);
                }
            }

            for (size_t r = 0; r < box_grid_size; r++) {
                for (size_t c = 0; c < box_grid_size; c++) {
                    auto box = world.addBox(0.1, 0.1, 0.1, 1.);
                    box->setPosition(r * 0.3, c * 0.3, init_height + 0.6);
                }
            }

            // Main simulation
            size_t loopN = sim_time / dt;
            auto start = std::chrono::steady_clock::now();
            for (size_t i = 0; i < loopN; i++) {
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

            // create and add floor
            auto floor_skel = dart_create_floor(100., 0.1, "floor");
            world->addSkeleton(floor_skel);

            // create spheres/boxes
            size_t id = 0;
            for (size_t r = 0; r < sphere_grid_size; r++) {
                for (size_t c = 0; c < sphere_grid_size; c++) {
                    auto sphere = dart_create_ellipsoid({0.1, 0.1, 0.1}, 1., "sphere_" + std::to_string(id++));
                    sphere->setPosition(3, r * 0.3);
                    sphere->setPosition(4, c * 0.3);
                    sphere->setPosition(5, init_height);

                    world->addSkeleton(sphere);

                    sphere = dart_create_ellipsoid({0.1, 0.1, 0.1}, 1., "sphere_" + std::to_string(id++));
                    sphere->setPosition(3, r * 0.3 + 0.05);
                    sphere->setPosition(4, c * 0.3 + 0.05);
                    sphere->setPosition(5, init_height + 0.3);

                    world->addSkeleton(sphere);
                }
            }

            for (size_t r = 0; r < box_grid_size; r++) {
                for (size_t c = 0; c < box_grid_size; c++) {
                    auto box = dart_create_box({0.1, 0.1, 0.1}, 1., "box_" + std::to_string(id++));
                    box->setPosition(3, r * 0.3);
                    box->setPosition(4, c * 0.3);
                    box->setPosition(5, init_height + 0.6);

                    world->addSkeleton(box);
                }
            }

            // Main simulation
            size_t loopN = sim_time / dt;
            auto start = std::chrono::steady_clock::now();
            for (size_t i = 0; i < loopN; i++) {
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