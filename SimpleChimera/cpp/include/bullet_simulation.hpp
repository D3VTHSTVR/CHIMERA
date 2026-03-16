#ifndef SIMPLECHIMERA_BULLET_SIMULATION_HPP
#define SIMPLECHIMERA_BULLET_SIMULATION_HPP

#include <memory>
#include <string>
#include <vector>

struct btDiscreteDynamicsWorld;
struct btDefaultCollisionConfiguration;
struct btCollisionDispatcher;
struct btBroadphaseInterface;
struct btSequentialImpulseConstraintSolver;
struct btRigidBody;
struct btHingeConstraint;
struct btCollisionShape;

namespace simplechimera {

/**
 * Bullet physics world + 6-legged insect robot (from six_leg_insect.urdf data).
 * Exposes: reset(), step(dt), setJointTorques(18), getState(40), and optional GUI.
 */
class BulletSimulation {
public:
    static constexpr int N_JOINTS = 18;
    static constexpr int STATE_DIM = 40;

    BulletSimulation(
        double timeStep = 0.002,
        double gravityScale = 1.0,
        double groundFriction = 3.2,
        double contactStiffness = 400,
        double contactDamping = 250,
        int solverIterations = 100
    );
    ~BulletSimulation();

    void reset();
    void step(double dt);
    /** Set joint torques (N·m); order L1_coxa,femur,tibia, L2_..., L3_..., R1_..., R2_..., R3_.... */
    void setJointTorques(const double* torques);
    /** Fill state[40]: roll, pitch, roll_dot, pitch_dot, joint_pos[18], joint_vel[18]. */
    void getState(double* state) const;
    /** True if any tibia (foot) is in contact with the ground. */
    bool limbTipsInContact() const;
    void getBasePosition(double* xyz) const;
    void getBaseLinearVelocity(double* vxyz) const;

    double getTimeStep() const { return timeStep_; }
    int getStepCount() const { return stepCount_; }

    /** For GUI: access the dynamics world to iterate and draw rigid bodies. */
    btDiscreteDynamicsWorld* getWorld() const { return world_.get(); }

private:
    void createWorld();
    void createGround();
    void createRobot();
    void setRobotPose(double baseZ);

    double timeStep_;
    double gravityScale_;
    double groundFriction_;
    double contactStiffness_;
    double contactDamping_;
    int solverIterations_;
    int stepCount_;

    std::unique_ptr<btDefaultCollisionConfiguration> collisionConfig_;
    std::unique_ptr<btCollisionDispatcher> dispatcher_;
    std::unique_ptr<btBroadphaseInterface> broadphase_;
    std::unique_ptr<btSequentialImpulseConstraintSolver> solver_;
    std::unique_ptr<btDiscreteDynamicsWorld> world_;

    std::vector<std::unique_ptr<btCollisionShape>> shapes_;
    std::unique_ptr<btRigidBody> ground_;
    std::unique_ptr<btRigidBody> base_;
    std::vector<std::unique_ptr<btRigidBody>> links_;
    std::vector<std::unique_ptr<btHingeConstraint>> hinges_;
};

}  // namespace simplechimera

#endif
