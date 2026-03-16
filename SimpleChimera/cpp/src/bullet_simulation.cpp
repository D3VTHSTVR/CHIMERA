#include "bullet_simulation.hpp"
#include <btBulletDynamicsCommon.h>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace simplechimera {

namespace {

// Link index: 0=base, 1=L1_coxa, 2=L1_femur, 3=L1_tibia, 4=L2_coxa, ... 18=R3_tibia
// Joint index: 0=L1_coxa (base->L1_coxa), 1=L1_femur, ... 17=R3_tibia
// Parent link index for each joint's child (-1 = base)
const int JOINT_PARENT[] = {-1, 1, 2, -1, 4, 5, -1, 7, 8, -1, 10, 11, -1, 13, 14, -1, 16, 17};
// Child link index for each joint
const int JOINT_CHILD[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};

// Pivot in parent frame (joint position); axis in parent frame (0,1,0 for all)
// From URDF: L1 coxa at (0.06, 0.055, -0.015), L2 at (0, 0.055, -0.015), L3 at (-0.06, 0.055, -0.015)
// R1 at (0.06, -0.055, -0.015), R2 at (0, -0.055, -0.015), R3 at (-0.06, -0.055, -0.015)
// Femur/tibia pivots: (0, 0.02, 0) then (0, 0, -0.035) for L; similar for R
void getJointPivotInParent(int j, btVector3& pivot) {
    if (j < 3) {  // L1
        if (j == 0) pivot = btVector3(0.06f, 0.055f, -0.015f);
        else if (j == 1) pivot = btVector3(0.f, 0.02f, 0.f);
        else pivot = btVector3(0.f, 0.f, -0.035f);
    } else if (j < 6) {  // L2
        if (j == 3) pivot = btVector3(0.f, 0.055f, -0.015f);
        else if (j == 4) pivot = btVector3(0.f, 0.02f, 0.f);
        else pivot = btVector3(0.f, 0.f, -0.035f);
    } else if (j < 9) {  // L3
        if (j == 6) pivot = btVector3(-0.06f, 0.055f, -0.015f);
        else if (j == 7) pivot = btVector3(0.f, 0.02f, 0.f);
        else pivot = btVector3(0.f, 0.f, -0.035f);
    } else if (j < 12) {  // R1
        if (j == 9) pivot = btVector3(0.06f, -0.055f, -0.015f);
        else if (j == 10) pivot = btVector3(0.f, 0.02f, 0.f);
        else pivot = btVector3(0.f, 0.f, -0.035f);
    } else if (j < 15) {  // R2
        if (j == 12) pivot = btVector3(0.f, -0.055f, -0.015f);
        else if (j == 13) pivot = btVector3(0.f, 0.02f, 0.f);
        else pivot = btVector3(0.f, 0.f, -0.035f);
    } else {  // R3
        if (j == 15) pivot = btVector3(-0.06f, -0.055f, -0.015f);
        else if (j == 16) pivot = btVector3(0.f, 0.02f, 0.f);
        else pivot = btVector3(0.f, 0.f, -0.035f);
    }
}

// Pivot in child frame (joint at child origin for coxa; for femur/tibia at (0,0,0) in child)
// Child COM offset from URDF: coxa (0,0.01,0), femur/tibia (0,0,-0.0175) so pivot in child = (0,0,0) for all
void getJointPivotInChild(int j, btVector3& pivot) {
    (void)j;
    pivot = btVector3(0.f, 0.f, 0.f);
}

// Axis is always (0,1,0) in both frames for our URDF
void getJointAxis(btVector3& axis) {
    axis = btVector3(0.f, 1.f, 0.f);
}

// Mass and inertia per link (0=base, 1..18 = links)
double getLinkMass(int linkIdx) {
    if (linkIdx == 0) return 0.2;
    int leg = linkIdx / 3, seg = linkIdx % 3;
    (void)leg;
    if (seg == 0) return 0.006;  // coxa
    return 0.008;  // femur, tibia
}

// Box half-extents (from URDF). Base 0.16x0.10x0.03 -> 0.08, 0.05, 0.015
void getLinkHalfExtents(int linkIdx, btVector3& halfExt) {
    if (linkIdx == 0) {
        halfExt = btVector3(0.08f, 0.05f, 0.015f);
        return;
    }
    int seg = linkIdx % 3;
    if (seg == 0) halfExt = btVector3(0.006f, 0.01f, 0.006f);   // coxa box 0.012 0.02 0.012
    else if (seg == 1) halfExt = btVector3(0.005f, 0.005f, 0.0175f);  // femur
    else halfExt = btVector3(0.004f, 0.004f, 0.0175f);  // tibia
}

btVector3 makeInertia(double mass, const btVector3& halfExt) {
    double lx = 2 * halfExt.x(), ly = 2 * halfExt.y(), lz = 2 * halfExt.z();
    double Ixx = mass * (ly*ly + lz*lz) / 12.0;
    double Iyy = mass * (lx*lx + lz*lz) / 12.0;
    double Izz = mass * (lx*lx + ly*ly) / 12.0;
    return btVector3(Ixx, Iyy, Izz);
}

}  // namespace

BulletSimulation::BulletSimulation(
    double timeStep, double gravityScale, double groundFriction,
    double contactStiffness, double contactDamping, int solverIterations
)
    : timeStep_(timeStep)
    , gravityScale_(gravityScale)
    , groundFriction_(groundFriction)
    , contactStiffness_(static_cast<btScalar>(contactStiffness))
    , contactDamping_(static_cast<btScalar>(contactDamping))
    , solverIterations_(solverIterations)
    , stepCount_(0)
{
    createWorld();
    createGround();
    createRobot();
    setRobotPose(0.102);
}

BulletSimulation::~BulletSimulation() {
    hinges_.clear();
    links_.clear();
    base_.reset();
    ground_.reset();
    shapes_.clear();
    world_.reset();
    solver_.reset();
    broadphase_.reset();
    dispatcher_.reset();
    collisionConfig_.reset();
}

void BulletSimulation::createWorld() {
    collisionConfig_ = std::make_unique<btDefaultCollisionConfiguration>();
    dispatcher_ = std::make_unique<btCollisionDispatcher>(collisionConfig_.get());
    broadphase_ = std::make_unique<btDbvtBroadphase>();
    solver_ = std::make_unique<btSequentialImpulseConstraintSolver>();
    world_ = std::make_unique<btDiscreteDynamicsWorld>(
        dispatcher_.get(), broadphase_.get(), solver_.get(), collisionConfig_.get());
    world_->setGravity(btVector3(0, 0, static_cast<btScalar>(-9.81 * gravityScale_)));
    world_->getSolverInfo().m_numIterations = solverIterations_;
}

void BulletSimulation::createGround() {
    btCollisionShape* groundShape = new btBoxShape(btVector3(50, 50, 0.01));
    shapes_.emplace_back(groundShape);
    btTransform start;
    start.setIdentity();
    start.setOrigin(btVector3(0, 0, 0));
    btScalar mass(0.);
    btVector3 localInertia(0, 0, 0);
    btRigidBody::btRigidBodyConstructionInfo ci(mass, nullptr, groundShape, localInertia);
    ground_ = std::make_unique<btRigidBody>(ci);
    ground_->setWorldTransform(start);
    ground_->setFriction(static_cast<btScalar>(groundFriction_));
    world_->addRigidBody(ground_.get());
}

void BulletSimulation::createRobot() {
    const double startZ = 0.102;
    btTransform baseTransform;
    baseTransform.setIdentity();
    baseTransform.setOrigin(btVector3(0, 0, static_cast<btScalar>(startZ)));

    btVector3 halfExt;
    getLinkHalfExtents(0, halfExt);
    btCollisionShape* baseShape = new btBoxShape(halfExt);
    shapes_.emplace_back(baseShape);
    double mass = getLinkMass(0);
    btVector3 inertia = makeInertia(mass, halfExt);
    btRigidBody::btRigidBodyConstructionInfo ci(
        static_cast<btScalar>(mass), nullptr, baseShape, btVector3(inertia.x(), inertia.y(), inertia.z()));
    base_ = std::make_unique<btRigidBody>(ci);
    base_->setWorldTransform(baseTransform);
    base_->setFriction(static_cast<btScalar>(groundFriction_));
    base_->setDamping(static_cast<btScalar>(3.0), static_cast<btScalar>(3.5));
    base_->setActivationState(DISABLE_DEACTIVATION);
    world_->addRigidBody(base_.get());

    // Initial link positions (zero joint angles): compute from kinematic chain
    btTransform linkWorldTrans[18];
    for (int j = 0; j < 18; ++j) {
        btVector3 pivotA;
        getJointPivotInParent(j, pivotA);
        btTransform parentTrans = (JOINT_PARENT[j] < 0) ? baseTransform : linkWorldTrans[JOINT_PARENT[j]];
        btVector3 pivotWorld = parentTrans.getOrigin() + parentTrans.getBasis() * pivotA;
        linkWorldTrans[JOINT_CHILD[j] - 1].setOrigin(pivotWorld);
        linkWorldTrans[JOINT_CHILD[j] - 1].setBasis(parentTrans.getBasis());
    }

    links_.resize(18);
    for (int i = 0; i < 18; ++i) {
        getLinkHalfExtents(i + 1, halfExt);
        btCollisionShape* shape = new btBoxShape(halfExt);
        shapes_.emplace_back(shape);
        mass = getLinkMass(i + 1);
        inertia = makeInertia(mass, halfExt);
        btRigidBody::btRigidBodyConstructionInfo linkCi(
            static_cast<btScalar>(mass), nullptr, shape,
            btVector3(static_cast<btScalar>(inertia.x()), static_cast<btScalar>(inertia.y()), static_cast<btScalar>(inertia.z())));
        links_[i] = std::make_unique<btRigidBody>(linkCi);
        links_[i]->setWorldTransform(linkWorldTrans[i]);
        links_[i]->setFriction(static_cast<btScalar>(groundFriction_));
        links_[i]->setActivationState(DISABLE_DEACTIVATION);
        world_->addRigidBody(links_[i].get());
    }

    hinges_.resize(18);
    btVector3 axis;
    getJointAxis(axis);
    for (int j = 0; j < 18; ++j) {
        btVector3 pivotA, pivotB;
        getJointPivotInParent(j, pivotA);
        getJointPivotInChild(j, pivotB);
        btRigidBody* bodyA = (JOINT_PARENT[j] < 0) ? base_.get() : links_[JOINT_PARENT[j]].get();
        btRigidBody* bodyB = links_[JOINT_CHILD[j] - 1].get();
        btHingeConstraint* hinge = new btHingeConstraint(*bodyA, *bodyB, pivotA, pivotB, axis, axis, true);
        hinge->setLimit(static_cast<btScalar>(-M_PI), static_cast<btScalar>(M_PI));
        hinge->enableAngularMotor(true, 0, 0.1f);
        hinges_[j] = std::unique_ptr<btHingeConstraint>(hinge);
        world_->addConstraint(hinges_[j].get(), true);
    }
}

void BulletSimulation::setRobotPose(double baseZ) {
    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(0, 0, static_cast<btScalar>(baseZ)));
    base_->setWorldTransform(t);
    base_->setLinearVelocity(btVector3(0, 0, 0));
    base_->setAngularVelocity(btVector3(0, 0, 0));
    for (auto& link : links_) {
        link->setLinearVelocity(btVector3(0, 0, 0));
        link->setAngularVelocity(btVector3(0, 0, 0));
    }
}

void BulletSimulation::reset() {
    setRobotPose(0.102);
    stepCount_ = 0;
}

void BulletSimulation::step(double dt) {
    if (!world_) return;
    world_->stepSimulation(static_cast<btScalar>(dt), 1, static_cast<btScalar>(dt));
    stepCount_++;
}

void BulletSimulation::setJointTorques(const double* torques) {
    for (int j = 0; j < 18; ++j) {
        btScalar t = static_cast<btScalar>(torques[j]);
        btScalar maxImpulse = std::abs(t) * static_cast<btScalar>(timeStep_) * 100.0f;
        btScalar targetVel = (t >= 0) ? 10.0f : -10.0f;
        hinges_[j]->enableAngularMotor(true, targetVel, maxImpulse);
    }
}

void BulletSimulation::getState(double* state) const {
    btTransform baseTrans = base_->getWorldTransform();
    btQuaternion q = baseTrans.getRotation();
    btScalar roll, pitch, yaw;
    btMatrix3x3(q).getEulerYPR(yaw, pitch, roll);
    state[0] = roll;
    state[1] = pitch;
    btVector3 angVel = base_->getAngularVelocity();
    state[2] = angVel.x();
    state[3] = angVel.y();
    for (int j = 0; j < 18; ++j) {
        state[4 + j] = hinges_[j]->getHingeAngle();
        btRigidBody* child = &hinges_[j]->getRigidBodyB();
        btVector3 w = child->getAngularVelocity();
        btVector3 axis;
        getJointAxis(axis);
        state[4 + 18 + j] = w.dot(axis);
    }
}

void BulletSimulation::getBasePosition(double* xyz) const {
    btVector3 o = base_->getWorldTransform().getOrigin();
    xyz[0] = o.x(); xyz[1] = o.y(); xyz[2] = o.z();
}

void BulletSimulation::getBaseLinearVelocity(double* vxyz) const {
    btVector3 v = base_->getLinearVelocity();
    vxyz[0] = v.x(); vxyz[1] = v.y(); vxyz[2] = v.z();
}

bool BulletSimulation::limbTipsInContact() const {
    // Tibia links are indices 2,5,8,11,14,17 (0-based: links_[2], [5], [8], ...)
    int tibiaLinkIndices[] = {2, 5, 8, 11, 14, 17};
    for (int i : tibiaLinkIndices) {
        int numManifolds = world_->getDispatcher()->getNumManifolds();
        for (int m = 0; m < numManifolds; ++m) {
            btPersistentManifold* manifold = world_->getDispatcher()->getManifoldByIndexInternal(m);
            const btCollisionObject* obA = manifold->getBody0();
            const btCollisionObject* obB = manifold->getBody1();
            const btRigidBody* tibia = links_[i].get();
            if ((obA == tibia || obB == tibia) && (obA == ground_.get() || obB == ground_.get()))
                return true;
        }
    }
    return false;
}

}  // namespace simplechimera
