/**
 * Minimal OpenGL/GLFW viewer for the Bullet simulation.
 * Draws all rigid bodies as boxes; camera follows the robot base.
 */
#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION 1
#endif

#include "balance_env.hpp"
#include "bullet_simulation.hpp"
#include "gui_viewer.hpp"
#include "kilowatt_chain.hpp"
#include <btBulletDynamicsCommon.h>
#include <GLFW/glfw3.h>
#include <cmath>
#include <cstdio>
#include <cstring>

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION 1
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

float camDistance = 0.5f;
float camYaw = 25.f * (float)(M_PI / 180);
float camPitch = -25.f * (float)(M_PI / 180);
float camTarget[3] = {0, 0, 0.04f};

// Box: 6 faces, 2 triangles each, CCW winding for outward normal.
// Faces: back(-z), front(+z), left(-x), right(+x), bottom(-y), top(+y)
static const int boxFaceIdx[6][6] = {
    {0,1,2, 0,2,3}, {4,5,6, 4,6,7}, {0,4,7, 0,7,3}, {1,2,6, 1,6,5}, {0,1,5, 0,5,4}, {3,2,6, 3,6,7}
};
static const float boxFaceNorm[6][3] = {
    {0,0,-1},{0,0,1}, {-1,0,0},{1,0,0}, {0,-1,0},{0,1,0}
};

void drawBox(const float* halfExtents, const float* color) {
    float x = halfExtents[0], y = halfExtents[1], z = halfExtents[2];
    float v[8][3] = {
        {-x,-y,-z},{x,-y,-z},{x,y,-z},{-x,y,-z},
        {-x,-y,z},{x,-y,z},{x,y,z},{-x,y,z}
    };
    float mat[4] = {color[0], color[1], color[2], 1.f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mat);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 20.f);
    glBegin(GL_TRIANGLES);
    for (int f = 0; f < 6; f++) {
        glNormal3fv(boxFaceNorm[f]);
        for (int t = 0; t < 6; t++)
            glVertex3fv(v[boxFaceIdx[f][t]]);
    }
    glEnd();
    glDisable(GL_LIGHTING);
    float edge[3] = { color[0] * 0.4f, color[1] * 0.4f, color[2] * 0.4f };
    glColor3fv(edge);
    glLineWidth(1.2f);
    glBegin(GL_LINES);
    int edges[24] = {0,1,1,2,2,3,3,0, 4,5,5,6,6,7,7,4, 0,4,1,5,2,6,3,7};
    for (int i = 0; i < 24; i += 2)
        { glVertex3fv(v[edges[i]]); glVertex3fv(v[edges[i+1]]); }
    glEnd();
    glEnable(GL_LIGHTING);
}

void drawGround() {
    const float half = 0.25f;
    float gray[3] = {0.45f, 0.48f, 0.52f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, gray);
    glNormal3f(0, 0, 1);
    glBegin(GL_QUADS);
    glVertex3f(-half, -half, 0); glVertex3f(half, -half, 0); glVertex3f(half, half, 0); glVertex3f(-half, half, 0);
    glEnd();
    glDisable(GL_LIGHTING);
    glColor3f(0.3f, 0.33f, 0.36f);
    const float step = 0.02f;
    glBegin(GL_LINES);
    for (float a = -half; a <= half + 1e-5f; a += step) {
        glVertex3f(a, -half, 0.001f); glVertex3f(a, half, 0.001f);
        glVertex3f(-half, a, 0.001f); glVertex3f(half, a, 0.001f);
    }
    glEnd();
    glEnable(GL_LIGHTING);
}

void drawWorld(btDynamicsWorld* world) {
    if (!world) return;
    int n = world->getNumCollisionObjects();
    for (int i = 0; i < n; i++) {
        btCollisionObject* obj = world->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (!body) continue;
        const btTransform& trans = obj->getWorldTransform();
        btVector3 o = trans.getOrigin();
        btMatrix3x3 b = trans.getBasis();
        float mat[16] = {
            (float)b[0][0], (float)b[1][0], (float)b[2][0], 0,
            (float)b[0][1], (float)b[1][1], (float)b[2][1], 0,
            (float)b[0][2], (float)b[1][2], (float)b[2][2], 0,
            (float)o.x(), (float)o.y(), (float)o.z(), 1
        };
        btCollisionShape* shape = obj->getCollisionShape();
        btBoxShape* box = shape ? (btBoxShape*)shape : nullptr;
        if (box && shape->getShapeType() == BOX_SHAPE_PROXYTYPE) {
            btVector3 he = box->getHalfExtentsWithMargin();
            float halfExt[3] = {(float)he.x(), (float)he.y(), (float)he.z()};
            float color[3] = {0.3f, 0.5f, 0.3f};
            if (i == 0) { color[0] = 0.5f; color[1] = 0.4f; color[2] = 0.3f; }  // ground
            else if (i == 1) { color[0] = 0.3f; color[1] = 0.5f; color[2] = 0.3f; }  // base
            glPushMatrix();
            glMultMatrixf(mat);
            drawBox(halfExt, color);
            glPopMatrix();
        }
    }
}

void setCamera(int width, int height) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = (float)width / (float)(height ? height : 1);
    float fovY = 45.f * (float)(M_PI / 180);
    float near = 0.02f, far = 2.f;
    float t = near * std::tan(fovY * 0.5f);
    float r = t * aspect;
    glFrustum(-r, r, -t, t, near, far);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0, 0, -camDistance);
    glRotatef(camPitch * (180.f / (float)M_PI), 1, 0, 0);
    glRotatef(camYaw * (180.f / (float)M_PI), 0, 0, 1);
    glTranslatef(-camTarget[0], -camTarget[1], -camTarget[2]);
}

void setupLighting() {
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);
    float lightPos[4] = {0.3f, 0.2f, 0.6f, 0.f};  // directional from above-right
    float lightDiff[4] = {0.85f, 0.85f, 0.9f, 1.f};
    float lightAmb[4] = {0.35f, 0.38f, 0.42f, 1.f};
    float lightSpec[4] = {0.5f, 0.5f, 0.5f, 1.f};
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiff);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmb);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpec);
}

}  // namespace

namespace simplechimera {

double runSimulationWithGui(
    BulletSimulation& sim,
    BalanceEnv& env,
    KilowattChain& controller,
    int maxSteps,
    double amplitude
) {
    if (!glfwInit()) {
        std::fprintf(stderr, "Failed to init GLFW\n");
        return -1e9;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    GLFWwindow* win = glfwCreateWindow(800, 600, "SimpleChimera (Bullet)", nullptr, nullptr);
    if (!win) {
        glfwTerminate();
        std::fprintf(stderr, "Failed to create window\n");
        return -1e9;
    }
    glfwMakeContextCurrent(win);
    glfwShowWindow(win);
    glfwFocusWindow(win);
    glfwSwapInterval(1);
    std::fprintf(stderr, "GUI window opened. If you don't see it, check behind other windows or the Dock.\n");

    double state[STATE_DIM];
    double action[ACTION_DIM];
    double totalReward = 0;
    int step = 0;
    const double simDt = sim.getTimeStep();
    const double targetFrameTime = 1.0 / 60.0;  // 60 fps
    double simTimeAccum = 0;
    sim.getState(state);

    while (!glfwWindowShouldClose(win) && step < maxSteps) {
        // Run simulation in real-time: advance by ~1/60 s per frame so motion is visible
        simTimeAccum += targetFrameTime;
        const int maxStepsPerFrame = (int)(targetFrameTime / simDt) + 2;
        bool episodeDone = false;
        for (int k = 0; k < maxStepsPerFrame && simTimeAccum >= simDt; k++) {
            controller.step(state, action);
            double reward;
            bool done;
            env.step(action, state, &reward, &done);
            totalReward += reward;
            step++;
            simTimeAccum -= simDt;
            if (done || step >= maxSteps) { episodeDone = true; break; }
        }
        if (episodeDone) break;

        double xyz[3];
        sim.getBasePosition(xyz);
        camTarget[0] = (float)xyz[0];
        camTarget[1] = (float)xyz[1];
        camTarget[2] = (float)xyz[2];

        int w, h;
        glfwGetFramebufferSize(win, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.12f, 0.14f, 0.18f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
        setCamera(w, h);
        setupLighting();
        drawGround();
        drawWorld(sim.getWorld());
        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    glfwDestroyWindow(win);
    glfwTerminate();
    return totalReward;
}

}  // namespace simplechimera
