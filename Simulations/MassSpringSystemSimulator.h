#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change


struct MassPoint {
	Vec3 position;
	Vec3 orignalPosition;
	Vec3 Velocity;
	Vec3 orignalVelocity;
	bool isFixed;
};

struct Spring {
	int masspoint1;
	int masspoint2;
	float initialLength;
};

struct MassSpringSystem {
	MassPoint* massPoints;
	int massPointCount;
	Spring* springs;
	int springCount;
	float damping;
	float stiffness;
	float mass;
	Vec3 externalForce;
	bool draw;
	bool simulate;
	int startIntegration = EULER;
	bool clamp = true;
};

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	void ResetSystem();
	void InitSetUps();

	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void simulateOneTimestepOnConsole(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);

	void stepEuler(float timeStep);
	void stepLeapFrog(float timeStep);
	void stepMidPoint(float timeStep);
	Vec3* MassSpringSystemSimulator::computeAcceloration();
	Vec3* MassSpringSystemSimulator::computeAcceloration(Vec3* points);
	Vec3 ClampVector(Vec3 input, float min, float max, int pointIndex);

	void ChangeSetUp(MassSpringSystem system);
	MassSpringSystem CreateSetUp1();
	MassSpringSystem CreateSetUp2();
	MassSpringSystem CreateSetUp3();
	MassSpringSystem CreateSetUp4();
	MassSpringSystem CreateExtraSetUp();

	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	int m_massPointCount;
	int m_massPiontLimit;
	int m_springCount;
	int m_springLimit;
	int m_mouseMovementThreshHold = 0.2f;
	int m_testCase = 0;
	bool m_useGravity = false;
	float m_gravity = -0.002;

	const int fieldBuffer = 10;

	// UI Attributes
	Vec3 m_externalForce;
	Vec3 m_mouseOffset = Vec3();
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	MassSpringSystem * m_systems;
	MassPoint * m_massPoints;
	Spring * m_springs;
};
#endif