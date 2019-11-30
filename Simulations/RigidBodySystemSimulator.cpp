#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_pRigidBodySystem = new RigidBodySystem();
}

const char * RigidBodySystemSimulator::getTestCasesStr(){
	return "Demo 1, Demo 2";
}

void RigidBodySystemSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:break;
		//TwAddVarRW(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT32, &m_iNumSpheres, "min=1");
		//TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");
	default:break;
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	const double pi = 3.14159265359;

	m_pRigidBodySystem->ClearRigidBodies();
	m_iTestCase = testCase;
	
	switch (m_iTestCase)
	{
	case 0:
		cout << "Demo 1" << endl;
		addRigidBody(Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(0.0, 0.0, 2 * pi * 0.25));
		m_pRigidBodySystem->Simulate(2.0, Vec3(0.3, 0.5, 0.25), Vec3(1.0, 1.0, 0.0));
		break;
	case 1:
		cout << "Demo 2" << endl;
		addRigidBody(Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(0.0, 0.0, 2 * pi * 0.25));
		break;
	default:
		cout << "Empty Test!" << endl;
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		m_externalForce += inputWorld * inputScale;
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 0:
		break;
	case 1:
		m_pRigidBodySystem->Simulate(timeStep, m_externalForce, Vec3(1.1, 1.1, 0.0));
		m_externalForce = Vec3(0.0);
		break;
	default:
		break;
	}
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	m_pRigidBodySystem->draw(DUC);
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_pRigidBodySystem->RigidBodyCount();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return m_pRigidBodySystem->GetRigidBody(i)->m_position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_pRigidBodySystem->GetRigidBody(i)->m_velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_pRigidBodySystem->GetRigidBody(i)->m_angularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	
}

void RigidBodySystemSimulator::addRigidBody(Vec3 &position, Vec3 &size, int mass)
{
	m_pRigidBodySystem->AddRigidBody(RigidBody(position, size, mass));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_pRigidBodySystem->GetRigidBody(i)->m_rotation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_pRigidBodySystem->GetRigidBody(i)->m_velocity = velocity;
}
