#include "Simulator.h"
#include "collisionDetect.h"

class RigidBody
{
public:
	RigidBody(Vec3 position, Vec3 size, float mass);
	~RigidBody();

	void applyForce(Vec3 &position, Vec3 &force);
	void simulate(double step, int demoId);
	Mat4 getTransformationMatrix();
	Mat4 getRotatedInverseInertiaTensor();

	double m_mass;
	Vec3 m_position;
	Vec3 m_size;
	Vec3 m_velocity;
	Quat m_rotation;
	Vec3 m_angularVelocity;
	Vec3 m_angularMomentum;

	Vec3 m_force;
	Vec3 m_torque;

	Mat4 m_inertiaTensor;
};

class RigidBodySystem
{
public:
	RigidBodySystem();
	void AddRigidBody(RigidBody &rigidbody);
	int RigidBodyCount();
	RigidBody *GetRigidBody(int index);
	void ClearRigidBodies();
	void simulate(double step, int demoId);
	void resolveCollisions(int demoId);
	void draw(DrawingUtilitiesClass *duc);
	~RigidBodySystem();

private:
	std::vector<RigidBody> m_rigidbodies;
};

