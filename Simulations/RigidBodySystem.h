#include "Simulator.h"

class RigidBody
{
public:
	RigidBody(Vec3 position, Vec3 size, float mass);
	~RigidBody();

	void simulate(double step, Vec3 force, Vec3 forcePoint);

	double m_mass;
	Vec3 m_position;
	Vec3 m_size;
	Vec3 m_velocity;
	Quat m_rotation;
	Vec3 m_angularVelocity;
	Vec3 m_angularMomentum;

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
	void Simulate(double step, Vec3 &force, Vec3 &forcePoint);
	void draw(DrawingUtilitiesClass *duc);
	~RigidBodySystem();

private:
	std::vector<RigidBody> m_rigidbodies;
};

