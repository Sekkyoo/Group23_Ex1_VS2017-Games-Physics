#include "RigidBodySystem.h"

RigidBody::RigidBody(Vec3 position, Vec3 size, float mass)
{
	m_position = position;
	m_size = size;
	m_velocity = Vec3(0.0, 0.0, 0.0);
	m_rotation = Quat(0.0, 0.0, 0.0, 1.0);
	m_angularVelocity = Vec3(0.0, 0.0, 0.0);
	m_angularMomentum = Vec3(0.0, 0.0, 0.0);
	m_mass = mass;

	m_force = Vec3(0.0, 0.0, 0.0);
	m_torque = Vec3(0.0, 0.0, 0.0);

	m_inertiaTensor = Mat4(
		(size.Y*size.Y + size.Z*size.Z) * (mass / 12.0), 0.0, 0.0, 0.0,
		0.0, (size.X*size.X + size.Z*size.Z) * (mass / 12.0), 0.0, 0.0,
		0.0, 0.0, (size.X*size.X + size.Y*size.Y) * (mass / 12.0), 0.0,
		0.0, 0.0, 0.0, 1.0
	);
}

RigidBody::~RigidBody()
{
}

void RigidBody::applyForce(Vec3 &position, Vec3 &force)
{
	m_torque += cross(force, m_position - position);
	m_force += force;
}

void RigidBody::simulate(double step)
{
	m_rotation += (Quat(m_angularVelocity.x, m_angularVelocity.y, m_angularVelocity.z, 0.0) * m_rotation) * step * 0.5;
	m_rotation.norm();

	m_angularMomentum += step * m_torque;
	m_torque = Vec3(0.0, 0.0, 0.0);

	Mat4 rotationMatrix = m_rotation.getRotMat();
	Mat4 rotationMatrixTransposed = m_rotation.getRotMat();
	rotationMatrixTransposed.transpose();
	Mat4 rotatedInverseInertiaTensor = rotationMatrix * m_inertiaTensor.inverse() * rotationMatrixTransposed;

	m_angularVelocity = rotatedInverseInertiaTensor.transformVector(m_angularMomentum);
	cout << "Angular Velocity: " << m_angularVelocity << endl;

	m_position += m_velocity * step;
	m_velocity += (m_force * step) / m_mass;
	m_force = Vec3(0.0, 0.0, 0.0);
	cout << "Linear Velocity: " << m_velocity << endl;
}

RigidBodySystem::RigidBodySystem()
{
	m_rigidbodies = vector<RigidBody>();
}

void RigidBodySystem::AddRigidBody(RigidBody &rigidbody)
{
	m_rigidbodies.push_back(rigidbody);
}

int RigidBodySystem::RigidBodyCount()
{
	return m_rigidbodies.size();
}

RigidBody *RigidBodySystem::GetRigidBody(int index)
{
	return &m_rigidbodies.at(index);
}

void RigidBodySystem::ClearRigidBodies()
{
	m_rigidbodies.clear();
}

void RigidBodySystem::Simulate(double step)
{
	for (auto &rb : m_rigidbodies) {
		rb.simulate(step);
	}
}

void RigidBodySystem::draw(DrawingUtilitiesClass *duc)
{
	for (auto rb : m_rigidbodies) {
		Mat4 scale(0.0);
		scale.initScaling(rb.m_size.x, rb.m_size.y, rb.m_size.z);
		Mat4 translate(0.0);
		translate.initTranslation(rb.m_position.x, rb.m_position.y, rb.m_position.z);
		duc->drawRigidBody(scale * rb.m_rotation.getRotMat() * translate);
	}
}

RigidBodySystem::~RigidBodySystem()
{
}
