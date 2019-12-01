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

void RigidBody::simulate(double step, int demoId)
{
	if (demoId == 3)
	{
		//applyForce(m_position, Vec3(0.0, -9.81, 0.0));
	}

	m_rotation += (Quat(m_angularVelocity.x, m_angularVelocity.y, m_angularVelocity.z, 0.0) * m_rotation) * step * 0.5;
	m_rotation = m_rotation.unit();

	m_angularMomentum += step * m_torque;
	m_torque = Vec3(0.0, 0.0, 0.0);

	m_angularVelocity = getRotatedInverseInertiaTensor().transformVector(m_angularMomentum);
	

	m_position += m_velocity * step;
	m_velocity += (m_force * step) / m_mass;
	m_force = Vec3(0.0, 0.0, 0.0);
	
	if (demoId == 0)
	{
		cout << "Angular Velocity: " << m_angularVelocity << endl;
		cout << "Linear Velocity: " << m_velocity << endl;
	}
}

Mat4 RigidBody::getTransformationMatrix()
{
	Mat4 scale(0.0);
	scale.initScaling(m_size.x, m_size.y, m_size.z);
	Mat4 translate(0.0);
	translate.initTranslation(m_position.x, m_position.y, m_position.z);
	return scale * m_rotation.getRotMat() * translate;
}

Mat4 RigidBody::getRotatedInverseInertiaTensor()
{
	Mat4 rotationMatrix = m_rotation.getRotMat();
	Mat4 rotationMatrixTransposed = m_rotation.getRotMat();
	rotationMatrixTransposed.transpose();
	return rotationMatrix * m_inertiaTensor.inverse() * rotationMatrixTransposed;
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

void RigidBodySystem::simulate(double step, int demoId)
{
	for (int i = 0; i < m_rigidbodies.size(); i++) {
		m_rigidbodies.at(i).simulate(step, demoId);
	}
}

void RigidBodySystem::resolveCollisions(int demoId)
{
	for (int i = 0; i < m_rigidbodies.size(); i++)
	{
		for (int j = 0; j < m_rigidbodies.size(); j++)
		{
			if (i != j)
			{
				RigidBody *rb1 = &m_rigidbodies.at(i);
				RigidBody *rb2 = &m_rigidbodies.at(j);
				CollisionInfo ci = checkCollisionSAT(rb1->getTransformationMatrix(), rb2->getTransformationMatrix());
				if (ci.isValid) {
					Vec3 localCollisionPos1 = ci.collisionPointWorld - rb1->m_position;
					Vec3 vel1 = rb1->m_velocity + cross(rb1->m_angularVelocity, localCollisionPos1);

					Vec3 localCollisionPos2 = ci.collisionPointWorld - rb2->m_position;
					Vec3 vel2 = rb2->m_velocity + cross(rb2->m_angularVelocity, localCollisionPos2);

					double relativeVelocity = dot(vel1 - vel2, ci.normalWorld);
					if (relativeVelocity < 0.0)
					{
						rb1->m_position -= localCollisionPos1 * ci.depth * 0.5;
						rb2->m_position -= localCollisionPos2 * ci.depth * 0.5;

						double energyCoefficient = 0.5;
						double impulseMagnitude = (-1.0 * (1.0 + energyCoefficient) * -relativeVelocity) / (
							(1.0 / rb1->m_mass)
							+ (1.0 / rb2->m_mass)
							+ dot((cross(rb1->getRotatedInverseInertiaTensor().transformVector(cross(localCollisionPos1, ci.normalWorld)), localCollisionPos1)
								 + cross(rb2->getRotatedInverseInertiaTensor().transformVector(cross(localCollisionPos2, ci.normalWorld)), localCollisionPos1)),
							  ci.normalWorld)
						);
						Vec3 impulse = impulseMagnitude * ci.normalWorld;

						rb1->m_velocity -= impulse / rb1->m_mass;
						rb2->m_velocity += impulse / rb2->m_mass;

						rb1->m_angularMomentum -= cross(localCollisionPos1, impulse);
						rb2->m_angularMomentum += cross(localCollisionPos2, impulse);
					}
				}
			}
		}
	}
}

void RigidBodySystem::draw(DrawingUtilitiesClass *duc)
{
	for (auto &rb : m_rigidbodies) {
		duc->drawRigidBody(rb.getTransformationMatrix());
	}
}

RigidBodySystem::~RigidBodySystem()
{
}
