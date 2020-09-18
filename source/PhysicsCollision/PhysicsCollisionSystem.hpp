#pragma once
#ifndef PHYSICSCOLLISIONSYSTEM_HPP
#define PHYSICSCOLLISIONSYSTEM_HPP
#include "btBulletDynamicsCommon.h"

namespace Mona {
	class PhysicsCollisionSystem {
	public:
		PhysicsCollisionSystem() {
			m_pCollisionConfiguration = new btDefaultCollisionConfiguration();
			m_pDispatcher = new btCollisionDispatcher(m_pCollisionConfiguration);
			m_pBroadphase = new btDbvtBroadphase();
			m_pSolver = new btSequentialImpulseConstraintSolver();
			m_pWorld = new btDiscreteDynamicsWorld(m_pDispatcher, m_pBroadphase, m_pSolver, m_pCollisionConfiguration);
		}
		~PhysicsCollisionSystem() {
			delete m_pWorld;
			delete m_pSolver;
			delete m_pBroadphase;
			delete m_pDispatcher;
			delete m_pCollisionConfiguration;

		}
	private:
		btBroadphaseInterface* m_pBroadphase;
		btCollisionConfiguration* m_pCollisionConfiguration;
		btCollisionDispatcher* m_pDispatcher;
		btConstraintSolver* m_pSolver;
		btDynamicsWorld* m_pWorld;

	};
}


#endif