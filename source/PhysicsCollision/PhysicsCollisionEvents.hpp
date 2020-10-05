#pragma once
#ifndef PHYSICSCOLLISIONEVENTS_HPP
#define PHYSICSCOLLISIONEVENTS_HPP
#include "../Event/Events.hpp"
#include "../World/ComponentHandle.hpp"
#include "RigidBodyComponent.hpp"
#include "CollisionInformation.hpp"
namespace Mona {

	struct StartCollisionEvent : public Event {
		static constexpr uint8_t eventIndex = GetEventIndex(EEventType::StartCollisionEvent);
		StartCollisionEvent(RigidBodyHandle&rb0, RigidBodyHandle&rb1, bool swaped, CollisionInformation& info) :
			firstRigidBody(rb0),
			secondRigidBody(rb1),
			areSwaped(swaped),
			collisionInfo(info)
		{}
		RigidBodyHandle& firstRigidBody;
		RigidBodyHandle& secondRigidBody;
		bool areSwaped;
		CollisionInformation& collisionInfo;

	};

	struct EndCollisionEvent : public Event {
		static constexpr uint8_t eventIndex = GetEventIndex(EEventType::EndCollisionEvent);
		EndCollisionEvent(RigidBodyHandle& rb0, RigidBodyHandle& rb1) :
			firstRigidBody(rb0),
			secondRigidBody(rb1)
		{}
		RigidBodyHandle& firstRigidBody;
		RigidBodyHandle& secondRigidBody;
	};
	
}
#endif