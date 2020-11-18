#pragma once
#ifndef SHAPETYPES_HPP
#define SHAPETYPES_HPP
#include <glm/glm.hpp>
namespace Mona {
	/*
	* Enumerador que representa la alineación de una figura. Por ejemplo, un cono tiene claramente un eje prefencial
	* este enumerador permite configurar cual es dicho eje.
	*/
	enum class ShapeAlignment {
		X,
		Y,
		Z
	};
	struct BoxShapeInformation {
		BoxShapeInformation(const glm::vec3& halfExtents = glm::vec3(1.0)) : m_boxHalfExtents(halfExtents) {}
		glm::vec3 m_boxHalfExtents;
	};

	struct ConeShapeInformation {

		ConeShapeInformation(float radius = 1.0f, float height = 1.0f, ShapeAlignment alignment = ShapeAlignment::Y) :
			m_radius(radius), m_height(height), m_alignment(alignment) {}

		float m_radius;
		float m_height;
		ShapeAlignment m_alignment;

	};

	struct SphereShapeInformation {
		SphereShapeInformation(float radius = 1.0f) : m_radius(radius) {}
		float m_radius;
	};

	struct CapsuleShapeInformation {
		CapsuleShapeInformation(float radius = 1.0f, float height = 1.0f, ShapeAlignment alignment = ShapeAlignment::Y) :
			m_radius(radius), m_height(height), m_alignment(alignment) {}

		float m_radius;
		float m_height;
		ShapeAlignment m_alignment;
	};

	struct CylinderShapeInformation {
		CylinderShapeInformation(const glm::vec3& halfExtents = glm::vec3(1.0), ShapeAlignment alignment = ShapeAlignment::Y) 
			: m_cylinderHalfExtents(halfExtents), m_alignment(alignment) {}

		glm::vec3 m_cylinderHalfExtents;
		ShapeAlignment m_alignment;
	};



}
#endif