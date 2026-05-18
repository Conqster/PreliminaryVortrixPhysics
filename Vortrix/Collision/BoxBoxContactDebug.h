#pragma once


#include "Vortrix/Collision/ContactManifold.h"
#include "Vortrix/Maths/VortrixMaths.h"

namespace vx {
	struct BoxBoxContactDebug
	{
		BoxBoxContactDebug() = default;
		BoxBoxContactDebug(const Mat44& a, const Mat44& b,
			const Vec3& hfa, const Vec3& hfb,
			ContactManifold pManifold, const Vec3& axis, bool _active,
			int _axis_type) :
			transA(a), transB(b), halfExtentA(hfa), halfExtentB(hfb),
			manifold(pManifold), contactAxis(axis), active(_active), bestAxisType(_axis_type) {
		}

		Mat44 transA;
		Mat44 transB;

		Vec3 halfExtentA;
		Vec3 halfExtentB;

		ContactManifold manifold{nullptr, nullptr};

		Vec3 contactAxis;
		bool active = false;
		//0= A face, 1= B face, 2 = edge-edge
		int bestAxisType = -1;
	};

	static BoxBoxContactDebug sBoxBoxDebugInstance;

	class BoxShape;

	struct BoxVsBoxSAT
	{
		Float3 best_axis;
		int best_axis_type = -1; //0= A face, 1= B face, 2 = edge-edge
		//3 == special case inc vertex to ref's face

		BoxShape* ref;
		BoxShape* inc;

		Vec2 planeSize;
		Float3 manifoldCenter;

		Float3 planeTangent;
		bool hasPlane;
		Float3 planeBiTangent;
		float axisMinOverlap;

		Float3 contactNormal;
		int padding;

		/// genetared points are point of the inclident box not ref box
		/// as in sampling ref box is broken down to a box face plane 
		std::array<Float3, 16> pointsGenerated; //<-- max contact point 
		int generatedPt = 0;

		std::array<Float3, 5> contactPoints; //<-- max contact point  4 + 1padding
		int numContactPt = 0;

		//testing edge detections
		Float3 edgeCenter0;
		Float3 edgeCenter1;

		Float3 edgeA0;
		Float3 edgeA1;
		Float3 edgeB0;
		Float3 edgeB1;
		bool isEdgeDetection = false;

	};
	static std::vector<BoxVsBoxSAT> sBoxVsBoxSATDebugInstances;

}