#pragma once

#include "Core.h"
#include "Vec4.h"

namespace vx
{

	/*
	* Mat44:- 4x4 matrix class
	* 4x4 Column-major matrix for affine and general tranformation
	* 
	* Primary usage:
	* - Affine transforms (rotation, translation, scale)
	* - 3x3 operations via upper-left submatrix (rotation / scaling)
	* 
	* Coordinate system:
	* - Right-handed coordniate system
	* - Column-major storage
	* - Column vectors
	* - Vectors are transformed as: v' = M * v
	* 
	* Mathematical layout (row x column):
	* 
	* | R00 R01 R02 Tx  | 
	* | R10 R11 R12 Ty  | 
	* | R20 R21 R22 Tz  | 
	* |  0   0   0   1  | 
	* 
	* Memory layout (column-major): 
	*	Column 0: [ R00 R10 R20 0 ] // indices [0..3]
	*	Column 1: [ R01 R11 R21 0 ]	// indices [4..7]
	*	Column 2: [	R02 R12 R32 0 ]	// indices [8..11]
	*	Column 3: [	Tx	Ty	Tz	1 ]	// indices [12..15]
	*	
	* Components:
	* - Rij: Rotation/scale components
	* - Tx, Ty, Tz: Translation components
	* - Bottom row is [0, 0,0, 1] for affine matrices
	* 
	* Basis representation:
	* 
	*	|X.x  Y.x  Z.x  T.x|
	*	|X.y  Y.y  Z.y  T.y|
	*	|X.z  Y.z  Z.z  T.z|
	*	| 0    0    0    1 |
	* 
	* - Columns 0-2 represent basis vectors (X, Y, Z)
	* - Column 3 reprensent translation
	* 
	* Vector sematics:
	*  - Direction vectors assume w = 0 (no translation applied)
	*  - Position vectors assume w = 1 (translation applied)
	*
	* 
	* Affine helpers:
	* - Multiply3x3()		: ignores translation
	* - TransformDirection(): direction vectors (w = 0)
	* - Transform()			: position vectors (w = 1)
	* 
	* Transform composition example:
	*	Mat44 M = Translation(t) * Rotation(q) * Scale(s)
	* 
	* - Mat44 Mtr = Mat44::RotationTranslation(q, t) * Mat44:Scale(s)
	* - Mat44 Mtrs = Mat44::RotationTranslation(q, t) * Mat44:Scale(s)
	* - translate a Matrix M; M.Multiple(vec4(p, 1.0f)) etc
	* 
	* 
	* Performance note:
	* - Mat44 is 64 bytes (cache-line aligned).
	* - Using Mat44 avoid Mat33 padding/straddling issues in arrays.
	* - Upper-left 3x3 region provides full rotation functionality.
	*/
	class alignas(16) Mat44
	{
	public:
		
		VX_INLINE Mat44() = default;
		explicit VX_INLINE Mat44(const float diagonal);
		VX_INLINE Mat44(const Vec4& col0, const Vec4& col1, const Vec4& col2, const Vec4& col3);
		VX_INLINE Mat44(const Vec4& col0, const Vec4& col1, const Vec4& col2);
		
		/// | 1 0 0 0 |
		/// | 0 1 0 0 |
		/// | 0 0 1 0 |
		/// | 0 0 0 1 |
		/// @return identity matrix
		static VX_INLINE Mat44 Identity();
		/// debug / utility matrix containing linear indices
		/// Values stored in memeory order (column-major)
		/// | 0 4 8 12 |
		/// | 1 5 9 13 |
		/// | 2 6 10 14 |
		/// | 3 7 11 15 | 
		static VX_INLINE Mat44 Dummy();

		/// Build translation matrix
		/// | 1 0 0 t.x |
		/// | 0 1 0 t.y |
		/// | 0 0 1 t.z |
		/// | 0 0 0 1 |
		static VX_INLINE Mat44 Translation(const Vec3& v);
		/// Build uniform scale matrix
		/// | s 0 0 0 |
		/// | 0 s 0 0 |
		/// | 0 0 s 0 |
		/// | 0 0 0 1 |
		static VX_INLINE Mat44 Scale(float scale);
		/// Build non-uniform scale matrix
		/// | s.x  0   0  0 |
		/// |  0  s.y  0  0 |
		/// |  0   0  s.z 0 |
		/// |  0   0   0  1 |
		static VX_INLINE Mat44 Scale(const Vec3& scale);
		/// Rotation about X axis (pitch), radians
		static VX_INLINE Mat44 RotationX(float angle);
		/// Rotation about Y axis (yaw), radians
		static VX_INLINE Mat44 RotationY(float angle);
		/// Rotation about Z axis (row), radians
		static VX_INLINE Mat44 RotationZ(float angle);
		/// Build matrix from orthonormal basis vectors
		/// Columns represent X, Y, Z axes; translation is zero
		static VX_INLINE Mat44 Basis(const Vec3& x, const Vec3& y, const Vec3& z);
		/// Build matrix from basis vectors and translation
		static VX_INLINE Mat44 BasisTranslation(const Vec3& x, const Vec3& y, const Vec3& z, const Vec3& t);
		
		/// Builds rotation matrix from a unit quaternion
		/// Translation is zero, bottom [0, 0, 0, 1]
		static VX_INLINE Mat44 Rotation(const Quat& q);
		/// Build rotation + translation matrix from unit quaternion
		static VX_INLINE Mat44 RotationTranslation(const Quat& q, const Vec3& t);

		/// Accessor
		/// Access matrix element (row, column)
		VX_INLINE float& operator()(int row, int column);
		/// @return Read matrix element (row, column)
		VX_INLINE float operator()(int row, int column) const;

		/// Access column vector by index
		/// @param i Index [0..3]
		/// @return column vector i
		VX_INLINE Vec4 GetColumn(int i) const;
		/// Access column vector xyz part by index
		/// @param i Index [0..3]
		/// @return xyz part of column vector i
		VX_INLINE Vec3 GetColumn3(int i) const;
		VX_INLINE void SetColumn(int i, const Vec4& rhs);
		VX_INLINE void SetColumn3(int i, const Vec3& rhs);

		/// @return X basis vector (column 0)
		VX_INLINE Vec3 GetAxisX() const { return mCol[0]; }
		/// @return Y basis vector (column 1)
		VX_INLINE Vec3 GetAxisY() const { return mCol[1]; }
		/// @return Z basis vector (column 2)
		VX_INLINE Vec3 GetAxisZ() const { return mCol[2]; }
		/// @return translation vector (column 3)
		VX_INLINE Vec3 GetTranslation() const { return mCol[3]; }

		VX_INLINE void SetAxisX(const Vec3& v, const float w = 0.0f) { mCol[0] = Vec4(v, w);}
		VX_INLINE void SetAxisY(const Vec3& v, const float w = 0.0f) { mCol[1] = Vec4(v, w);}
		VX_INLINE void SetAxisZ(const Vec3& v, const float w = 0.0f) { mCol[2] = Vec4(v, w);}
		VX_INLINE void SetTranslation(const Vec3& v, const float w = 1.0f) { mCol[3] = Vec4(v, w); }

		VX_INLINE Vec4 GetDiagonal() const { return Vec4(mFloats[0], mFloats[5], mFloats[10], mFloats[15]); }
		VX_INLINE Vec3 GetDiagonal3() const { return Vec3(mFloats[0], mFloats[5], mFloats[10]); }
		VX_INLINE void SetDiagonal(const Vec4& rhs) { mFloats[0] = rhs.X(); mFloats[5] = rhs.Y(); mFloats[10] = rhs.Z(); mFloats[15] = rhs.W(); }
		VX_INLINE void SetDiagonal3(const Vec3& rhs) { mFloats[0] = rhs.X(); mFloats[5] = rhs.Y(); mFloats[10] = rhs.Z(); mFloats[15] = 1.0f; }


		/// Comparison
		VX_INLINE bool operator == (const Mat44& rhs) const;
		VX_INLINE bool operator != (const Mat44& rhs) const { return !(*this == rhs); }

		/// @return determinant of upper-left 3x3 matrix
		VX_INLINE float Determinant3x3() const;
		/// @return basis handness (+1 right-handed, 0 degarded, -1 left-handed)
		VX_INLINE int GetBasisHandness() const;
		/// @return true, if matrix is affine (bottom row = [0 0 0 1])
		VX_INLINE bool IsAffine() const;
		/// @return true if upper 3x3 is affine-compatible
		VX_INLINE bool IsAffine3x3() const;
		/// Validates that teh upper 3x3 matrix forms an orthonormal basis
		/// column must have unit length and be perpendicular to the others.
		/// @param tolerance Maximum allowed deviation from ideal orthonormality
		/// @return true, if the basis is a pure rotation(no scale, no shear).
		VX_INLINE bool IsOrthonormal(float tolerance = 1e-4f) const;
		/// @return transpose of upper 3x3 matrix
		VX_INLINE Mat44 Transposed3x3() const;
		/// @return full transpose matrix
		VX_INLINE Mat44 Transposed() const;
		///// @return inverse 
		//VX_INLINE Mat44 Inverse() const;
		/// @return inverse upper 3x3 matrix
		VX_INLINE Mat44 Inverse3x3() const;
		/// Invert affine matrix (rotation + translation only)
		/// Undefined behaviour, if matrix contains scale or shear
		VX_INLINE Mat44 InverseAffine() const;

		/// Multiply vector by upper 3x3 matrix
		VX_INLINE Vec3 Multiply3x3(const Vec3& rhs) const;
		/// Multiply vector by transposed upper 3x3 matrix
		VX_INLINE Vec3 Multiply3x3Transposed(const Vec3& rhs) const;
		/// Multiply vector by affine matrix
		VX_INLINE Vec3 MultiplyAffine(const Vec3& rhs) const;
		VX_INLINE Mat44 Multiply3x3(const Mat44& rhs) const;
		VX_INLINE Mat44 Multiply3x3LeftTransposed(const Mat44& rhs) const;
		VX_INLINE Mat44 Multiply3x3RightTransposed(const Mat44& rhs) const;
		VX_INLINE Mat44 Multiply(const Mat44& rhs) const;
		VX_INLINE Vec4 Multiply(const Vec4& rhs) const;
		VX_INLINE Mat44 MultiplyAffine(const Mat44& rhs) const;

		VX_INLINE Mat44 Add(const Mat44& rhs) const;
		VX_INLINE Mat44 AddAffine(const Mat44& rhs) const;
		/// Cross Mat44(Mat33) x Vec3
		static VX_INLINE Mat44 SkewSymmetric3x3(const Vec3& rhs);
		VX_INLINE Mat44 operator+(const Mat44& rhs)const = delete;
		VX_INLINE Mat44& operator+=(const Mat44& rhs) = delete;


		/// Transform a position vector by this matrix.
		/// Applies rotation and translation
		VX_INLINE Vec3 Transform(const Vec3& vec)const;
		VX_INLINE static Vec3 Transform(const Mat44& matrix, const Vec3& translate);
		
		/// Tranforms a position vector by inverse of this matrix (assume affine). 
		/// removes translation + inverse rotation
		VX_INLINE Vec3 TransformInverse(const Vec3& vector) const;
		/// Tranforms a position vector by inverse of matrix (assume affine).
		VX_INLINE static Vec3 TransformInverse(const Mat44& matrix, const Vec3& translate);
		
		/// Transform a direction vector by this matrix (ignore translation)
		VX_INLINE Vec3 TransformDirection(const Vec3& vec) const;
		/// Transform a direction vector by the inverse of this matrix.
		/// Assue pure rotation (no scale/shear).
		VX_INLINE Vec3 TransformInverseDirection(const Vec3& vec) const;

		/// Sets the rotation part of matrix from unit quaternion 
		/// Translation preserved, bottom [0, 0, 0, 1]
		VX_INLINE void SetRotation(const Quat& q);
		/// Set both rotation and translation.
		VX_INLINE void SetRotationAndTranslation(const Quat& q, const Vec3& pos);

		VX_INLINE Mat44 GetRotationMat33() const;
		/// Extract the rotation component of matrix as quartenion 
		/// matrix must contain pure rotation (no scale/shear)
		VX_INLINE Quat GetRotationQuat() const;

		/// Pre-scale affine matrix (scale axes)
		/// M = S * M
		VX_INLINE void ScaleGlobal(const Vec3& s)
		{
			mCol[0] *= s.X();
			mCol[1] *= s.Y();
			mCol[2] *= s.Z();
			mCol[3].Multiply3(s);
			mCol[3][3] = 1.0f;
		}
		VX_INLINE Mat44 ScaledGlobal(const Vec3& scale)
		{
			Mat44 M = *this;
			M.ScaleGlobal(scale);
			return M;
		}
		/// Post-scale affine matrix
		/// M = M * S
		VX_INLINE void ScaleLocal(const Vec3& s)
		{
			mCol[0] *= s.X();
			mCol[1] *= s.Y();
			mCol[2] *= s.Z();
		}

		VX_INLINE Mat44 ScaledLocal(const Vec3& scale)
		{
			Mat44 M = *this;
			M.ScaleLocal(scale);
			return M;
		}

		/// Decomposing matrix into rotation+translation matrix and vector scale
		/// Uses Gram-Schmidt orthonormalisation
		/// see https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process
		/// @param o_scale Output non-uniform scale
		/// @return Mat44(rotation+translation) 
		/// @return Vec3 o_scale 
		VX_INLINE Mat44 Decompose(Vec3& o_scale) const;
		/// Orthonormalise upper 3x3 basis vectors
		VX_INLINE void MakeOrthonormal();


		/// NEW
		const float* Data() const { return &mFloats[0]; }
		float* Data() { return &mFloats[0]; }
		/// Multiply Affine operator
		VX_INLINE Mat44 operator*(const Mat44& rhs) const { return MultiplyAffine(rhs); }


		VX_INLINE Mat44 Inverse() const
		{
			Mat44 result;

			const float* m = mFloats;
			float* inv = result.mFloats;

			inv[0] = m[5] * m[10] * m[15] -
				m[5] * m[11] * m[14] -
				m[9] * m[6] * m[15] +
				m[9] * m[7] * m[14] +
				m[13] * m[6] * m[11] -
				m[13] * m[7] * m[10];

			inv[4] = -m[4] * m[10] * m[15] +
				m[4] * m[11] * m[14] +
				m[8] * m[6] * m[15] -
				m[8] * m[7] * m[14] -
				m[12] * m[6] * m[11] +
				m[12] * m[7] * m[10];

			inv[8] = m[4] * m[9] * m[15] -
				m[4] * m[11] * m[13] -
				m[8] * m[5] * m[15] +
				m[8] * m[7] * m[13] +
				m[12] * m[5] * m[11] -
				m[12] * m[7] * m[9];

			inv[12] = -m[4] * m[9] * m[14] +
				m[4] * m[10] * m[13] +
				m[8] * m[5] * m[14] -
				m[8] * m[6] * m[13] -
				m[12] * m[5] * m[10] +
				m[12] * m[6] * m[9];

			inv[1] = -m[1] * m[10] * m[15] +
				m[1] * m[11] * m[14] +
				m[9] * m[2] * m[15] -
				m[9] * m[3] * m[14] -
				m[13] * m[2] * m[11] +
				m[13] * m[3] * m[10];

			inv[5] = m[0] * m[10] * m[15] -
				m[0] * m[11] * m[14] -
				m[8] * m[2] * m[15] +
				m[8] * m[3] * m[14] +
				m[12] * m[2] * m[11] -
				m[12] * m[3] * m[10];

			inv[9] = -m[0] * m[9] * m[15] +
				m[0] * m[11] * m[13] +
				m[8] * m[1] * m[15] -
				m[8] * m[3] * m[13] -
				m[12] * m[1] * m[11] +
				m[12] * m[3] * m[9];

			inv[13] = m[0] * m[9] * m[14] -
				m[0] * m[10] * m[13] -
				m[8] * m[1] * m[14] +
				m[8] * m[2] * m[13] +
				m[12] * m[1] * m[10] -
				m[12] * m[2] * m[9];

			inv[2] = m[1] * m[6] * m[15] -
				m[1] * m[7] * m[14] -
				m[5] * m[2] * m[15] +
				m[5] * m[3] * m[14] +
				m[13] * m[2] * m[7] -
				m[13] * m[3] * m[6];

			inv[6] = -m[0] * m[6] * m[15] +
				m[0] * m[7] * m[14] +
				m[4] * m[2] * m[15] -
				m[4] * m[3] * m[14] -
				m[12] * m[2] * m[7] +
				m[12] * m[3] * m[6];

			inv[10] = m[0] * m[5] * m[15] -
				m[0] * m[7] * m[13] -
				m[4] * m[1] * m[15] +
				m[4] * m[3] * m[13] +
				m[12] * m[1] * m[7] -
				m[12] * m[3] * m[5];

			inv[14] = -m[0] * m[5] * m[14] +
				m[0] * m[6] * m[13] +
				m[4] * m[1] * m[14] -
				m[4] * m[2] * m[13] -
				m[12] * m[1] * m[6] +
				m[12] * m[2] * m[5];

			inv[3] = -m[1] * m[6] * m[11] +
				m[1] * m[7] * m[10] +
				m[5] * m[2] * m[11] -
				m[5] * m[3] * m[10] -
				m[9] * m[2] * m[7] +
				m[9] * m[3] * m[6];

			inv[7] = m[0] * m[6] * m[11] -
				m[0] * m[7] * m[10] -
				m[4] * m[2] * m[11] +
				m[4] * m[3] * m[10] +
				m[8] * m[2] * m[7] -
				m[8] * m[3] * m[6];

			inv[11] = -m[0] * m[5] * m[11] +
				m[0] * m[7] * m[9] +
				m[4] * m[1] * m[11] -
				m[4] * m[3] * m[9] -
				m[8] * m[1] * m[7] +
				m[8] * m[3] * m[5];

			inv[15] = m[0] * m[5] * m[10] -
				m[0] * m[6] * m[9] -
				m[4] * m[1] * m[10] +
				m[4] * m[2] * m[9] +
				m[8] * m[1] * m[6] -
				m[8] * m[2] * m[5];

			float det =
				m[0] * inv[0] +
				m[1] * inv[4] +
				m[2] * inv[8] +
				m[3] * inv[12];

			VX_ASSERT(VxAbs(det) > kEpsilon, "Matrix is singular");

			float invDet = 1.0f / det;
			for (int i = 0; i < 16; ++i)
				inv[i] *= invDet;

			return result;
		}

		/// M = T * M (World space)
		/// object along fixed X, Y, Z axes of universe
		VX_INLINE void TranslateGlobal(const Vec3& t)
		{
			mCol[3].Add3(t);
			mCol[3][3] = 1.0f; //<-- ensure homogenouse
		}

		/// M = M * T
		/// move obj along its basis axes
		VX_INLINE void TranslateLocal(const Vec3& t)
		{
			const Vec3& local_offset = Multiply3x3(t);
			//add oriented offset to world pos
			mCol[3].Add3(local_offset);
			mCol[3][3] = 1.0f; //<-- ensure homogenouse
		}
		VX_INLINE static Mat44 TranslatedLocal(const Mat44& M, const Vec3& t)
		{
			Mat44 Mt = M;
			Mt.TranslateLocal(t);
			return Mt;
		}
		VX_INLINE static Mat44 TranslatedGlobal(const Mat44& M, const Vec3& t)
		{
			Mat44 Mt = M;
			Mt.TranslateGlobal(t);
			return Mt;
		}


		VX_INLINE Mat44 Abs() const
		{
			Mat44 t;
			t.mCol[0] = mCol[0].Abs();
			t.mCol[1] = mCol[1].Abs();
			t.mCol[2] = mCol[2].Abs();
			t.mCol[3] = mCol[3].Abs();
			return t;
		}


		/// <summary>
		/// Need to optimise this later
		/// </summary>
		/// <param name="q"></param>
		/// <param name="t"></param>
		/// <returns></returns>
		static VX_INLINE Mat44 InverseRotationTranslation(const Quat& q, const Vec3& t);


		friend std::ostream& operator<<(std::ostream& os, const Mat44& m);
	private:
		//Get column by index
		/// Retruns the i-th axis vector column).
		/// i column index [0-3].
		/// @return Vec4 representing the axis/column
		VX_INLINE Vec4& operator[](int column);
		VX_INLINE Vec4 operator[](int column) const;
		union
		{
			Vec4 mCol[4];
			float mFloats[16];
		};
	};
}

#include "Mat44.inl"