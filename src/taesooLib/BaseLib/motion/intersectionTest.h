#pragma once

#include "../../BaseLib/math/tvector.h"
class Box2D
{
	public:
		Box2D();
		Box2D(vector2 const& m, vector2 const& M);
		vector2 min;
		vector2 max;
		double distance(vector2 const& p);
		// works only when p is inside the box.
		double negativeDistance(vector2 const& p);
		bool contains(vector2 const& pt, double margin=0)	const;
};

//	Mainly for picking objects.
/*
	A plane is defined in 3D space by the equation
        Ax + By + Cz + D = 0
		(normal=(A,B,C))
 */

class Plane
{
public:
    Plane ();
	// vNormal should be a unit vector, offset=-d.
	Plane(double nx, double ny, double nz, double d);
    Plane (const vector3& vNormal, m_real offset);
    Plane (const vector3& vNormal, const vector3& vPoint);
    Plane (const vector3& vPoint0, const vector3& vPoint1,
        const vector3& vPoint2);

	// + if a point is normal-side, otherwise - values
    m_real distance (const vector3& point) const;

    /** define this plane based on 3 points. */
    void setPlane(const vector3& vPoint0, const vector3& vPoint1,
        const vector3& vPoint2);

	/** define this plane based on a normal and a point. */
	void setPlane(const vector3& vNormal, const vector3& vPoint);

	// should be unit vector
	vector3 normal;
    m_real d;
};

class Sphere
{
public:
	vector3 center;
	m_real radius;	// half radius
	Sphere(vector3 c, m_real r):center(c), radius(r){}

	bool isInside(std::vector<Plane>& selectionVolume) const;
};

class Ray
{
protected:
    vector3 mOrigin;
    vector3 mDirection;
public:
    Ray():mOrigin(0,0,0), mDirection(0,0,1){}
    Ray(const vector3& origin, const vector3& direction)
        :mOrigin(origin), mDirection(direction) {}

	vector3& origin()					{ return mOrigin;}
	vector3 const& origin() const		{ return mOrigin;}

	vector3& direction()				{ return mDirection;}
	vector3 const& direction() const	{ return mDirection;}

	vector3 getPoint(m_real t) const
	{
		return mOrigin + (mDirection * t);
	}
	void translate(vector3 const& t) { mOrigin+=t; }

	std::pair<bool, m_real> intersects(const Plane& p) const;
	// planes form a convex volume, and normal vectors face outside
	std::pair<bool, m_real> intersects(const std::vector<Plane>& planes) const;
	std::pair<bool, m_real> intersects(const Sphere& s) const;
	std::pair<bool, m_real> intersects(const vector3& a,
    const vector3& b, const vector3& c, const vector3& normal,
    bool positiveSide, bool negativeSide) const;

	std::pair<bool, m_real> intersects(const vector3& a,
    const vector3& b, const vector3& c, bool bCCW=true) const;
};

namespace intersectionTest
{
	// line segment
	class LineSegment
	{
		vector3 mOrigin;
		vector3 mDirection;	// unit length
		m_real mLength;		// end point=mOrigin+mDirection*mLength
	public:

		LineSegment(){}
		LineSegment(const vector3& from, const vector3& to){ resetPosition(from, to);}

		void resetPosition(const vector3& from, const vector3& to);

		vector3 const& origin() const	{ return mOrigin;}
		vector3 const& dir() const		{ return mDirection;}
		m_real length() const			{ return mLength;}
		vector3 pos(m_real t) const		{ return mOrigin+mDirection*t;}
		vector3 target() const			{ return pos(mLength);}

		// 가장 가까운 두 점 사이의 거리.
		m_real minDist(LineSegment const& other) const;
		
		// returns t in [0,1] (not distance)
		m_real minDistTime(vector3 const& pos) const;

		// 가장 가까워지는 t와 그때 둘을 떼내는 방향.
		std::pair<m_real, vector3> minDistDir(LineSegment const& other) const;
	};

	LineSegment Ray2LineSegment(::Ray const& r);

	bool testTriTriOverlap(vector3 const& p1,  vector3 const& q1, vector3 const& r1,
			vector3 const& p2,  vector3 const& q2, vector3 const& r2);

	/** Borrowed from Ogre3D. A 3D box aligned with the x/y/z axes.
	@remarks
	This class represents a simple box which is aligned with the
	axes. Internally it only stores 2 points as the extremeties of
	the box, one which is the minima of all 3 axes, and the other
	which is the maxima of all 3 axes. This class is typically used
	for an axis-aligned bounding box (AABB) for collision and
	visibility determination.
	*/
	class AABB
	{
	public:
		enum Extent
		{
			EXTENT_NULL,
			EXTENT_FINITE,
			EXTENT_INFINITE
		};
	protected:

		vector3 mMinimum;
		vector3 mMaximum;
		Extent mExtent;

	public:
		/*
		1-----2
		/|    /|
		/ |   / |
		5-----4  |
		|  0--|--3
		| /   | /
		|/    |/
		6-----7
		*/
		typedef enum {
			FAR_LEFT_BOTTOM = 0,
			FAR_LEFT_TOP = 1,
			FAR_RIGHT_TOP = 2,
			FAR_RIGHT_BOTTOM = 3,
			NEAR_RIGHT_BOTTOM = 7,
			NEAR_LEFT_BOTTOM = 6,
			NEAR_LEFT_TOP = 5,
			NEAR_RIGHT_TOP = 4
		} CornerEnum;
		inline AABB()
		{
			// Default to a null box
			setMinimum( -0.5, -0.5, -0.5 );
			setMaximum( 0.5, 0.5, 0.5 );
			mExtent = EXTENT_NULL;
		}
		inline AABB(Extent e)
		{
			setMinimum( -0.5, -0.5, -0.5 );
			setMaximum( 0.5, 0.5, 0.5 );
			mExtent = e;
		}

		inline AABB(const AABB & rkBox)
		{
			if (rkBox.isNull())
				setNull();
			else if (rkBox.isInfinite())
				setInfinite();
			else
				setExtents( rkBox.mMinimum, rkBox.mMaximum );
		}

		inline AABB( const vector3& min, const vector3& max )
		{
			setExtents( min, max );
		}

		inline AABB(
			m_real mx, m_real my, m_real mz,
			m_real Mx, m_real My, m_real Mz )
		{
			setExtents( mx, my, mz, Mx, My, Mz );
		}

		AABB& operator=(const AABB& rhs)
		{
			// Specifically override to avoid copying mpCorners
			if (rhs.isNull())
				setNull();
			else if (rhs.isInfinite())
				setInfinite();
			else
				setExtents(rhs.mMinimum, rhs.mMaximum);

			return *this;
		}

		~AABB()
		{
		}

		inline void enlarge(m_real margin)
		{
			if(isNull() || isInfinite())
				return;

			mMinimum.x-=margin;
			mMinimum.y-=margin;
			mMinimum.z-=margin;


			mMaximum.x+=margin;
			mMaximum.y+=margin;
			mMaximum.z+=margin;
		}


		/** Gets the minimum corner of the box.
		*/
		inline const vector3& getMinimum(void) const
		{
			return mMinimum;
		}

		/** Gets a modifiable version of the minimum
		corner of the box.
		*/
		inline vector3& getMinimum(void)
		{
			return mMinimum;
		}

		/** Gets the maximum corner of the box.
		*/
		inline const vector3& getMaximum(void) const
		{
			return mMaximum;
		}

		/** Gets a modifiable version of the maximum
		corner of the box.
		*/
		inline vector3& getMaximum(void)
		{
			return mMaximum;
		}


		/** Sets the minimum corner of the box.
		*/
		inline void setMinimum( const vector3& vec )
		{
			mExtent = EXTENT_FINITE;
			mMinimum = vec;
		}

		inline void setMinimum( m_real x, m_real y, m_real z )
		{
			mExtent = EXTENT_FINITE;
			mMinimum.x = x;
			mMinimum.y = y;
			mMinimum.z = z;
		}

		/** Changes one of the components of the minimum corner of the box
		used to resize only one dimension of the box
		*/
		inline void setMinimumX(m_real x)
		{
			mMinimum.x = x;
		}

		inline void setMinimumY(m_real y)
		{
			mMinimum.y = y;
		}

		inline void setMinimumZ(m_real z)
		{
			mMinimum.z = z;
		}

		/** Sets the maximum corner of the box.
		*/
		inline void setMaximum( const vector3& vec )
		{
			mExtent = EXTENT_FINITE;
			mMaximum = vec;
		}

		inline void setMaximum( m_real x, m_real y, m_real z )
		{
			mExtent = EXTENT_FINITE;
			mMaximum.x = x;
			mMaximum.y = y;
			mMaximum.z = z;
		}

		/** Changes one of the components of the maximum corner of the box
		used to resize only one dimension of the box
		*/
		inline void setMaximumX( m_real x )
		{
			mMaximum.x = x;
		}

		inline void setMaximumY( m_real y )
		{
			mMaximum.y = y;
		}

		inline void setMaximumZ( m_real z )
		{
			mMaximum.z = z;
		}

		/** Sets both minimum and maximum extents at once.
		*/
		inline void setExtents( const vector3& min, const vector3& max )
		{
            assert( (min.x <= max.x && min.y <= max.y && min.z <= max.z) &&
                "The minimum corner of the box must be less than or equal to maximum corner" );

			mExtent = EXTENT_FINITE;
			mMinimum = min;
			mMaximum = max;
		}

		inline void setExtents(
			m_real mx, m_real my, m_real mz,
			m_real Mx, m_real My, m_real Mz )
		{
            assert( (mx <= Mx && my <= My && mz <= Mz) &&
                "The minimum corner of the box must be less than or equal to maximum corner" );

			mExtent = EXTENT_FINITE;

			mMinimum.x = mx;
			mMinimum.y = my;
			mMinimum.z = mz;

			mMaximum.x = Mx;
			mMaximum.y = My;
			mMaximum.z = Mz;

		}

		/** gets the position of one of the corners
		*/
		vector3 getCorner(CornerEnum cornerToGet) const
		{
			switch(cornerToGet)
			{
			case FAR_LEFT_BOTTOM:
				return mMinimum;
			case FAR_LEFT_TOP:
				return vector3(mMinimum.x, mMaximum.y, mMinimum.z);
			case FAR_RIGHT_TOP:
				return vector3(mMaximum.x, mMaximum.y, mMinimum.z);
			case FAR_RIGHT_BOTTOM:
				return vector3(mMaximum.x, mMinimum.y, mMinimum.z);
			case NEAR_RIGHT_BOTTOM:
				return vector3(mMaximum.x, mMinimum.y, mMaximum.z);
			case NEAR_LEFT_BOTTOM:
				return vector3(mMinimum.x, mMinimum.y, mMaximum.z);
			case NEAR_LEFT_TOP:
				return vector3(mMinimum.x, mMaximum.y, mMaximum.z);
			case NEAR_RIGHT_TOP:
				return mMaximum;
			default:
				return vector3();
			}
		}


		/** Merges the passed in box into the current box. The result is the
		box which encompasses both.
		*/
		void merge( const AABB& rhs )
		{
			// Do nothing if rhs null, or this is infinite
			if ((rhs.mExtent == EXTENT_NULL) || (mExtent == EXTENT_INFINITE))
			{
				return;
			}
			// Otherwise if rhs is infinite, make this infinite, too
			else if (rhs.mExtent == EXTENT_INFINITE)
			{
				mExtent = EXTENT_INFINITE;
			}
			// Otherwise if current null, just take rhs
			else if (mExtent == EXTENT_NULL)
			{
				setExtents(rhs.mMinimum, rhs.mMaximum);
			}
			// Otherwise merge
			else
			{
				vector3 min = mMinimum;
				vector3 max = mMaximum;
				max.makeCeil(rhs.mMaximum);
				min.makeFloor(rhs.mMinimum);

				setExtents(min, max);
			}

		}

		/** Extends the box to encompass the specified point (if needed).
		*/
		inline void merge( const vector3& point )
		{
			switch (mExtent)
			{
			case EXTENT_NULL: // if null, use this point
				setExtents(point, point);
				return;

			case EXTENT_FINITE:
				mMaximum.makeCeil(point);
				mMinimum.makeFloor(point);
				return;

			case EXTENT_INFINITE: // if infinite, makes no difference
				return;
			}

			assert( false && "Never reached" );
		}



		/** Transforms the box according to the affine matrix supplied.
		@remarks
		By calling this method you get the axis-aligned box which
		surrounds the transformed version of this box. Therefore each
		corner of the box is transformed by the matrix, then the
		extents are mapped back onto the axes to produce another
		AABB. Useful when you have a local AABB for an object which
		is then transformed.
		@note
		The matrix must be an affine matrix. @see Matrix4::isAffine.
		*/
		void transform(const matrix4& m)
		{
			//assert(m.isAffine());

			// Do nothing if current null or infinite
			if ( mExtent != EXTENT_FINITE )
				return;

			vector3 centre = getCenter();
			vector3 halfSize = getHalfSize();

			vector3 newCentre;
			newCentre.mult(m, centre);
			vector3 newHalfSize(
				ABS(m.m[0][0]) * halfSize.x + ABS(m.m[0][1]) * halfSize.y + ABS(m.m[0][2]) * halfSize.z,
				ABS(m.m[1][0]) * halfSize.x + ABS(m.m[1][1]) * halfSize.y + ABS(m.m[1][2]) * halfSize.z,
				ABS(m.m[2][0]) * halfSize.x + ABS(m.m[2][1]) * halfSize.y + ABS(m.m[2][2]) * halfSize.z);

			setExtents(newCentre - newHalfSize, newCentre + newHalfSize);
		}

		/** Sets the box to a 'null' value i.e. not a box.
		*/
		inline void setNull()
		{
			mExtent = EXTENT_NULL;
		}

		/** Returns true if the box is null i.e. empty.
		*/
		inline bool isNull(void) const
		{
			return (mExtent == EXTENT_NULL);
		}

		/** Returns true if the box is finite.
		*/
		bool isFinite(void) const
		{
			return (mExtent == EXTENT_FINITE);
		}

		/** Sets the box to 'infinite'
		*/
		inline void setInfinite()
		{
			mExtent = EXTENT_INFINITE;
		}

		/** Returns true if the box is infinite.
		*/
		bool isInfinite(void) const
		{
			return (mExtent == EXTENT_INFINITE);
		}

		/** Returns whether or not this box intersects another. */
		inline bool intersects(const AABB& b2) const
		{
			// Early-fail for nulls
			if (this->isNull() || b2.isNull())
				return false;

			// Early-success for infinites
			if (this->isInfinite() || b2.isInfinite())
				return true;

			// Use up to 6 separating planes
			if (mMaximum.x < b2.mMinimum.x)
				return false;
			if (mMaximum.y < b2.mMinimum.y)
				return false;
			if (mMaximum.z < b2.mMinimum.z)
				return false;

			if (mMinimum.x > b2.mMaximum.x)
				return false;
			if (mMinimum.y > b2.mMaximum.y)
				return false;
			if (mMinimum.z > b2.mMaximum.z)
				return false;

			// otherwise, must be intersecting
			return true;

		}

		/// Calculate the area of intersection of this box and another
		inline AABB intersection(const AABB& b2) const
		{
            if (this->isNull() || b2.isNull())
			{
				return AABB();
			}
			else if (this->isInfinite())
			{
				return b2;
			}
			else if (b2.isInfinite())
			{
				return *this;
			}

			vector3 intMin = mMinimum;
            vector3 intMax = mMaximum;

            intMin.makeCeil(b2.getMinimum());
            intMax.makeFloor(b2.getMaximum());

            // Check intersection isn't null
            if (intMin.x < intMax.x &&
                intMin.y < intMax.y &&
                intMin.z < intMax.z)
            {
                return AABB(intMin, intMax);
            }

            return AABB();
		}

		/// Calculate the volume of this box
		m_real volume(void) const
		{
			switch (mExtent)
			{
			case EXTENT_NULL:
				return 0.0f;

			case EXTENT_FINITE:
				{
					vector3 diff = mMaximum - mMinimum;
					return diff.x * diff.y * diff.z;
				}

			case EXTENT_INFINITE:
				return DBL_MAX;

			default: // shut up compiler
				assert( false && "Never reached" );
				return 0.0f;
			}
		}

		/** Scales the AABB by the vector given. */
		inline void scale(const vector3& s)
		{
			// Do nothing if current null or infinite
			if (mExtent != EXTENT_FINITE)
				return;

			// NB assumes centered on origin
			vector3 min = mMinimum * s;
			vector3 max = mMaximum * s;
			setExtents(min, max);
		}

		/** Tests whether the vector point is within this box. */
		bool intersects(const vector3& v) const
		{
			switch (mExtent)
			{
			case EXTENT_NULL:
				return false;

			case EXTENT_FINITE:
				return(v.x >= mMinimum.x  &&  v.x <= mMaximum.x  &&
					v.y >= mMinimum.y  &&  v.y <= mMaximum.y  &&
					v.z >= mMinimum.z  &&  v.z <= mMaximum.z);

			case EXTENT_INFINITE:
				return true;

			default: // shut up compiler
				assert( false && "Never reached" );
				return false;
			}
		}
		/// Gets the centre of the box
		vector3 getCenter(void) const
		{
			assert( (mExtent == EXTENT_FINITE) && "Can't get center of a null or infinite AAB" );

			return vector3(
				(mMaximum.x + mMinimum.x) * 0.5,
				(mMaximum.y + mMinimum.y) * 0.5,
				(mMaximum.z + mMinimum.z) * 0.5);
		}
		/// Gets the size of the box
		vector3 getSize(void) const
		{
			switch (mExtent)
			{
			case EXTENT_NULL:
				return vector3(0,0,0);

			case EXTENT_FINITE:
				return mMaximum - mMinimum;

			case EXTENT_INFINITE:
				return vector3(DBL_MAX, DBL_MAX, DBL_MAX);

			default: // shut up compiler
				assert( false && "Never reached" );
				return vector3(0,0,0);
			}
		}
		/// Gets the half-size of the box
		vector3 getHalfSize(void) const
		{
			switch (mExtent)
			{
			case EXTENT_NULL:
				return vector3(0,0,0);

			case EXTENT_FINITE:
				return (mMaximum - mMinimum) * 0.5;

			case EXTENT_INFINITE:
				return vector3(
					DBL_MAX,
					DBL_MAX,
					DBL_MAX);

			default: // shut up compiler
				assert( false && "Never reached" );
				return vector3(0,0,0);
			}
		}

        /** Tests whether the given point contained by this box.
        */
        bool contains(const vector3& v) const
        {
            if (isNull())
                return false;
            if (isInfinite())
                return true;

            return mMinimum.x <= v.x && v.x <= mMaximum.x &&
                   mMinimum.y <= v.y && v.y <= mMaximum.y &&
                   mMinimum.z <= v.z && v.z <= mMaximum.z;
        }

        /** Tests whether another box contained by this box.
        */
        bool contains(const AABB& other) const
        {
            if (other.isNull() || this->isInfinite())
                return true;

            if (this->isNull() || other.isInfinite())
                return false;

            return this->mMinimum.x <= other.mMinimum.x &&
                   this->mMinimum.y <= other.mMinimum.y &&
                   this->mMinimum.z <= other.mMinimum.z &&
                   other.mMaximum.x <= this->mMaximum.x &&
                   other.mMaximum.y <= this->mMaximum.y &&
                   other.mMaximum.z <= this->mMaximum.z;
        }

        /** Tests 2 boxes for equality.
        */
        bool operator== (const AABB& rhs) const
        {
            if (this->mExtent != rhs.mExtent)
                return false;

            if (!this->isFinite())
                return true;

            return this->mMinimum == rhs.mMinimum &&
                   this->mMaximum == rhs.mMaximum;
        }

        /** Tests 2 boxes for inequality.
        */
        bool operator!= (const AABB& rhs) const
        {
            return !(*this == rhs);
        }

	};


}


