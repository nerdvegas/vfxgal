#ifndef _DGAL_LINE3__H_
#define _DGAL_LINE3__H_

#include <OpenEXR/ImathLine.h>
#include <limits>


namespace vfxgal {

	/*
	 * @brief equivalent_line
	 * @param allow_reverse If true, lines can be in opposite directions
	 * @returns True if the lines are equivalent
	 */
	template<typename T>
	bool equivalent_line(const Imath::Line3<T>& l1, const Imath::Line3<T>& l2, bool allow_reverse=false);


///////////////////////// impl

template<typename T>
bool equivalent_line(const Imath::Line3<T>& l1, const Imath::Line3<T>& l2, bool allow_reverse)
{
	typedef Imath::Vec3<T> vec_type;
	const T epsilon = std::numeric_limits<T>::epsilon();

	T dirdot = l1.dir.dot(l2.dir);
	if(allow_reverse)
		dirdot = fabs(dirdot);
	if(!Imath::equalWithAbsError(dirdot, T(1), epsilon))
		return false;

	vec_type posdiff = l2.pos - l1.pos;
	if(posdiff.length() > epsilon)
	{
		posdiff.normalize();
		T posdot = fabs(posdiff.dot(l1.dir));
		if(!Imath::equalWithAbsError(posdot, T(1), epsilon))
			return false;
	}

	return true;
}

}




#endif
