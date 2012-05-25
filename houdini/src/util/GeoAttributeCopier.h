#ifndef _HDKUTILS_GEOATTRIBUTECOPIER__H_
#define _HDKUTILS_GEOATTRIBUTECOPIER__H_

#if VFXGAL_HOU_MAJOR_VER > 11
#error "Houdini-12+ not yet supported."
#endif

#include <map>
#include <vector>
#include <utility>
#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>
#include <boost/unordered_map.hpp>
#include <boost/mpl/vector.hpp>
#include <GEO/GEO_AttributeHandleList.h>
#include <GU/GU_Detail.h>
#include <UT/UT_ThreadedAlgorithm.h>
#include <GEO/GEO_PrimPoly.h>
#include <stdexcept>


namespace vfxgal_hou {


	/*
	 * @class GeoAttributeCopier
	 * @brief
	 * Copies attributes from source geometry(s) to a destination geometry.
	 * Attributes can be copied from one label to another (eg "foo" to "bah");
	 * Attributes can be copied from one dict type to another (eg prim data to point);
	 * Attributes can be copied with remapping applied (eg dest[7] = src[112]);
	 * Attributes can be copied with interpolated remapping applied (eg dest[7] =
	 * (src[55]*0.4 + src[76]*0.6);
	 * Interpolated vertex attribute remapping is also supported;
	 * Values can be sparsely copied from src to dest, and different copies with different
	 * remappings can be consecutively applied to the same dest attribute.
	 * In cases where the attribute type does not support interpolation, the 'closest'
	 * value is chosen - eg if dest[7] = src[55]*0.4 + src[76]*0.6, src[76] will be used.
	 * Attributes of different data types cannot be copied - for example you can't copy
	 * string attributes into an int attribute. In this case, the dest attribute will be
	 * replaced by a new attrib of the matching data type.
	 * Any out-of-range indices found in any remapping data will result in that part of
	 * the data copy being silently skipped (the rest of the attribute copy will continue
	 * on unchanged).
	 */
	class GeoAttributeCopier
	{
		struct no_remap{};

	public:

		/*
		 * @brief
		 * When adding a copy to GeoAttributeCopier, we need one of these methods to specify the behavior.
		 *     GAC_REPLACE : Any existing copy will be replaced if it has same id with the new copy.
		 *     GAC_SKIP    : New copy will be skipped if there is already an existing same id.
		 *     GAC_MERGE   : New copy will be merged with the existing one with same id into a vector.
		 *                   The new one will be appended to the end.
		 */
		enum compMethod {
			GAC_REPLACE,
			GAC_SKIP,
			GAC_MERGE
		};

		/*
		 * @brief Remapping types
		 * remapping_vector: copies src[vec[n]] to dest[n], for n in 0..len(vec)-1.
		 * remapping_map: copies src[second] to dest[first].
		 * interpolated_remap: copies the sum of fractions of multiple src values
		 * to dest[first].
		 * interpolated_vertex_remap: copies, for each vertex in dest[first], the sum of
		 * fractions of multiple src values. Note that this is only valid for (vertex ->
		 * vertex) remapping, and the src indices index into the src poly's vertices (so
		 * will be 0..N-1 for an N-sided src poly).
		 */
		typedef std::vector<unsigned int> 								remapping_vector;
		typedef boost::unordered_map<unsigned int, unsigned int> 		remapping_map;

		typedef std::pair<unsigned int, float>							contrib_type;
		typedef std::vector<contrib_type>								contribs_type;
		typedef boost::unordered_map<unsigned int, contribs_type>		interpolated_remap;

		typedef std::vector<contribs_type>								contribs_vector;
		typedef std::pair<unsigned int, contribs_vector>				vertex_contribs;
		typedef boost::unordered_map<unsigned int, vertex_contribs>		interpolated_vertex_remap;

		typedef boost::mpl::vector<
			no_remap,
			const remapping_vector*,
			const remapping_map*,
			const interpolated_remap*,
			const interpolated_vertex_remap*
		> remapping_type_vec;

		typedef boost::make_variant_over<remapping_type_vec>::type remapping_type;


		/*
		 * @brief
		 * Create an attrib copying context.
		 * @param destGeo Geometry to write attributes to.
		 */
		GeoAttributeCopier(GU_Detail& destGeo);

		/*
		 * @brief add
		 * Add an attrib copy to this copying context.
		 * @param srcGeo Geo to copy attrib from.
		 * @param srcAttribName Name of source attrib.
		 * @param srcDict Source attrib dict.
		 * @param destAttribName Name of dest attrib.
		 * @param destDict Dest attrib dict.
		 * @param remapping Remapping. See 'Remapping types' above for more details.
		 * @param overwrite If true, any attrib copy that already exists and is going to
		 * copy to (destAttribName, destDict) will be replaced.
		 */
		void add(const GU_Detail& srcGeo, const std::string& srcAttribName,
			GEO_AttributeOwner srcDict, const std::string& destAttribName,
			GEO_AttributeOwner destDict,
			const remapping_type& remapping = remapping_type(),
			compMethod comp=GAC_REPLACE);

		/*
		 * @brief add
		 * Add an attrib copy for all attribs of a specific dict.
		 * @param srcGeo Geo to copy attrib from.
		 * @param srcDict Source attrib dict.
		 * @param destDict Dest attrib dict.
		 * @param remapping Remapping. See 'Remapping types' above for more details.
		 * @param skipP If true, the 'P' attrib will not be copied.
		 * @param overwrite (see first add() for more details).
		 */
		void add(const GU_Detail& srcGeo, GEO_AttributeOwner srcDict,
			GEO_AttributeOwner destDict, const remapping_type& remapping = remapping_type(),
			bool skipP = true, compMethod comp=GAC_REPLACE);

		/*
		 * @brief add
		 * Add an attrib copy for all attribs of a specific dict and data-type.
		 * @param srcGeo Geo to copy attrib from.
		 * @param srcDict Source attrib dict.
		 * @param destDict Dest attrib dict.
		 * @param type Type of attrib to copy from.
		 * @param remapping Remapping. See 'Remapping types' above for more details.
		 * @param skipP If true, the 'P' attrib will not be copied.
		 * @param overwrite (see first add() for more details).
		 */
		void add(const GU_Detail& srcGeo, GEO_AttributeOwner srcDict,
			GEO_AttributeOwner destDict, GB_AttribType type,
			const remapping_type& remapping = remapping_type(),
			bool skipP = true, compMethod comp=GAC_REPLACE);

		/*
		 * @brief remove
		 * Remove an attrib copy from this copying context.
		 * @returns True if an attrib was removed, false otherwise
		 */
		bool remove(const std::string& destAttribName, GEO_AttributeOwner destDict);

		/*
		 * @brief remove
		 * Remove from this copying context all of the (destAttribName, destDict) entries
		 * in c.
		 * @note It does not matter what source/dest geos are being used in c, or what
		 * remapping data - this information is not used.
		 */
		void remove(const GeoAttributeCopier& c);

		/*
		 * @brief apply
		 * Perform the attributes copy.
		 * @param createOnly If true, the destination attribs will only be created; the
		 * actual values from the source geometry(s) will not be copied. Otherwise, the
		 * attribs will be created and their data will be copied from the source geo(s).
		 */
		void apply(bool createOnly = false);

	protected:

		struct attrib_copy
		{
			attrib_copy():m_srcGeo(NULL){}

			const GU_Detail* m_srcGeo;
			std::string m_srcName, m_destName;
			GEO_AttributeOwner m_srcDict, m_destDict;
			remapping_type m_remapping;
		};

		bool createHandles(const attrib_copy& copy, GEO_AttributeHandle& hSrc,
			GEO_AttributeHandle& hDest);

		bool apply(const attrib_copy& copy);

		template<typename T>
		bool apply1(const attrib_copy& copy);

		template<typename T, typename U>
		bool apply2(const attrib_copy& copy);

	protected:

		typedef std::pair<std::string, GEO_AttributeOwner>	attrib_id;
		typedef std::map< attrib_id, vector<attrib_copy> >	attrib_copy_map;

		GU_Detail& m_destGeo;
		attrib_copy_map m_copies;
	};


	namespace detail {

		typedef map<int, int> stringTableMapping;

		// attribute adaptors
		template<typename T>
		struct attrib_traits{};

		template<>
		struct attrib_traits<GEO_Detail>
		{
			typedef GEO_Detail		type;
			typedef GEO_Detail 		elem_type;

			static inline type& get(GU_Detail& g) 									{ return g; }
			static inline const type& get(const GU_Detail& g)						{ return g; }

			static inline unsigned int entries(const type& t)						{ return 1; }
			static inline elem_type* getElem(type& t, unsigned int i)				{ return &t; }
			static inline const elem_type* getElem(const type& t, unsigned int i)	{ return &t; }
		};

		template<>
		struct attrib_traits<GEO_PointList>
		{
			typedef GEO_PointList	type;
			typedef GEO_Point		elem_type;

			static inline type& get(GU_Detail& g) 									{ return g.points(); }
			static inline const type& get(const GU_Detail& g)						{ return g.points(); }

			static inline unsigned int entries(const type& t)						{ return t.entries(); }
			static inline elem_type* getElem(type& t, unsigned int i)				{ return t[i]; }
			static inline const elem_type* getElem(const type& t, unsigned int i)	{ return t[i]; }
		};

		template<>
		struct attrib_traits<GEO_PrimList>
		{
			typedef GEO_PrimList	type;
			typedef GEO_PrimPoly	elem_type;

			static inline type& get(GU_Detail& g) 									{ return g.primitives(); }
			static inline const type& get(const GU_Detail& g)						{ return g.primitives(); }

			static inline unsigned int entries(const type& t)						{ return t.entries(); }
			static inline elem_type* getElem(type& t, unsigned int i)				{ return dynamic_cast<elem_type*>(t[i]); }
			static inline const elem_type* getElem(const type& t, unsigned int i)	{ return dynamic_cast<const elem_type*>(t[i]); }
		};

		template<>
		struct attrib_traits<GEO_PrimPoly>
		{
			typedef GEO_PrimPoly	type;
			typedef GEO_Vertex		elem_type;

			static inline unsigned int entries(const type& t)						{ return t.getVertexCount(); }
			static inline elem_type* getElem(type& t, unsigned int i)				{ return &(t.getVertex(i)); }
			static inline const elem_type* getElem(const type& t, unsigned int i)	{ return &(t.getVertex(i)); }
		};

		// element adaptors
		template<typename T>
		struct elem_traits
		{
			static inline unsigned int getVertexCount(const T&) { return 0; }
			static inline const GEO_Vertex* getVertex(const T&, unsigned int i) { return NULL; }
			static inline GEO_Vertex* getVertex(T&, unsigned int i) { return NULL; }
		};

		template<>
		struct elem_traits<GEO_PrimPoly>
		{
			static inline unsigned int getVertexCount(const GEO_PrimPoly& t) {
				return t.getVertexCount();
			}

			static inline const GEO_Vertex* getVertex(const GEO_PrimPoly& t, unsigned int i) {
				return &(t.getVertex(i));
			}

			static inline GEO_Vertex* getVertex(GEO_PrimPoly& t, unsigned int i) {
				return &(t.getVertex(i));
			}
		};

		// remapping visitors
		class is_mapped : public boost::static_visitor<bool>
		{
		public:
			template<typename T>
			bool operator()(T&) const { return false; }

			template<typename T>
			bool operator()(T* t) const { return (t != NULL); }
		};

		template<typename T, typename U>
		class apply_remapping : public boost::static_visitor<>
		{
		public:
			apply_remapping(const T& srcElems, U& destElems,
				GEO_AttributeHandle& hSrc, GEO_AttributeHandle& hDest,
				const stringTableMapping* strTableMap)
			: 	m_srcElems(srcElems),
				m_destElems(destElems),
				m_hSrc(hSrc), m_hDest(hDest),
				m_strTableMap(strTableMap)
			{}

		void operator()(const GeoAttributeCopier::no_remap&) const {}
		void operator()(const GeoAttributeCopier::remapping_vector* remap) const;
		void operator()(const GeoAttributeCopier::remapping_map* remap) const;
		void operator()(const GeoAttributeCopier::interpolated_remap* remap) const;
		void operator()(const GeoAttributeCopier::interpolated_vertex_remap* remap) const;

		protected:
			const T& m_srcElems;
			U& m_destElems;
			GEO_AttributeHandle& m_hSrc;
			GEO_AttributeHandle& m_hDest;
			const stringTableMapping* m_strTableMap;
		};

		template<>
		void apply_remapping<GEO_PrimList,GEO_PrimList>::operator()(
			const GeoAttributeCopier::interpolated_vertex_remap* remap) const;

		// element copying
		template<typename T, typename U>
		static void attrib_elem_copy(const T& srcElems, U& destElems,
			unsigned int srcIndex, unsigned int destIndex,
			GEO_AttributeHandle& hSrc, GEO_AttributeHandle& hDest,
			const stringTableMapping* strTableMap = NULL);

		template<typename T, typename U>
		static void attrib_base_elem_copy(const T& srcElem, U& destElem,
			GEO_AttributeHandle& hSrc, GEO_AttributeHandle& hDest,
			const stringTableMapping* strTableMap = NULL);

		// interpolated element copying
		struct non_interpolatable_tag{};
		struct interp_float_tag{};
		struct interp_vector_tag{};

		template<typename TypeTag>
		struct interpolated_elem_copier
		{
			template<typename T, typename U>
			static void apply(const T& srcElems, U& destElem,
				const GeoAttributeCopier::contribs_type& contribs,
				GEO_AttributeHandle& hSrc, GEO_AttributeHandle& hDest,
				const stringTableMapping* strTableMap = NULL) {}
		};

		template<typename T, typename U, typename TypeTag>
		void apply_interpolated_remap(
			const GeoAttributeCopier::interpolated_remap* remap,
			const T& srcElems, U& destElems,
			GEO_AttributeHandle& hSrc, GEO_AttributeHandle& hDest,
			const stringTableMapping* strTableMap = NULL);

		template<typename TypeTag>
		void apply_interpolated_vertex_remap(
			const GeoAttributeCopier::interpolated_vertex_remap* remap,
			const GEO_PrimList& srcElems, GEO_PrimList& destElems,
			GEO_AttributeHandle& hSrc, GEO_AttributeHandle& hDest,
			const stringTableMapping* strTableMap = NULL);

		void getStringTableMapping(GEO_AttributeHandle &srcAttrH,
			GEO_AttributeHandle &destAttrH, stringTableMapping& o_stm);

	} // detail ns


///////////////////////// impl

template<typename T, typename U>
void detail::apply_remapping<T,U>::operator()(const GeoAttributeCopier::remapping_vector* remap) const
{
	unsigned int numSrcElems = attrib_traits<T>::entries(m_srcElems);
	unsigned int numDestElems = attrib_traits<U>::entries(m_destElems);
	unsigned int nelems = std::min(numDestElems, static_cast<unsigned int>(remap->size()));

	for(unsigned int i=0; i<nelems; ++i)
	{
		unsigned int j = (*remap)[i];
		if(j < numSrcElems)
			detail::attrib_elem_copy(m_srcElems, m_destElems, j, i, m_hSrc, m_hDest, m_strTableMap);
	}
}


template<typename T, typename U>
void detail::apply_remapping<T,U>::operator()(const GeoAttributeCopier::remapping_map* remap) const
{
	unsigned int numSrcElems = attrib_traits<T>::entries(m_srcElems);
	unsigned int numDestElems = attrib_traits<U>::entries(m_destElems);

	for(GeoAttributeCopier::remapping_map::const_iterator it=remap->begin(); it!=remap->end(); ++it)
	{
		unsigned int j = it->second; // src
		unsigned int i = it->first; // dest

		if (i<numDestElems && j<numSrcElems)
			detail::attrib_elem_copy(m_srcElems, m_destElems, j, i, m_hSrc, m_hDest, m_strTableMap);
	}
}


template<typename T, typename U>
void detail::apply_remapping<T,U>::operator()(
	const GeoAttributeCopier::interpolated_remap* remap) const
{
	GB_AttribType t = m_hSrc.getAttribute()->getType();
	switch(t)
	{
	case GB_ATTRIB_INT:
	case GB_ATTRIB_INDEX:
	case GB_ATTRIB_MIXED:
	{
		apply_interpolated_remap<T,U,non_interpolatable_tag>(
			remap, m_srcElems, m_destElems, m_hSrc, m_hDest, m_strTableMap);
	}
	break;
	case GB_ATTRIB_FLOAT:
	{
		apply_interpolated_remap<T,U,interp_float_tag>(
			remap, m_srcElems, m_destElems, m_hSrc, m_hDest);
	}
	break;
	case GB_ATTRIB_VECTOR:
	{
		apply_interpolated_remap<T,U,interp_vector_tag>(
			remap, m_srcElems, m_destElems, m_hSrc, m_hDest);
	}
	break;
	}
}


template<typename T, typename U>
void detail::apply_remapping<T,U>::operator()(
	const GeoAttributeCopier::interpolated_vertex_remap* remap) const
{
	// see specialization in cpp for <GEO_PrimList,GEO_PrimList> case
	throw std::runtime_error("Attempted to apply interpolated vertex remapping "
		"where attrib src, dest or both are not type GEO_PrimList");
}


template<typename T, typename U>
static void detail::attrib_elem_copy(const T& srcElems, U& destElems,
	unsigned int srcIndex, unsigned int destIndex,
	GEO_AttributeHandle& hSrc, GEO_AttributeHandle& hDest,
	const detail::stringTableMapping* strTableMap)
{
	typedef attrib_traits<T>						src_attrib_traits;
	typedef typename src_attrib_traits::elem_type	src_elem_type;
	typedef elem_traits<src_elem_type>				src_elem_traits;

	typedef attrib_traits<U>						dest_attrib_traits;
	typedef typename dest_attrib_traits::elem_type	dest_elem_type;
	typedef elem_traits<dest_elem_type>				dest_elem_traits;

	const src_elem_type* pSrcElem = src_attrib_traits::getElem(srcElems, srcIndex);
	dest_elem_type* pDestElem = dest_attrib_traits::getElem(destElems, destIndex);

	if(pSrcElem && pDestElem)
	{
		if(hDest.getDictionary() == GEO_VERTEX_DICT)
		{
			unsigned int nDestVerts = dest_elem_traits::getVertexCount(*pDestElem);

			if(hSrc.getDictionary() == GEO_VERTEX_DICT)
			{
				// vertex -> vertex is a special case
				unsigned int nSrcVerts = src_elem_traits::getVertexCount(*pSrcElem);
				unsigned int nverts = std::min(nSrcVerts, nDestVerts);

				for(unsigned int i=0; i<nverts; ++i)
				{
					attrib_base_elem_copy(
						*(src_elem_traits::getVertex(*pSrcElem, i)),
						*(dest_elem_traits::getVertex(*pDestElem, i)),
						hSrc, hDest, strTableMap);
				}
			}
			else
			{
				// (non-vertex) -> (vertex) is a special case
				for(unsigned int i=0; i<nDestVerts; ++i)
				{
					attrib_base_elem_copy(
						*pSrcElem, *(dest_elem_traits::getVertex(*pDestElem, i)),
						hSrc, hDest, strTableMap);
				}
			}
		}
		else
		{
			attrib_base_elem_copy(*pSrcElem, *pDestElem, hSrc, hDest, strTableMap);
		}
	}
}


template<typename T, typename U>
void detail::attrib_base_elem_copy(const T& srcElem, U& destElem,
	GEO_AttributeHandle& hSrc, GEO_AttributeHandle& hDest,
	const detail::stringTableMapping* strTableMap)
{
	hDest.setElement(&destElem);

	if(strTableMap && strTableMap->size()>0)
	{
		hSrc.setElement(&srcElem);
		stringTableMapping::const_iterator it = strTableMap->find(hSrc.getI());
		if(it!=strTableMap->end())
			hDest.setI(it->second);
	}
	else
		hDest.copyDataFrom(&srcElem);
}


template<typename T, typename U, typename TypeTag>
void detail::apply_interpolated_remap(
	const GeoAttributeCopier::interpolated_remap* remap,
	const T& srcElems, U& destElems,
	GEO_AttributeHandle& hSrc, GEO_AttributeHandle& hDest,
	const stringTableMapping* strTableMap)
{
	typedef attrib_traits<U> 						dest_attrib_traits;
	typedef typename dest_attrib_traits::elem_type	dest_elem_type;

	unsigned int numDestElems = dest_attrib_traits::entries(destElems);

	for(GeoAttributeCopier::interpolated_remap::const_iterator it=remap->begin();
		it!=remap->end(); ++it)
	{
		unsigned int i = it->first; // dest
		if(i>=numDestElems)
			continue;

		dest_elem_type* pDestElem = dest_attrib_traits::getElem(destElems, i);

		interpolated_elem_copier<TypeTag>::apply(srcElems, *pDestElem,
			it->second, hSrc, hDest, strTableMap);
	}
}


template<typename TypeTag>
void detail::apply_interpolated_vertex_remap(
	const GeoAttributeCopier::interpolated_vertex_remap* remap,
	const GEO_PrimList& srcElems, GEO_PrimList& destElems,
	GEO_AttributeHandle& hSrc, GEO_AttributeHandle& hDest,
	const stringTableMapping* strTableMap)
{
	unsigned int numSrcElems = srcElems.entries();
	unsigned int numDestElems = destElems.entries();

	for(GeoAttributeCopier::interpolated_vertex_remap::const_iterator it=remap->begin();
		it!=remap->end(); ++it)
	{
		unsigned int j = it->second.first; // src
		unsigned int i = it->first; // dest

		if((i>=numDestElems) || (j>=numSrcElems))
			continue;

		const GEO_PrimPoly* srcPoly = dynamic_cast<const GEO_PrimPoly*>(srcElems[j]);
		GEO_PrimPoly* destPoly = dynamic_cast<GEO_PrimPoly*>(destElems[i]);
		if(!srcPoly || !destPoly)
			continue;

		unsigned int nSrcVerts = srcPoly->getVertexCount();
		unsigned int nDestVerts = destPoly->getVertexCount();

		const GeoAttributeCopier::contribs_vector& vert_contribs = it->second.second;
		unsigned int nverts = std::min(nDestVerts, static_cast<unsigned int>(vert_contribs.size()));

		for(unsigned int k=0; k<nverts; ++k)
		{
			const GeoAttributeCopier::contribs_type& contribs = vert_contribs[k];
			GEO_Vertex& vert = destPoly->getVertex(k);

			interpolated_elem_copier<TypeTag>::apply(*srcPoly, vert,
				contribs, hSrc, hDest, strTableMap);
		}
	}
}


template<>
template<typename T, typename U>
void detail::interpolated_elem_copier<detail::interp_float_tag>::apply(
	const T& srcElems, U& destElem,
	const GeoAttributeCopier::contribs_type& contribs,
	GEO_AttributeHandle& hSrc, GEO_AttributeHandle& hDest,
	const stringTableMapping*)
{
	typedef attrib_traits<T>						src_attrib_traits;
	typedef typename src_attrib_traits::elem_type	src_elem_type;

	if(contribs.empty())
		return;

	unsigned int nSrcElems = src_attrib_traits::entries(srcElems);
	unsigned int stride = hSrc.getAttribute()->getSize() / sizeof(float);
	std::vector<float> f(stride, 0.f);

	// calc interpolated value
	for(GeoAttributeCopier::contribs_type::const_iterator it=contribs.begin();
		it!=contribs.end(); ++it)
	{
		if(it->first < nSrcElems)
		{
			const src_elem_type* pSrcElem = src_attrib_traits::getElem(srcElems, it->first);
			hSrc.setElement(pSrcElem);

			for(unsigned int i=0; i<stride; ++i)
				f[i] += hSrc.getF(i) * it->second;
		}
	}

	// set value into dest
	hDest.setElement(&destElem);
	for(unsigned int i=0; i<stride; ++i)
		hDest.setF(f[i], i);
}


template<>
template<typename T, typename U>
void detail::interpolated_elem_copier<detail::interp_vector_tag>::apply(
	const T& srcElems, U& destElem,
	const GeoAttributeCopier::contribs_type& contribs,
	GEO_AttributeHandle& hSrc, GEO_AttributeHandle& hDest,
	const stringTableMapping*)
{
	typedef attrib_traits<T>						src_attrib_traits;
	typedef typename src_attrib_traits::elem_type	src_elem_type;

	if(contribs.empty())
		return;

	unsigned int nSrcElems = src_attrib_traits::entries(srcElems);
	UT_Vector3 v(0.f, 0.f, 0.f);
	float len = 0.f;

	// calc interpolated value
	for(GeoAttributeCopier::contribs_type::const_iterator it=contribs.begin();
		it!=contribs.end(); ++it)
	{
		if(it->first < nSrcElems)
		{
			const src_elem_type* pSrcElem = src_attrib_traits::getElem(srcElems, it->first);
			hSrc.setElement(pSrcElem);

			UT_Vector3 v_ = hSrc.getV3();
			v_ *= it->second;
			len += v_.length();
			v += v_;
		}
	}

	v.normalize();
	v *= len;

	// set value into dest
	hDest.setElement(&destElem);
	hDest.setV3(v);
}


// the type is not interpolatable, so we just take the value with the highest contrib
template<>
template<typename T, typename U>
void detail::interpolated_elem_copier<detail::non_interpolatable_tag>::apply(
	const T& srcElems, U& destElem,
	const GeoAttributeCopier::contribs_type& contribs,
	GEO_AttributeHandle& hSrc, GEO_AttributeHandle& hDest,
	const stringTableMapping* strTableMap)
{
	typedef attrib_traits<T>						src_attrib_traits;
	typedef typename src_attrib_traits::elem_type	src_elem_type;

	if(contribs.empty())
		return;

	const GeoAttributeCopier::contrib_type* contr = &contribs[0];
	for(unsigned int i=1; i<contribs.size(); ++i)
	{
		if(contribs[i].second > contr->second)
			contr = &contribs[i];
	}

	unsigned int srcIndex = contr->first;
	if(srcIndex < src_attrib_traits::entries(srcElems))
	{
		const src_elem_type* pSrcElem = src_attrib_traits::getElem(srcElems, srcIndex);
		attrib_base_elem_copy(*pSrcElem, destElem, hSrc, hDest, strTableMap);
	}
}


template<typename T>
bool GeoAttributeCopier::apply1(const attrib_copy& copy)
{
	switch(copy.m_destDict)
	{
	case GEO_DETAIL_DICT:		return apply2<T,GEO_Detail>(copy);
	case GEO_POINT_DICT:		return apply2<T,GEO_PointList>(copy);
	case GEO_PRIMITIVE_DICT:	return apply2<T,GEO_PrimList>(copy);
	case GEO_VERTEX_DICT:		return apply2<T,GEO_PrimList>(copy);
	default: return false;
	}
}


template<typename T, typename U>
bool GeoAttributeCopier::apply2(const attrib_copy& copy)
{
	GEO_AttributeHandle hSrc, hDest;
	if(!createHandles(copy, hSrc, hDest))
		return false;

	const T& srcElems = detail::attrib_traits<T>::get(*(copy.m_srcGeo));
	U& destElems = detail::attrib_traits<U>::get(m_destGeo);

	detail::stringTableMapping strTableMap;
	detail::getStringTableMapping(hSrc, hDest, strTableMap);

	if(boost::apply_visitor(detail::is_mapped(), copy.m_remapping))
	{
		detail::apply_remapping<T,U> remapper(srcElems, destElems, hSrc, hDest, &strTableMap);
		boost::apply_visitor(remapper, copy.m_remapping);
	}
	else
	{
		unsigned int numSrcElems = detail::attrib_traits<T>::entries(srcElems);
		unsigned int numDestElems = detail::attrib_traits<U>::entries(destElems);
		unsigned int nelems = std::min(numSrcElems, numDestElems);

		for(unsigned int i=0; i<nelems; ++i)
			detail::attrib_elem_copy(srcElems, destElems, i, i, hSrc, hDest, &strTableMap);
	}
}

}

#endif



/***
    Copyright 2008-2012 Dr D Studios Pty Limited (ACN 127 184 954) (Dr. D Studios)

    This file is part of vfxgal.

    vfxgal is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    vfxgal is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with vfxgal.  If not, see <http://www.gnu.org/licenses/>.
***/
