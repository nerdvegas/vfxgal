#include "GeoAttributeCopier.h"
//#include "GeoUtils.h"
#include <GEO/GEO_PrimPoly.h>
#include <GB/GB_AttributeDefines.h>


namespace vfxgal_hou {

GeoAttributeCopier::GeoAttributeCopier(GU_Detail& destGeo)
:	m_destGeo(destGeo)
{}


// -------------------------

void GeoAttributeCopier::add(const GU_Detail& srcGeo, const std::string& srcAttribName,
	GEO_AttributeOwner srcDict, const std::string& destAttribName, GEO_AttributeOwner destDict,
	const remapping_type& remapping, compMethod comp)
{
	attrib_id id(destAttribName, destDict);
	attrib_copy_map::iterator it = m_copies.find(id);

	attrib_copy c;
	c.m_srcGeo = &srcGeo;
	c.m_srcName = srcAttribName;
	c.m_destName = destAttribName;
	c.m_srcDict = srcDict;
	c.m_destDict = destDict;
	c.m_remapping = remapping;

	switch (comp)
	{
		case GAC_REPLACE:
			m_copies[id].push_back(c);
			break;

		case GAC_SKIP:
			if (it == m_copies.end())
				m_copies[id].push_back(c);
			break;

		case GAC_MERGE:
			m_copies[id].push_back(c);
			break;

		default:
			break;
	}
}


void GeoAttributeCopier::add(const GU_Detail& srcGeo, GEO_AttributeOwner srcDict,
	GEO_AttributeOwner destDict, const remapping_type& remapping, bool skipP, compMethod comp)
{

	GEO_AttributeHandleList hlist;
	hlist.bindDetail(&srcGeo);
	hlist.appendAllAttributes(srcDict);

	for(unsigned int i=0; i<hlist.entries(); ++i)
	{
		const GEO_AttributeHandle* h = hlist[i];
		std::string name(h->getName());
		if(!skipP || (name != "P"))
			add(srcGeo, name.c_str(), srcDict, name.c_str(), destDict, remapping, comp);
	}
}


void GeoAttributeCopier::add(const GU_Detail& srcGeo, GEO_AttributeOwner srcDict,
	GEO_AttributeOwner destDict, GB_AttribType type,
	const remapping_type& remapping, bool skipP, compMethod comp)
{
	GEO_AttributeHandleList hlist;
	hlist.bindDetail(&srcGeo);
	hlist.appendAllAttributes(srcDict);

	for(unsigned int i=0; i<hlist.entries(); ++i)
	{
		const GEO_AttributeHandle* h = hlist[i];
		if(h->getAttribute() && (h->getAttribute()->getType() == type))
		{
			std::string name(h->getName());
			if(!skipP || (name != "P"))
				add(srcGeo, name.c_str(), srcDict, name.c_str(), destDict, remapping, comp);
		}
	}
}


bool GeoAttributeCopier::remove(const std::string& destAttribName, GEO_AttributeOwner destDict)
{
	attrib_id id(destAttribName, destDict);
	attrib_copy_map::iterator it2 = m_copies.find(id);
	if(it2 != m_copies.end())
	{
		m_copies.erase(it2);
		return true;
	}
	return false;
}


void GeoAttributeCopier::remove(const GeoAttributeCopier& c)
{
	for(attrib_copy_map::const_iterator it=c.m_copies.begin(); it!=c.m_copies.end(); ++it)
		remove(it->first.first, it->first.second);
}


bool GeoAttributeCopier::createHandles(const attrib_copy& copy, GEO_AttributeHandle& hSrc,
	GEO_AttributeHandle& hDest)
{
	// find source attrib
	hSrc = copy.m_srcGeo->getAttribute(copy.m_srcDict, copy.m_srcName.c_str());
	if(!hSrc.isAttributeValid())
		return false;

	const char* destName_ = copy.m_destName.c_str();
	const GB_Attribute* srcAttrib = hSrc.getAttribute();
	assert(srcAttrib);

	// create attrib if it doesn't exist, or does but data type doesn't match
	bool createAttrib = true;
	GEO_AttributeHandle hExist = m_destGeo.getAttribute(copy.m_destDict, destName_);
	if(hExist.isAttributeValid())
	{
		const GB_Attribute* destAttrib = hExist.getAttribute();
		assert(destAttrib);
		createAttrib = (destAttrib->getType() != srcAttrib->getType());
	}

	if(createAttrib)
	{
		GB_AttributeRef aref = m_destGeo.addAttribute(destName_, srcAttrib->getSize(),
			srcAttrib->getType(), srcAttrib->getDefault(), copy.m_destDict);

		if(!aref.isValid())
			return false;
	}

	hDest = m_destGeo.getAttribute(copy.m_destDict, destName_);
	hDest.setSourceMap(hSrc);
	return true;
}


// todo try threading this
void GeoAttributeCopier::apply(bool createOnly)
{
	for(attrib_copy_map::const_iterator it=m_copies.begin(); it!=m_copies.end(); ++it)
	{
		if(createOnly)
		{
			GEO_AttributeHandle hSrc, hDest;
			// since all attrib_copy in the vector are all applied onto same dest attr, so here just use the first one.
			createHandles(it->second[0], hSrc, hDest);
		}
		else
			for(vector<attrib_copy>::const_iterator cpit=it->second.begin(); cpit!=it->second.end(); ++cpit)
				apply(*cpit);
	}
}


bool GeoAttributeCopier::apply(const attrib_copy& copy)
{
	switch(copy.m_srcDict)
	{
	case GEO_DETAIL_DICT:		return apply1<GEO_Detail>(copy);
	case GEO_POINT_DICT:		return apply1<GEO_PointList>(copy);
	case GEO_PRIMITIVE_DICT:	return apply1<GEO_PrimList>(copy);
	case GEO_VERTEX_DICT:
	{
		// vertex->!vertex copies are not supported
		if(copy.m_destDict != GEO_VERTEX_DICT)
			return false;

		return apply2<GEO_PrimList,GEO_PrimList>(copy);
	}
	break;
	default: return false;
	}
}


void detail::getStringTableMapping(GEO_AttributeHandle &srcAttrH, GEO_AttributeHandle &destAttrH, detail::stringTableMapping& o_stm)
{
	o_stm.clear();

	const GB_Attribute* srcAttr = srcAttrH.getAttribute();
	GB_Attribute* destAttr = destAttrH.getAttribute();

	if( srcAttr->getType() == GB_ATTRIB_INDEX && destAttr->getType() == GB_ATTRIB_INDEX)
	{
		for( int i=0; i<srcAttr->getIndexSize(); ++i)
		{
			int destIndex = destAttr->addIndex(srcAttr->getIndex(i));
			o_stm.insert(stringTableMapping::value_type(i, destIndex));
		}
	}

}


template<>
void detail::apply_remapping<GEO_PrimList,GEO_PrimList>::operator()(
	const GeoAttributeCopier::interpolated_vertex_remap* remap) const
{
	if(	(m_hSrc.getDictionary() != GEO_VERTEX_DICT) ||
		(m_hDest.getDictionary() != GEO_VERTEX_DICT) )
	{
		throw std::runtime_error("Attempted to apply interpolated vertex remapping "
			"where attrib src, dest or both are not dictionary type GEO_VERTEX_DICT");
	}

	GB_AttribType t = m_hSrc.getAttribute()->getType();
	switch(t)
	{
	case GB_ATTRIB_INT:
	case GB_ATTRIB_INDEX:
	case GB_ATTRIB_MIXED:
	{
		apply_interpolated_vertex_remap<non_interpolatable_tag>(
			remap, m_srcElems, m_destElems, m_hSrc, m_hDest, m_strTableMap);
	}
	break;
	case GB_ATTRIB_FLOAT:
	{
		apply_interpolated_vertex_remap<interp_float_tag>(
			remap, m_srcElems, m_destElems, m_hSrc, m_hDest);
	}
	break;
	case GB_ATTRIB_VECTOR:
	{
		apply_interpolated_vertex_remap<interp_vector_tag>(
			remap, m_srcElems, m_destElems, m_hSrc, m_hDest);
	}
	break;
	}
}

}























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
