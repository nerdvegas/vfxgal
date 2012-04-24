#ifndef _DGAL_CELL__H_
#define _DGAL_CELL__H_

#include <OpenEXR/ImathVec.h>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <assert.h>
#include <OpenEXR/ImathBox.h>
#include "../exceptions.h"
#include "../plane.hpp"

namespace dgal {

	/*
	 * @class Cell
	 * @brief
	 */

	// forward declarations
	template< typename DataSet > class Octree;

	////
	class Cell
		{
		public :

			// ---- Ctor // Dtor ----
			Cell(int level=0);
			~Cell();


			// ---- Data ----
			// add the vert index to the cell vertex list
			inline void									addData(unsigned d) { _aDataIndex.insert(std::pair<unsigned,unsigned>(d,d)); }
			inline const boost::unordered_map<unsigned, unsigned >& getData() const { return _aDataIndex;}
			inline void									clearData() { _aDataIndex.clear(); }

			// box query
			template<typename DataSet>
			void 										queryDataRecurse(	const typename Imath::Box<typename DataSet::point_type>& qBox,
																			const Octree< DataSet >& oct, std::vector< boost::unordered_set<unsigned> >& outRes ) const;
			template<typename DataSet>
			void 										intersectRecurse(	const typename Imath::Box<typename DataSet::point_type>& qBox,
																			const Octree< DataSet >& oct, bool& outRes ) const;
			// plane query
			template<typename DataSet>
			void 										queryDataRecurse(	const Imath::Plane3<typename DataSet::T>&  qPlane,
																			const Octree< DataSet >& oct,
																			std::vector< boost::unordered_set<unsigned> >* outLeft,
																			std::vector< boost::unordered_set<unsigned> >* outRight,
																			std::vector< boost::unordered_set<unsigned> >* outIntersect, typename DataSet::T epsilon  ) const;

			// axis aligned ray query
			template<typename DataSet>
			void 										queryDataRecurse(	const typename DataSet::point_type&  pStart,
																			short axis, bool castForward,
																			const Octree< DataSet >& oct,
																			std::vector< boost::unordered_set<unsigned> >& outRes) const;


			// ---- Hierachy // traversal ----
			// create a new children if NULL else return the existing one
			unsigned			level() const { return _level;}
			Cell*				getChild(int i) const { assert(i>=0 && i<8); return _aChild[i];}
			template<typename DataSet>
			void				getNeighbors(std::vector<Cell*>& aOutNeighborCell,const Octree< DataSet >& oct) const;

			bool 				hasChild() const;
			template<typename DataSet>
			Cell* 				createChild(unsigned i,const Octree< DataSet >& oct);
			void				getLocCode(unsigned& outLocX, unsigned& outLocY, unsigned& outLocZ) const { outLocX = _xLocCode; outLocY = _yLocCode; outLocZ = _zLocCode;}
			int					computeDescendantCountRecurse();
			// ---- Debug  ----
			void 				getStatRecurse(unsigned&  nMinElemByCell,unsigned& nMaxElemByCell) const;


		private:
			template<typename DataSet>
			inline bool 		AARayBoxTest(	const typename DataSet::point_type&  pStart,
												const Imath::Box<typename DataSet::point_type>& box,
												short axis,bool castForward) const;
			template<typename DataSet>
			inline int 			planeBoxTest(const Imath::Plane3<typename DataSet::T>& qPlane, const Imath::Box<typename DataSet::point_type>& box, typename DataSet::T epsilon ) const;

			inline Cell*				getCommonAncestor(unsigned binaryDiff) const;
			template<typename DataSet>
			inline Cell*				getNeighbor(const typename DataSet::point_type& Direction,const Octree< DataSet >& oct) const;

		private :

			unsigned 					_xLocCode;
			unsigned 					_yLocCode;
			unsigned 					_zLocCode;
			unsigned 					_level;
			Cell*		 				_parent;
			Cell*						_aChild[8];

			//Cell content :  Octree Data index
			int									_descendantElemCount; // num of element below this node
			boost::unordered_map<unsigned,unsigned> 		_aDataIndex;
		};

//// Implementation
Cell::Cell(int level) :
_level(level),
_descendantElemCount(0)
{
	memset(_aChild,0,sizeof(Cell*)*8);
	_parent = NULL;
	_xLocCode = 0;
	_yLocCode = 0;
	_zLocCode = 0;
}
//-------------------------------------------------------------------------------------------------
Cell::~Cell()
{
	for(uint i=0;i<8;i++)
	{
		if (_aChild[i]!=NULL)
		{
			delete _aChild[i];
		}
	}
}
//-------------------------------------------------------------------------------------------------
template<typename DataSet>
Cell* Cell::createChild(unsigned i,const Octree<DataSet>& oct)
{
	//convenient typededf - octree centric -
	typedef Octree<DataSet>		 				octree_type;
	typedef typename octree_type::point_type 	point_type;
	typedef typename octree_type::T 		  	scalar;
	typedef typename octree_type::cube_type 	cube_type;
	//
	 assert(i>=0 && i<8);
	 if (_aChild[i] != 0)
		 return _aChild[i];
	 else
	 {
		Cell* newChild = new Cell;
		cube_type c;
		bool bTransform = false;
		oct.getCube(this,bTransform,c);
		//Cell minPos & width.
		const point_type& 	pMin = c.min();
		scalar 				cellW = c.size();

		point_type pNewMin = pMin+ octree_type::_aParentToChildtranslate[i]*cellW;

		newChild->_xLocCode = (unsigned) (pNewMin.x* oct._twoPowMaxLevel);
		newChild->_yLocCode = (unsigned) (pNewMin.y* oct._twoPowMaxLevel);
		newChild->_zLocCode = (unsigned) (pNewMin.z* oct._twoPowMaxLevel);
		newChild->_parent = this;
		newChild->_level = _level-1;
		assert(newChild->_level>=0);

		assert( i == oct.childIndex(newChild->_level,newChild->_xLocCode,newChild->_yLocCode,newChild->_zLocCode));

		_aChild[i] = newChild;
		return newChild;
	 }
}

//-------------------------------------------------------------------------------------------------
bool Cell::hasChild() const
{
	return ( _aChild[0]!=NULL || _aChild[1]!=NULL ||
			 _aChild[2]!=NULL || _aChild[3]!=NULL ||
			 _aChild[4]!=NULL || _aChild[5]!=NULL ||
			 _aChild[6]!=NULL || _aChild[7]!=NULL );
}


//-------------------------------------------------------------------------------------------------
void Cell::getStatRecurse(unsigned& nMinElemByCell,unsigned& nMaxElemByCell) const
{
	unsigned nData = _aDataIndex.size();
	nMaxElemByCell = (nMaxElemByCell>nData)? nMaxElemByCell : nData;
	nMinElemByCell = (nMinElemByCell<nData || nData ==0 )? nMinElemByCell : nData;
	for (unsigned i=0;i<8;i++)
	{
		if (_aChild[i]!=NULL)
		{
			_aChild[i]->getStatRecurse(nMinElemByCell,nMaxElemByCell);
		}
	}
}

//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void Cell::queryDataRecurse(const typename Imath::Box<typename DataSet::point_type>& qBox,
							const Octree< DataSet >& oct,
							std::vector< boost::unordered_set<unsigned> >& outRes ) const
{
	typename Octree< DataSet >::cube_type c;
	oct.getCube(this,false, c);
	if (!c.intersect(qBox))
	{
		return;
	}

	if (_aDataIndex.size() !=0)
	{
			boost::unordered_map<unsigned,unsigned>::const_iterator cit;
			for (cit =_aDataIndex.begin(); cit!= _aDataIndex.end();cit++)
			{
				unsigned currIndex = (*cit).first;
				assert(currIndex< oct._aCellData.size());
				const std::pair<char,unsigned>& data = oct._aCellData[currIndex];

				Imath::Box<typename DataSet::point_type> box;
				assert(data.first < oct._aDataSet.size());
				oct._aDataSet[data.first]->getBounds(data.second,box);
				if (qBox.intersects(box)) // test the intersection btw query box and element box
				{
					assert(data.first < outRes.size());
					outRes[data.first].insert(data.second);
				}
			}
	}
	//Recurse
	for (unsigned i=0;i<8;i++)
	{
		if (_aChild[i]!=NULL)
		{
			_aChild[i]->queryDataRecurse(qBox,oct,outRes);
		}
	}
}
//-------------------------------------------------------------------------------------------------
template<typename DataSet>
int Cell::planeBoxTest(	const Imath::Plane3<typename DataSet::T>& qPlane,
						const Imath::Box<typename DataSet::point_type>& box,
						typename DataSet::T epsilon) const
{
	typename DataSet::point_type bSize = box.size();
	int nLocation = 		locatePoint( qPlane, box.min,epsilon )
								+	locatePoint( qPlane, box.min + typename DataSet::point_type(0,bSize.y,0), epsilon)
								+	locatePoint( qPlane, box.min + typename DataSet::point_type(0,0,bSize.z),epsilon )
								+ 	locatePoint( qPlane, box.min + typename DataSet::point_type(0,bSize.y ,bSize.z),epsilon)
								+ 	locatePoint( qPlane, box.min + typename DataSet::point_type(bSize.x,0 ,0),epsilon)
								+	locatePoint( qPlane, box.min + typename DataSet::point_type(bSize.x,bSize.y,0),epsilon )
								+	locatePoint( qPlane, box.min + typename DataSet::point_type(bSize.x,0,bSize.z),epsilon )
								+	locatePoint( qPlane, box.min + typename DataSet::point_type(bSize.x ,bSize.y ,bSize.z),epsilon);

	return (nLocation==-8) ? -1 : ( (nLocation == 8)?  1 : 0);
}

//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void Cell::queryDataRecurse(	const Imath::Plane3<typename DataSet::T>& qPlane,
								const Octree< DataSet >& oct,
								std::vector< boost::unordered_set<unsigned> >* outLeft,
								std::vector< boost::unordered_set<unsigned> >* outRight,
								std::vector< boost::unordered_set<unsigned> >* outIntersect ,typename DataSet::T epsilon) const
{
	//
	typename DataSet::point_type vEpsilon(epsilon,epsilon,epsilon);
	//
	if (_aDataIndex.size() !=0)
	{
		typename Octree< DataSet >::cube_type c;
		oct.getCube(this,false, c);

		int intersectionTest = c.intersect(qPlane,epsilon);

		if (intersectionTest==0  ) //test intersection between the leaf cell and the query box
		{
			boost::unordered_map<unsigned,unsigned>::const_iterator cit;
			for (cit =_aDataIndex.begin(); cit!= _aDataIndex.end(); cit++)
			{
				unsigned currIndex = (*cit).first;
				assert(currIndex< oct._aCellData.size());
				const std::pair<char,unsigned>& data = oct._aCellData[currIndex];

				Imath::Box<typename DataSet::point_type> box;
				assert(data.first < oct._aDataSet.size());
				oct._aDataSet[data.first]->getBounds(data.second,box);

				//test plane /box intersection (1 inside // 0 intersect //-1 outside)
				int intersect= planeBoxTest<DataSet>(qPlane,box,epsilon);

				//
				if ( intersect == 0 && outIntersect != NULL)
				{

					assert(data.first < outIntersect->size());
					(*outIntersect)[data.first].insert(data.second);
				}
				else
				{
					if (intersect >0 && outRight !=NULL)
					{
						assert(data.first < outRight->size());
						(*outRight)[data.first].insert(data.second);
					}
					else if (intersect <0 && outLeft !=NULL)
					{
						assert(data.first < outLeft->size());
						(*outLeft)[data.first].insert(data.second);
					}
				}

			}
		}
		else
		{

			if (intersectionTest>0 && outRight!=NULL)

			{
				boost::unordered_map<unsigned,unsigned>::const_iterator cit;
				for (cit =_aDataIndex.begin(); cit!= _aDataIndex.end(); cit++)
				{
					unsigned currIndex = (*cit).first;
					assert(currIndex< oct._aCellData.size());
					const std::pair<char,unsigned>& data = oct._aCellData[currIndex];

					Imath::Box<typename DataSet::point_type> box;
					assert(data.first < oct._aDataSet.size());
					oct._aDataSet[data.first]->getBounds(data.second,box);
					if ( c.contains(box) )
					{
						assert(data.first < outRight->size());
						(*outRight)[data.first].insert(data.second);
					}
					else
					{
						int intersect= planeBoxTest<DataSet>(qPlane,box,epsilon);
						assert( !(intersect<0));
						if (intersect>0)
						{
							assert(data.first < outRight->size());
							(*outRight)[data.first].insert(data.second);
						}
						else
						{
							if (outIntersect !=NULL)
							{
								assert(data.first < outIntersect->size());
								(*outIntersect)[data.first].insert(data.second);
							}
						}

					}
				}
			}
			else if (intersectionTest<0 && outLeft!=NULL)
			{
				boost::unordered_map<unsigned,unsigned>::const_iterator cit;
				for (cit =_aDataIndex.begin(); cit!= _aDataIndex.end(); cit++)
				{
					unsigned currIndex = (*cit).first;
					assert(currIndex< oct._aCellData.size());
					const std::pair<char,unsigned>& data = oct._aCellData[currIndex];


					Imath::Box<typename DataSet::point_type> box;
					assert(data.first < oct._aDataSet.size());
					oct._aDataSet[data.first]->getBounds(data.second,box);
					box.min+=vEpsilon; 	box.max+=vEpsilon;

					if ( c.contains(box) )
					{
						assert(data.first < outLeft->size());
						(*outLeft)[data.first].insert(data.second);
					}
					else
					{
						int intersect= planeBoxTest<DataSet>(qPlane,box, epsilon);
						assert( !(intersect>0));
						if (intersect<0)
						{
							assert(data.first < outLeft->size());
							(*outLeft)[data.first].insert(data.second);
						}
						else
						{
							if (outIntersect !=NULL)
							{
								assert(data.first < outIntersect->size());
								(*outIntersect)[data.first].insert(data.second);
							}
						}
					}

				}
			}
		}
	}
	//Recurse

	for (unsigned i=0;i<8;i++)
	{
		if (_aChild[i]!=NULL)
		{
			_aChild[i]->queryDataRecurse(qPlane, oct, outLeft, outRight, outIntersect,epsilon);
		}
	}

}
//-------------------------------------------------------------------------------------------------
template<typename DataSet>
bool Cell::AARayBoxTest(	const typename DataSet::point_type&  ptStart,
							const Imath::Box<typename DataSet::point_type>& box,
							short axis,bool castForward) const
{
	assert(axis<3);
	short ax0 = (axis+1)>2 ? 0 :axis+1;
	short ax1 = (axis-1)<0 ? 2 :axis-1;

	if (castForward)
	{
		if ( ptStart[axis]>box.max[axis] )
			return false;
	}
	else
	{
		if ( ptStart[axis]<box.min[axis] )
			return false;
	}
	return !( 	ptStart[ax0]<box.min[ax0] || ptStart[ax0]>box.max[ax0] ||
				ptStart[ax1]<box.min[ax1] || ptStart[ax1]>box.max[ax1] );
}
//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void Cell::queryDataRecurse(	const typename DataSet::point_type&  pStart,
								short axis, bool castForward,
								const Octree< DataSet >& oct,
								std::vector< boost::unordered_set<unsigned> >& outRes ) const
{
	typename Octree< DataSet >::cube_type c;
	oct.getCube(this,false, c);

	bool intersectionTest = c.intersect(pStart,axis,castForward);
	if (!intersectionTest  ) //test intersection between the leaf cell and the query box
	{
		return;
	}

	if (_aDataIndex.size() !=0)
	{
		boost::unordered_map<unsigned,unsigned>::const_iterator cit;
		for (cit =_aDataIndex.begin(); cit!= _aDataIndex.end(); cit++)
		{
			unsigned currIndex = (*cit).first;
			assert(currIndex< oct._aCellData.size());
			const std::pair<char,unsigned>& data = oct._aCellData[currIndex];

			Imath::Box<typename DataSet::point_type> box;
			assert(data.first < oct._aDataSet.size());
			oct._aDataSet[data.first]->getBounds(data.second,box);

			//test plane /box intersection (1 inside // 0 intersect //-1 outside)
			bool intersect= AARayBoxTest<DataSet>(pStart,box,axis,castForward);
			if ( intersect )
			{
				outRes[data.first].insert(data.second);
			}
		}
	}
	//Recurse
	for (unsigned i=0;i<8;i++)
	{
		if (_aChild[i]!=NULL)
		{
			_aChild[i]->queryDataRecurse( pStart, axis,castForward, oct, outRes );
		}
	}
}


//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void Cell::intersectRecurse(	const typename Imath::Box<typename DataSet::point_type>& qBox,
								const Octree< DataSet >& oct ,bool& outRes  ) const
{
	typename Octree< DataSet >::cube_type c;
	oct.getCube(this,false, c);
	if (!c.intersect(qBox)) //test intersection between the leaf cell and the query box
	{
		return;
	}
	if (_aDataIndex.size() !=0)
	{
		boost::unordered_map<unsigned,unsigned>::const_iterator cit;
		for (cit =_aDataIndex.begin(); cit!= _aDataIndex.end();cit++)
		{
			unsigned currIndex = (*cit).first;
			assert(currIndex< oct._aCellData.size());
			const std::pair<char,unsigned>& data = oct._aCellData[currIndex];

			Imath::Box<typename DataSet::point_type> box;
			assert(data.first < oct._aDataSet.size());
			oct._aDataSet[data.first]->getBounds(data.second,box);
			if (qBox.intersects(box)) // test the intersection btw query box and element box
			{
				outRes = true;
				return;
			}
		}
	}
	//Recurse
	for (unsigned i=0;i<8;i++)
	{
		if (_aChild[i]!=NULL)
		{
			_aChild[i]->intersectRecurse(qBox,oct,outRes);
		}
	}

}
//-------------------------------------------------------------------------------------------------
Cell* Cell::getCommonAncestor(unsigned binaryDiff) const
{
	Cell* pCell = const_cast<Cell*>(this);
	while ( binaryDiff & (1 << pCell->_level))
	{
		pCell = pCell->_parent;
	}
	return pCell;
}
//-------------------------------------------------------------------------------------------------
template<typename DataSet>
Cell* Cell::getNeighbor(const typename DataSet::point_type& direction,const Octree< DataSet >& oct) const
{

	unsigned n_LocCode[3];
	unsigned locCode[3] = { _xLocCode, _yLocCode, _zLocCode};

	unsigned int binaryCellSize = 1 << _level;
	Cell*	 commonAncestor[3] = { this, this, this};

	for (unsigned i=0; i< 3;i++)
	{
		switch( (int)direction[i] )
		{
			case -1 :
				//----No left neighbor if this is the left side of the Octree
				if (locCode[i] == 0) return NULL;
				n_LocCode[i] =  locCode[i] - 0x00000001;
				break;
			case 1 :
				//----No right neighbor if this is the right side of the Octree
				if ((locCode[i] + binaryCellSize) >= (unsigned)(1 << (oct.GetDepth()-1) )) return NULL;
				n_LocCode[i] = locCode[i] + binaryCellSize;
				break;
			case 0 :
				n_LocCode[i] =  locCode[i];
				break;
			default :
				throw DgalError("Cell::getNeighbor: Invalid direction");
				break;
		}

		//
		if (direction[i] != 0)
		{
			//----Determine the smallest common ancestor of the cell and the cell's
			//----smallest possible left neighbor
			unsigned int diff = locCode[i] ^ n_LocCode[i];
			commonAncestor[i] = getCommonAncestor(diff);

		}
	}

	Cell* resCell = commonAncestor[0]->_level > commonAncestor[1]->_level ? commonAncestor[0] : commonAncestor[1];
	resCell = resCell->_level > commonAncestor[2]->_level ? resCell : commonAncestor[2];
	//----Start from the smallest common ancestor and follow the branching
	//----patterns of the locational codes downward to the smallest left
	//----neighbor of size greater than or equal to cell

	unsigned nextLevel = resCell->_level-1;
	oct.traverse(resCell, nextLevel, n_LocCode[0], n_LocCode[1], n_LocCode[2]);
	return(resCell);

}


//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void Cell::getNeighbors(std::vector<Cell*>& aOutNeighborCell,const Octree< DataSet >&  oct) const
{
	typedef typename DataSet::point_type point_type;

	//Add face neighbors
	aOutNeighborCell.push_back(getNeighbor(point_type(1,0,0),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(-1,0,0),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(0,1,0),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(0,-1,0),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(0,0,1),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(0,0,-1),oct));

	//Add edges neighbor
	aOutNeighborCell.push_back(getNeighbor(point_type(-1,0,-1),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(-1,0,1),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(-1,-1,0),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(-1,1,0),oct));

	aOutNeighborCell.push_back(getNeighbor(point_type(1,0,-1),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(1,0,1),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(1,-1,0),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(1,1,0),oct));

	aOutNeighborCell.push_back(getNeighbor(point_type(0,-1,-1),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(0,-1,1),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(0,1,-1),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(0,1,1),oct));

	//Add verts neighbor
	aOutNeighborCell.push_back(getNeighbor(point_type(1,-1,-1),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(1, 1,-1),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(1,-1,1),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(1,1,1),oct));

	aOutNeighborCell.push_back(getNeighbor(point_type(-1,1,1),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(-1,-1,-1),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(-1,1,-1),oct));
	aOutNeighborCell.push_back(getNeighbor(point_type(-1,-1,1),oct));

}

//-------------------------------------------------------------------------------------------------
int Cell::computeDescendantCountRecurse()
{
	//
	_descendantElemCount = 0;
	bool bHasChild = false;
	for (unsigned i=0;i<8;i++)
	{

		if (_aChild[i]!=NULL)
		{
			bHasChild = true;
			_descendantElemCount += _aChild[i]->computeDescendantCountRecurse();
		}
	}
	// Cell is it a leaf ?
	int nElem = _aDataIndex.size();
	if (bHasChild)
	{
		assert(nElem==0);
	}
	else
	{
		_descendantElemCount+= nElem;
	}
	//
	return _descendantElemCount;
}

}
#endif // _DGAL_CELL__H_

//=============================================================================
//	CODE ENDS HERE
//=============================================================================


/***
    Copyright 2008-2012 Dr D Studios Pty Limited (ACN 127 184 954) (Dr. D Studios)

    This file is part of anim-studio-tools.

    anim-studio-tools is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    anim-studio-tools is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with anim-studio-tools.  If not, see <http://www.gnu.org/licenses/>.
***/
