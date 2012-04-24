#ifndef _DGAL_OCTREE__H_
#define _DGAL_OCTREE__H_

#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <OpenEXR/ImathVec.h>
#include "cell.hpp"
#include "cube.hpp"
#include "limits.h"
#include <set>

namespace dgal {
	/*
	 * @class Octree
	 * @brief
	 * acceleration structure for spatial queries on generic dataSet..
	 */
	template<typename DataSet>
	class Octree
		{
		friend class Cell;
		public :

			typedef DataSet							dataset_type;
			typedef typename DataSet::T				T;
			typedef typename DataSet::point_type	point_type;
			typedef typename DataSet::index_type	index_type;
			typedef typename DataSet::cube_type		cube_type;

			// Construct & populate
			Octree(const DataSet& ds,int maxNumElemByLeaf,const boost::unordered_set<unsigned>* components=NULL);

			//add a dataset to this octree
			void add(const DataSet& ds, const boost::unordered_set<unsigned>* components=NULL );

			// datasets accessor
			const std::vector<const DataSet*>& datasets() const { return _aDataSet; }

			~Octree();

			// Query : Fill outResult with the index of each dataSet element intersecting qBox
			// outResult has the size of the number of dataSet in the octree;
			void query(	const Imath::Box<point_type>& qBox,
						std::vector< boost::unordered_set<unsigned> >& outResult,
						T epsilon = 1e-4) const;

			// intersect return true if an element intersect the bbox.
			bool intersect(	const Imath::Box<point_type>& qBox,
							T epsilon = 1e-4,
							std::vector<cube_type>* aC = NULL) const;

			// Query : plane left(d<0) side elements right side(d>0) elements and intersected elements (d==0).
			// outIntersect has the size of the number of dataSet in the octree;
			void query(	const Imath::Plane3<T>& plane,
						std::vector< boost::unordered_set<unsigned> >* outLeft = NULL,
						std::vector< boost::unordered_set<unsigned> >* outRight =NULL,
						std::vector< boost::unordered_set<unsigned> >* outIntersect =NULL, T epsilon = 1e-4) const;


			// Axis-aligned ray Query : Fill outResult with the index of each dataSet element intersecting qBox
			// outResult has the size of the number of dataSet in the octree;
			void query(	const point_type& pt,
						short axis,bool castForward,
						std::vector< boost::unordered_set<unsigned> >& outResult,
						T epsilon = 1e-4) const;



			//---- octree transform ----
			inline void					getAffineTransform(cube_type& outTransform);
			// stat / debug
			// get a geometric representation of this octree for a given level.
			void getCubes(bool bTransform, unsigned level, std::vector<cube_type>& outCube ) const;
			// collect element distribution infos
			void getStat(unsigned& nMaxElemByCell,unsigned& nMinElemByCell) const;


		private :
			//
			// apply the octree affine transform to this point to set each coordinate in the range [0,1]
			inline point_type 			transformTo_01( const point_type& v ) const;
			// apply the octree affine transform to map [0,1] to the Bounding Cube
			inline point_type 			transformFrom_01( const point_type& v ) const;
			// clip abbox against the octree return false if the bb becomes tiny
			bool 						clipBox(const Imath::Box<point_type>& box,Imath::Box<point_type>& oClipedBox,T epsilon) const;
			//test if the pt is inside the octree bounds.
			bool 						contains(const point_type& pt) const;
			// ---- Location code  ----
			// Compute the location code for a given position // the vertex coords are first transformed to [0,1]
			inline void 				opCodeFromPos(  const point_type& v, unsigned& outXLocCode, unsigned& outYLocCode, unsigned& outZLocCode) const;
			// Compute the location code for a given position // v should lie in the cube [0,1]x[0,1]x[0,1]
			inline void 				opCodeFromPos_01(  const point_type& v, unsigned& outXLocCode, unsigned& outYLocCode, unsigned& outZLocCode) const;
			//  traverse from "cell" to the specified level minLevel
			 void 						traverse(Cell*& cell,unsigned& nextLevel,unsigned xLocCode,unsigned yLocCode,unsigned zLocCode,unsigned minLevel) const;
			// Return the index of the child node containing this location code
			unsigned 					childIndex(unsigned level,unsigned xLocCode,unsigned yLocCode,unsigned zLocCode) const;
			// Locate the smallest cell that entirely contains a rectangular region
			Cell* 						locateRegion (Cell * root, const Imath::Box<point_type>& box) const;

			// ---- Insertion  ---
			// insertData, start to insert at insertionLevel then use the nMaxNumElemByLeaf criterion for further subdivision
			void 						insertVolumeData(Cell* pCell,const std::pair<char,unsigned> & data,unsigned dataIndex,  unsigned insertionLevel, unsigned nMaxNumElemByLeaf);
			void 						insertPointData(Cell* pCell,const std::pair<char,unsigned> & data,unsigned dataIndex,  unsigned insertionLevel, unsigned nMaxNumElemByLeaf);
			// update each Cell _descendantElemCount
			void						computeDescendantCount();

			// ---- cube ----
			// get the cube for the cell pCell (apply octree affine transform if bTransform == true)
			void 						getCube(const Cell* pCell,bool bTransform, cube_type& outCube) const;
			// get recursively all cubes from each cells of the specified level
			void 						getCubes_recurse(const Cell* pCell, bool bTransform, unsigned level, std::vector<cube_type>& outCube) const;
			//transform a cube in a list of childCubes
			void 						getChildCubes(const cube_type& cub, cube_type* aChildCubes) const ;
			//inline int addVertex(const point_type& v, unsigned nNumVertexbyLeaf );

		public :
			double 									_twoPowMaxLevel; //cached value computed from depth (extensive use by loc code )
		private :
			static double 							_cellEpsilon;
			std::vector<const DataSet*>				_aDataSet;
			std::vector<std::pair<char,unsigned> >  _aCellData; //each cell index refer to an element of this vector
			static point_type  						_aParentToChildtranslate[8];

			Cell* 									_pRoot;
			//Affine transform for the octree
			point_type 								_rootTranslate;
			T 										_rootScale;
			//depth
			unsigned 								_depth;
			int 									_maxNumElemByLeaf;
		};


//// Implementation

template< typename DataSet >
double Octree< DataSet >::_cellEpsilon = 1e-4f;


template< typename DataSet >
typename Octree< DataSet >::point_type Octree< DataSet >::_aParentToChildtranslate[8]=					{ 	point_type(0.0f,0.0f,0.0f), point_type(0.5f,0.0f,0.0f),
																											point_type(0.0f,0.5f,0.0f), point_type(0.5f,0.5f,0.0f),
																											point_type(0.0f,0.0f,0.5f), point_type(0.5f,0.0f,0.5f),
																										point_type(0.0f,0.5f,0.5f), point_type(0.5f,0.5f,0.5f)   };

//-------------------------------------------------------------------------------------------------
template< typename DataSet >
void Octree<DataSet>::add(const DataSet& ds,const boost::unordered_set<unsigned>* components )
{
	//
	int dataSetIndex = _aDataSet.size();
	//
	_aDataSet.push_back(&ds);
	assert(_aDataSet.size()>0);
	cube_type  octreeCube;
	_aDataSet[0]->getBoundingCube(octreeCube);
	//the octree Cube must contains ds cube
	cube_type  c;
	ds.getBoundingCube(c);
	assert(octreeCube.contains(c));
	if (octreeCube.contains(c) == false)
	{
		throw DgalError("OCTREE::add : octree doesn't contains the new dataSet cube");
	}


	//populate the tree
	unsigned insertionLevel = _depth-1;
	T minSize, maxSize, avgSize;
	ds.getSizeRange(minSize,maxSize,avgSize);

	if (components== NULL) //add all data set elements
	{
		if (maxSize ==0)
		{ // data with elem of size null (pts)
			for (unsigned  dataIndex=0;dataIndex<ds.size();dataIndex++)
			{
				//fill _aCellData
				std::pair<char,unsigned> data(dataSetIndex ,dataIndex);
				_aCellData.push_back(data);
				insertPointData(_pRoot,data , _aCellData.size()-1,  insertionLevel, _maxNumElemByLeaf);
			}
		}
		else
		{ // data with elem of size not null ()
			for (unsigned  dataIndex=0;dataIndex<ds.size();dataIndex++)
			{
				//Find the smallest cell bounding the region for this element
				Imath::Box<point_type> box;
				ds.getBounds(dataIndex,box);
				Cell* pCellRegion = locateRegion(_pRoot,box);
				//fill _aCellData
				std::pair<char,unsigned> data(dataSetIndex ,dataIndex);
				_aCellData.push_back(data);
				insertVolumeData( pCellRegion ,data , _aCellData.size()-1,  insertionLevel, _maxNumElemByLeaf);
			}
		}
	}
	else //add just component list
	{
		if (maxSize ==0)
		{ // data with elem of size null (pts)
			boost::unordered_set<unsigned>::const_iterator cIt;
			for (cIt =  components->begin(); cIt != components->end(); cIt++)
			{
				//fill _aCellData
				unsigned component = *cIt;
				assert( component<ds.size() );
				std::pair<char,unsigned> data(dataSetIndex ,component);
				_aCellData.push_back(data);
				insertPointData(_pRoot,data , _aCellData.size()-1,  insertionLevel, _maxNumElemByLeaf);
			}
		}
		else
		{ // data with elem of size not null ()
			boost::unordered_set<unsigned>::const_iterator cIt;
			for (cIt =  components->begin(); cIt != components->end(); cIt++)
			{
				//fill _aCellData
				unsigned component = *cIt;
				//Find the smallest cell bounding the region for this element
				Imath::Box<point_type> box;
				ds.getBounds(component,box);
				Cell* pCellRegion = locateRegion(_pRoot,box);
				//fill _aCellData
				std::pair<char,unsigned> data(dataSetIndex ,component);
				_aCellData.push_back(data);
				insertVolumeData( pCellRegion ,data , _aCellData.size()-1,  insertionLevel, _maxNumElemByLeaf);
			}
		}
	}
}
//-------------------------------------------------------------------------------------------------
template< typename DataSet >
Octree<DataSet>::Octree( const DataSet& ds, int maxNumElemByLeaf,const boost::unordered_set<unsigned>* components ):
_maxNumElemByLeaf(maxNumElemByLeaf)
{

	 _depth = 32;
	 _twoPowMaxLevel =  pow(2.0,(double)(_depth-1)) ,
	_pRoot = new Cell(_depth-1); //level of the root is depth -1

	//set affine transformation for the tree
	cube_type c;
	ds.getBoundingCube(c);
	_rootTranslate = point_type(c.min().x,c.min().y,c.min().z);
	_rootScale = c.size();

	add(ds,components);

}

//-------------------------------------------------------------------------------------------------
template<typename DataSet >
Octree< DataSet >::~Octree()
{
	delete _pRoot;
	_pRoot = NULL;
}

//-------------------------------------------------------------------------------------------------
template<typename DataSet>
typename Octree<DataSet>::point_type Octree<DataSet>::transformFrom_01(const point_type& v) const
{
	return (v*_rootScale + _rootTranslate) ;
}
//-------------------------------------------------------------------------------------------------
template<typename DataSet>
typename Octree<DataSet>::point_type Octree<DataSet>::transformTo_01(const point_type& v) const
{
	point_type v_tr = v+(_rootTranslate*-1.0f);
	v_tr = v_tr*(1/_rootScale);
	assert( 	v_tr.x>-_cellEpsilon && v_tr.y >-_cellEpsilon && v_tr.z >-_cellEpsilon &&
				v_tr.x<1.0+_cellEpsilon && v_tr.y<1.0+_cellEpsilon && v_tr.z<1.0+_cellEpsilon);
	return v_tr;
}
//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void Octree<DataSet>::opCodeFromPos(const point_type& v,unsigned& outXLocCode, unsigned& outYLocCode, unsigned& outZLocCode) const
{
	const point_type& vtx01 = transformTo_01(v);
	//----Determine the x and y locational codes of the point's position. Refer
	//----to [King2001] for more efficient methods for converting floating point
	//----numbers to integers.

	opCodeFromPos_01(vtx01, outXLocCode, outYLocCode, outZLocCode);


}
//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void Octree<DataSet>::opCodeFromPos_01(const point_type& v,unsigned& outXLocCode, unsigned& outYLocCode, unsigned& outZLocCode) const
{

	//----Determine the x and y locational codes of the point's position. Refer
	//----to [King2001] for more efficient methods for converting floating point
	//----numbers to integers.


	assert(v.x>=-_cellEpsilon);
	assert(v.x<1.0+_cellEpsilon);
	assert(v.y>=-_cellEpsilon);
	assert(v.y<1.0+_cellEpsilon);
	assert(v.z>=-_cellEpsilon);
	assert(v.z<1.0+_cellEpsilon);

	outXLocCode = (unsigned) (v.x* (_twoPowMaxLevel));
	outYLocCode = (unsigned) (v.y* (_twoPowMaxLevel));
	outZLocCode = (unsigned) (v.z* (_twoPowMaxLevel));


}
//-------------------------------------------------------------------------------------------------
// traverse the Octree  from a specified cell (typically the root cell)
// to a leaf cell by following the x, y and z locational codes, xLocCode and
// yLocCode, zLocCode. Upon entering, cell is the specified cell and nextLevel is one less
// than the level of the specified cell. Upon termination, cell is the leaf cell or the level specified by minLevel.
// and nextLevel is one less than the level of the leaf cell.
template<typename DataSet>
void Octree<DataSet>::traverse(Cell*& cell,unsigned& nextLevel,unsigned xLocCode,unsigned yLocCode,unsigned zLocCode,unsigned minLevel) const
{
	assert(nextLevel>= minLevel);
	while(true)
	{
		unsigned ci = childIndex(nextLevel, xLocCode, yLocCode, zLocCode);

		if ( cell->getChild(ci) != NULL )
		{
			cell = cell->getChild(ci);
			if (nextLevel == minLevel)
				break;
			nextLevel--;
		}
		else
			break;
	}
}

//-------------------------------------------------------------------------------------------------
template<typename DataSet>
unsigned Octree< DataSet >::childIndex(unsigned level,unsigned xLocCode,unsigned yLocCode,unsigned zLocCode) const
{
	//unsigned int childBranchBit = 1 << (level);
	unsigned int childIndex =	   ( ( xLocCode >> level) & 0x1 )
								 | ( ( yLocCode >> level & 0x1) << 1)
								 | ( ( zLocCode >> level & 0x1) << 2);
    return childIndex;

}

//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void Octree<DataSet>::getCube(const Cell* pCell,bool bTransform, cube_type& outCube) const
{
	float cWidth = 1.0f/pow(2.0f,(float)(_depth -pCell->level()-1));
	unsigned xLocCode,yLocCode,zLocCode;
	pCell->getLocCode(xLocCode,yLocCode,zLocCode);

	point_type cPos(xLocCode/_twoPowMaxLevel,yLocCode/_twoPowMaxLevel,zLocCode/_twoPowMaxLevel);

	if (bTransform)
	{
		cWidth *= _rootScale;
		cPos = transformFrom_01(cPos);
	}
	outCube.set(cPos,cWidth);
}

//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void Octree<DataSet>::getCubes_recurse(const Cell* pCell, bool bTransform, unsigned level, std::vector<cube_type>& outCubes) const
{
	if (level == pCell->level())
	{
		cube_type c;

		getCube(pCell, bTransform, c);
		outCubes.push_back(c);
	}
	//
	for (int i=0;i<8;i++)
	{
		Cell* c = pCell->getChild(i);
		if ( c != NULL)
			getCubes_recurse(c,bTransform,level,outCubes);
	}


}
//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void Octree<DataSet>::getCubes(bool bTransform, unsigned level, std::vector<cube_type>& outCubes ) const
{
	if (level == -1)
	{
		for (unsigned i=0;i<_depth;i++)
		{
			getCubes_recurse( _pRoot,bTransform, i, outCubes );
		}
	}
	else
	{
		getCubes_recurse( _pRoot,bTransform, level, outCubes );
	}

}

//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void Octree<DataSet>::getChildCubes(const cube_type& cub, cube_type* aChildCube) const
{
	T currSize = cub.size();
	for (unsigned c =0;c<8;c++)
	{
		//test against child Cube
		cube_type childCub = cub;
		childCub.scale(0.5);
		childCub.translate(_aParentToChildtranslate[c]*currSize);
		aChildCube[c]= childCub;
	}
}

//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void Octree<DataSet>::insertVolumeData(Cell* pCell,const std::pair<char,unsigned> & data,unsigned dataIndex,  unsigned insertionLevel, unsigned nMaxNumElemByLeaf)
{
	assert(pCell !=NULL);

	Imath::Box<point_type> elemBox;
	cube_type cub;
	getCube(pCell, false, cub);

	if (pCell->level() <= insertionLevel &&  !pCell->hasChild() ) //Insertion level reached and this is a leaf
	{
		//insert data here
		pCell->addData(dataIndex);
		//num elem max by cell reached
		const boost::unordered_map<unsigned, unsigned >& aData = pCell->getData();
		if (aData.size()>= nMaxNumElemByLeaf)
		{
			assert(pCell->hasChild() == false);
			if (pCell->level()==0)
			{
				throw DgalError("OCTREE::AddVertex : octree depth exceeded");
			}

			//precompute childCubes
			cube_type aChildCube[8];
			getChildCubes(cub,aChildCube);

			//move each cell elem down
			boost::unordered_map<unsigned,unsigned>::const_iterator cit;
			for (cit =aData.begin(); cit!= aData.end();cit++)
			{
				unsigned currIndex = (*cit).first;
				char elemSetIndex = _aCellData[currIndex].first;
				unsigned elemIndex = _aCellData[currIndex].second;
				_aDataSet[elemSetIndex]->getBounds(elemIndex,elemBox);
				for (unsigned c =0;c<8;c++)
				{
					if (aChildCube[c].intersect(elemBox))
					{
						Cell* pCurrChildCell = pCell->createChild(c,*this);
						pCurrChildCell->addData(currIndex);
					}
				}
			}
			pCell->clearData();
		}
	}
	else
	{
		// Cell level > insertionLevel --> subdivide the tree or thaverse until a leaf is found
		cube_type aChildCube[8];
		getChildCubes(cub,aChildCube);
		_aDataSet[data.first]->getBounds(data.second,elemBox);
		for ( unsigned c=0; c<8; c++ )
		{

			if (aChildCube[c].intersect(elemBox))
			{
				Cell* pCurrChildCell = pCell->createChild(c,*this);
				insertVolumeData(pCurrChildCell,data, dataIndex, insertionLevel, nMaxNumElemByLeaf);
			}
		}
	}

	// make the descendantCountInfo available per node.
	//computeDescendantCount();
}

template<typename DataSet>
void Octree<DataSet>::insertPointData(Cell* pCell,const std::pair<char,unsigned> & data,unsigned dataIndex,  unsigned insertionLevel, unsigned nMaxNumElemByLeaf)
{

	point_type center;
	_aDataSet[data.first]->getBarycenter(data.second,center);
	unsigned xLocCode, yLocCode, zLocCode;
	opCodeFromPos_01( center, xLocCode, yLocCode, zLocCode);
	// traverse down the tree
	unsigned int nextLevel = _depth - 2;
	traverse(pCell,nextLevel,xLocCode,yLocCode,zLocCode,insertionLevel);

	// subdivide the tree until insertionLevel is reached
	while ( pCell->level() > insertionLevel  )
	{
		unsigned ci = childIndex(pCell->level()-1,xLocCode,yLocCode,zLocCode);
		pCell = pCell->createChild(ci,*this);
	}
	//	set data
	if (pCell->hasChild()) //create a new child as a container for this point
	{
		assert(pCell->getData().size()==0); //this cell has child thus, all verts should belong to the children

		int chIndex = childIndex(pCell->level()-1,xLocCode,yLocCode,zLocCode);
		Cell* pCurrChildCell = pCell->createChild(chIndex,*this);
		pCurrChildCell->addData(dataIndex);
	}
	else
	{
		pCell->addData(dataIndex);
		const boost::unordered_map<unsigned, unsigned >& aData = pCell->getData();
		if (aData.size() > nMaxNumElemByLeaf) //Cell subdiv condition
		{
			assert(pCell->hasChild() == false);
			if (pCell->level()==0)
			{
				throw DgalError("OCTREE::AddVertex : octree depth exceeded");
			}
			boost::unordered_map<unsigned,unsigned>::const_iterator cit;
			for (cit =aData.begin(); cit!= aData.end();cit++)
			{
				unsigned currIndex = (*cit).first;
				char elemSetIndex = _aCellData[currIndex].first;
				unsigned elemIndex = _aCellData[currIndex].second;
				_aDataSet[elemSetIndex]->getBarycenter(elemIndex,center);
				unsigned xLocCode, yLocCode, zLocCode;
				opCodeFromPos_01(center, xLocCode, yLocCode, zLocCode);

				int chIndex = childIndex(pCell->level()-1,xLocCode,yLocCode,zLocCode);
				Cell* pCurrChildCell = pCell->createChild(chIndex,*this);

				pCurrChildCell->addData(currIndex);

			}
			pCell->clearData();
		}
	}
	// make the descendantCountInfo available per node.
	//computeDescendantCount();
}
//-------------------------------------------------------------------------------
// Locate the smallest cell that entirely contains a rectangular region defined
// by its bottom-left vertex v0 and its top-right vertex v1, where v0 and v1
// lie in [0,1)x[0,1).
//-------------------------------------------------------------------------------
template<typename DataSet>
Cell* Octree<DataSet>::locateRegion (Cell * root, const Imath::Box<point_type>& box ) const
{

	unsigned int x0LocCode,y0LocCode,z0LocCode;
	unsigned int x1LocCode,y1LocCode,z1LocCode;
	opCodeFromPos_01(box.min,x0LocCode,y0LocCode,z0LocCode);
	opCodeFromPos_01(box.max,x1LocCode,y1LocCode,z1LocCode);

	//----Determine the XOR'ed pairs of locational codes of the region boundaries
	unsigned int xDiff = x0LocCode ^ x1LocCode;
	unsigned int yDiff = y0LocCode ^ y1LocCode;
	unsigned int zDiff = z0LocCode ^ z1LocCode;


	//----Determine the level of the smallest possible cell entirely containing
	//----the region
	Cell *cell = root;
	unsigned int level = _depth - 1;
	unsigned int minLevel = level;
	unsigned int minLevel1 = level;
	while (!(xDiff & (1 << level)) && level) level--;
	while (!(yDiff & (1 << minLevel)) && (minLevel > level)) minLevel--;
	while (!(zDiff & (1 << minLevel1)) && (minLevel1 > minLevel)) minLevel1--;
	minLevel1++;


	//----Follow the branching patterns of the locational codes of v0 from the
	//----root cell to the smallest cell entirely containing the region
	level = _depth - 2;
	if (level>=minLevel1)
	{
		traverse(cell,level,x0LocCode,y0LocCode,z0LocCode,minLevel1);
	}
	return(cell);

}
//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void Octree<DataSet>::getStat(unsigned& nMinElemByCell ,unsigned& nMaxElemByCell) const
{
	nMaxElemByCell = 0;
	nMinElemByCell = UINT_MAX;
	_pRoot->getStatRecurse(nMinElemByCell,nMaxElemByCell);
}

template<typename DataSet>
bool Octree<DataSet>::clipBox(const Imath::Box<point_type>& box,Imath::Box<point_type>& oClipedBox,T epsilon) const
{
	//clip box against the octree cube
	Imath::Box<point_type> boxClip;
	boxClip.min.x = (box.min.x-epsilon<_rootTranslate.x) ? _rootTranslate.x : box.min.x-epsilon;
	boxClip.min.y = (box.min.y-epsilon<_rootTranslate.y) ? _rootTranslate.y : box.min.y-epsilon;
	boxClip.min.z = (box.min.z-epsilon<_rootTranslate.z) ? _rootTranslate.z : box.min.z-epsilon;

	boxClip.max.x = (box.max.x+epsilon>_rootTranslate.x+_rootScale) ? _rootTranslate.x+_rootScale : box.max.x+epsilon;
	boxClip.max.y = (box.max.y+epsilon>_rootTranslate.y+_rootScale) ? _rootTranslate.y+_rootScale : box.max.y+epsilon;
	boxClip.max.z = (box.max.z+epsilon>_rootTranslate.z+_rootScale) ? _rootTranslate.z+_rootScale : box.max.z+epsilon;

	//exit if boxClip is too small
	point_type diff = boxClip.max - boxClip.min;
	if ( diff.x<_cellEpsilon || diff.y<_cellEpsilon || diff.z<_cellEpsilon )
		return false;

	oClipedBox.min = transformTo_01(boxClip.min);
	oClipedBox.max = transformTo_01(boxClip.max);

	return true;
}
//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void Octree<DataSet>::query(const Imath::Box<point_type>& qBox, std::vector< boost::unordered_set<unsigned> >& outResult,T epsilon) const
{
	Imath::Box<point_type> qBox01;

	if (!clipBox(qBox,qBox01,epsilon))
		return; // cliped box is too small
	//set out result size
	outResult.resize(_aDataSet.size());
	//locate region and recurse
	Cell* pCellRegion = locateRegion(_pRoot,qBox01);
	pCellRegion->queryDataRecurse(qBox01, *this, outResult );

}


//-------------------------------------------------------------------------------------------------
template<typename DataSet>
bool Octree<DataSet>::intersect(const Imath::Box<point_type>& qBox,  T epsilon, std::vector<cube_type>* aC ) const
{
	Imath::Box<point_type> qBox01;
	if (!clipBox(qBox,qBox01,epsilon ))
		return false; // cliped box is too small

	Cell* pCellRegion = locateRegion(_pRoot,qBox01);
	//tmp debug
	cube_type c;
	getCube(pCellRegion,false,c);
	if(aC)
		aC->push_back(c);
	//
	bool outRes = false;
	pCellRegion->intersectRecurse(qBox01, *this,outRes );
	return outRes;

}

//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void  Octree<DataSet>::computeDescendantCount()
{
	_pRoot->computeDescendantCountRecurse();
}
//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void  Octree<DataSet>::query(const Imath::Plane3<T>& plane,
	std::vector< boost::unordered_set<unsigned> >* outLeft,
	std::vector< boost::unordered_set<unsigned> >* outRight,
	std::vector< boost::unordered_set<unsigned> >* outIntersect,T epsilon ) const
{
	if (_rootScale==0)
		return;

	//apply octree affine transformation to the plane
	Imath::Plane3<T> plane_01;
	Imath::Vec3<T> pOrg = plane.normal* plane.distance;
	Imath::Vec3<T> v_tr = pOrg+(_rootTranslate*-1.0f);
	v_tr = v_tr*(1/_rootScale);


	T epsilon_01 = epsilon/_rootScale;
	T prjLen = v_tr.dot(plane.normal);

	plane_01.distance = prjLen;
	plane_01.normal= plane.normal;

	assert(	plane_01.normal.length()<1.0f+1e-4 && plane_01.normal.length()>1.0f-1e-4);

	//set out result size
	if (outLeft!=NULL)
		outLeft->resize(_aDataSet.size());
	if (outRight!=NULL)
		outRight->resize(_aDataSet.size());
	if (outIntersect != NULL)
		outIntersect->resize(_aDataSet.size());
	_pRoot->queryDataRecurse(plane_01, *this, outLeft, outRight, outIntersect,epsilon );


}
//-------------------------------------------------------------------------------------------------
template<typename DataSet>
bool Octree<DataSet>::contains(const point_type& pt) const
{
	return !( 	pt.x-_cellEpsilon < _rootTranslate.x ||
				pt.y-_cellEpsilon < _rootTranslate.y ||
				pt.z-_cellEpsilon < _rootTranslate.z ||

				pt.x+_cellEpsilon > _rootTranslate.x+_rootScale ||
				pt.y+_cellEpsilon > _rootTranslate.y+_rootScale  ||
				pt.z+_cellEpsilon > _rootTranslate.z+_rootScale   );
}
//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void Octree<DataSet>::query(	const point_type& pt,
								short axis,bool castForward,
								std::vector< boost::unordered_set<unsigned> >& outResult,
								T epsilon) const
{
	bool bInside = contains(pt); //todo remove this limitation later if needed.
	//assert(bInside);
	if (!bInside)
		return;

	point_type ptStart_01 =transformTo_01(pt);
	ptStart_01[axis] -= epsilon;


	//set out result size
	outResult.resize(_aDataSet.size());

	_pRoot->queryDataRecurse(ptStart_01, axis, castForward, *this, outResult);
}
//-------------------------------------------------------------------------------------------------
template<typename DataSet>
void  Octree<DataSet>::getAffineTransform(cube_type& outTransform)
{
	outTransform.set(_rootTranslate,_rootScale);
}

} // namespace dgal
#endif //_DGAL_OCTREE__H_


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
