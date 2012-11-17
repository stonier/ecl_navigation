/**
 * @file /include/ecl/maps/grid_map_dynamic.hpp
 *
 * @brief Basic (resizable) grid map container.
 *
 * @date January 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_MAPS_GRID_MAP_DYNAMIC_HPP_
#define ECL_MAPS_GRID_MAP_DYNAMIC_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "grid_map_fixed.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [GridMap]
*****************************************************************************/
/**
 * @brief Stores a discretised map over a real world co-ordinate (x-y) system.
 *
 * Interface for a discretised grid map overlaying a real world x,y co-ordinate
 * system. See the constructor for detailed information on how the grid map
 * is overlaid on the real world co-ordinate system.
 * This is just dynamic storage version of GirdMap. Actually nothing special
 *
 * This is just a container storage system with real world/grid map coordinate access, no more.
 *
 * <b>Usage:</b>
 *
 * - Specify the content type and size via the template parameters.
 * - Lock and scale the grid map over the real world map via the constructor.
 * - Access either via the 1d iterators or via the 2d operator(_,_) methods.
 *
 * @tparam CellType : type for allocated cells in the grid map.
 */
template <typename CellType>
class GridMap<CellType,DynamicGridMapStorage,DynamicGridMapStorage> {
public:
	/*********************
	** Typedefs
	**********************/
	typedef CellType value_type; /**< @brief Grid map's cell type. **/
	typedef CellType* iterator; /**< Array's iterator type. **/
	typedef const CellType* const_iterator; /**< Grid map's constant iterator type. **/
	typedef CellType& reference;  /**< Grid map's element reference type. **/
	typedef const CellType& const_reference;  /**< Grid map's element const reference type. **/
	typedef int size_type; /**< Grid map's type used to denote the length of the array. **/

	/*********************
	** Initialisation
	**********************/
	GridMap(const int & numOfRows=1, const int & numOfCols=1,
			const double &origin_x = 0.0, const double &origin_y = 0.0,
			const double &scale = 1.0);
	~GridMap();
	void init(const CellType & initialValue= CellType() );

	/*********************
	** 2D Accessors
	**********************/
	CellType& operator()(const int& x, const int& y) ecl_assert_throw_decl(StandardException);
	CellType& operator()(const double& x, const double& y) ecl_assert_throw_decl(StandardException);

	/*********************
	** 1D Iterators
	**********************/
	size_type size() const { return num_of_rows*num_of_cols; } /**< @brief Grid map size (in 1 dimension), i.e. total number of cells. **/
	iterator begin() { return cells; } /**< @brief One dimensional iterator to the start of the underlying array of cells. **/
	const_iterator begin() const { return cells; } /**< @brief One dimensional const iterator to the start of the underlying array of cells. **/
	iterator end() { return cells+num_of_rows*num_of_cols; } /**< @brief One dimensional iterator to the end of the underlying array of cells. **/
	const_iterator end() const { return cells+num_of_rows*num_of_cols; } /**< @brief One dimensional const iterator to the end of the underlying array of cells. **/
	reference front() { return cells[0]; } /**< @brief Reference to the first grid cell. **/
	const_reference front() const { return cells[0]; } /**< @brief Const reference to the first grid cell. **/
	reference back() { return cells[num_of_rows*num_of_cols-1]; } /**< @brief Reference to the last grid cell. **/
	const_reference back() const { return cells[num_of_rows*num_of_cols-1]; } /**< @brief Const reference to the last grid cell. **/

	/*********************
	** Conversions
	**********************/
	linear_algebra::Vector2i cellCoordinates(const double &x, const double &y ) const ecl_assert_throw_decl(StandardException);
	linear_algebra::Vector2d worldCoordinates(const int &x, const int &y ) const ecl_assert_throw_decl(StandardException);
	iterator cell(const int &x, const int &y) const ecl_assert_throw_decl(StandardException)
	{
		ecl_assert_throw( ( ( x >= 0 ) && (x <= num_of_cols) ), StandardException(LOC,OutOfRangeError) );
		ecl_assert_throw( ( ( y >= 0 ) && (y <= num_of_rows) ), StandardException(LOC,OutOfRangeError) );
		return (cells +num_of_cols*y+x);
	}
	iterator cell(const double &x, const double &y) const ecl_assert_throw_decl(StandardException)
	{
		int cx = static_cast<int>( ( x - origin[0] )/scale_multiplier );
		int cy = static_cast<int>( ( y - origin[1] )/scale_multiplier );
		return cell(cx,cy);
	}

	bool cell(const int &x, const int &y, iterator & it) const
	{
		if( x < 0 || x>= num_of_cols ) return false;
		if( y < 0 || y>= num_of_rows ) return false;
		it = cells + num_of_cols*y+x;
		return true;
	}
	bool cell(const double &x, const double &y, iterator & it) const
	{
		int cx = static_cast<int>( ( x - origin[0] )/scale_multiplier );
		int cy = static_cast<int>( ( y - origin[1] )/scale_multiplier );
		return cell(cx,cy, it);
	}


	/*********************
	** Utility
	**********************/
	double cellWidth() const { return scale_multiplier; } /**< @brief Width of a single cell. **/
	size_type rows() const { return num_of_rows; } /**< @brief Number of num_of_rows in the grid map. **/
	size_type cols() const { return num_of_cols; } /**< @brief Number of num_of_cols in the grid map. **/
	void remap(const int & numOfRows=1, const int & numOfCols=1,
				const double &origin_x = 0.0, const double &origin_y = 0.0,
				const double &scale = 1.0);
	linear_algebra::Vector2d originVector() const { return origin; }
	void operator=(const GridMap<CellType,DynamicGridMapStorage,DynamicGridMapStorage> & gridMap );
	bool operator == (const GridMap<CellType,DynamicGridMapStorage,DynamicGridMapStorage> & gridMap )
	{
		if( num_of_rows != gridMap.rows() ||
			num_of_cols != gridMap.cols() ||
			origin[0] != gridMap.originVector()[0] ||
			origin[1] != gridMap.originVector()[1] ||
			scale_multiplier != gridMap.cellWidth() )
			return false;
		else
			return true;
	}

	bool operator != (const GridMap<CellType,DynamicGridMapStorage,DynamicGridMapStorage> & gridMap )
	{
		if( num_of_rows != gridMap.rows() ||
			num_of_cols != gridMap.cols() ||
			origin[0] != gridMap.originVector()[0] ||
			origin[1] != gridMap.originVector()[1] ||
			scale_multiplier != gridMap.cellWidth() )
			return true;
		else
			return false;
	}

private:
	double scale_multiplier;
	linear_algebra::Vector2d origin;
	linear_algebra::Vector2d limits;
	CellType * cells;
	int num_of_rows;
	int num_of_cols;
};

/*****************************************************************************
** Implementation [GridMap][Constructors]
*****************************************************************************/
/**
 * @brief Locks real world co-ordinates onto the the grid map.
 *
 * This constructor sets the real world location and scale (stretch) multiplier
 * for the grid map. The bottom left corner of the grid map will lock onto
 * the specified real world co-ordinates.
 *
 * Note that the size of the grid map is specified through the numOfRows and numOfCols
 * instead of template parameters which is used in fixed size version of GridMap.
 * Subseqently the real world coverage is determined via these
 * and the specified real world input args.
 *
 * - Real Domain : [origin_x, origin_x + num_of_cols*scale]
 * - Grid Domain : [    0,             num_of_cols        ]
 * - Real Range  : [origin_y, origin_y + num_of_rows*scale]
 * - Grid Range  : [    0,             num_of_rows        ]
 *
 * Also note that the grid map is filled with the default cell type object (ie.
 * constructor with () call).
 */
template <typename CellType>
GridMap<CellType,DynamicGridMapStorage,DynamicGridMapStorage>
::GridMap( const int & numOfRows, const int & numOfCols, const double &origin_x, const double &origin_y, const double &scale)
    :scale_multiplier(scale),
	num_of_rows(numOfRows),
	num_of_cols(numOfCols)
{
	ecl_assert_throw( (num_of_rows > 0) && (num_of_cols > 0), StandardException(LOC,OutOfRangeError) );

	origin << origin_x, origin_y;
	limits << origin[0] + scale_multiplier*num_of_cols, origin[1] + scale_multiplier*num_of_rows;

	cells = new CellType[ num_of_rows * num_of_cols ];
}

template <typename CellType>
void GridMap<CellType,DynamicGridMapStorage,DynamicGridMapStorage>
::remap( const int & numOfRows, const int & numOfCols, const double &origin_x, const double &origin_y, const double &scale)
{
	scale_multiplier = scale;
	num_of_rows = numOfRows;
	num_of_cols = numOfCols;

	ecl_assert_throw( (num_of_rows > 0) && (num_of_cols > 0), StandardException(LOC,OutOfRangeError) );

	origin << origin_x, origin_y;
	limits << origin[0] + scale_multiplier*num_of_cols, origin[1] + scale_multiplier*num_of_rows;

	if( cells != 0 )	delete [] cells;

	cells = new CellType[ num_of_rows * num_of_cols ];
}

template<typename CellType>
GridMap<CellType,DynamicGridMapStorage,DynamicGridMapStorage>
::~GridMap()
{
	delete [] cells;
}


/**
 * This allows resetting of the grid map at a later stage. This currently just fills
 * the map with the same default cell type object. (We're assuming here that the
 * coordinates of the grid map do not change).
 */
template<typename CellType>
void GridMap<CellType,DynamicGridMapStorage,DynamicGridMapStorage>
::init(const CellType & initialValue ) {
	for ( int i = 0; i < num_of_rows*num_of_cols; ++i ) {
		cells[i] = initialValue;//CellType();
	}
}

/*****************************************************************************
** Implementation [GridMap][Access]
*****************************************************************************/

template<typename CellType>
CellType& GridMap<CellType,DynamicGridMapStorage,DynamicGridMapStorage>
::operator() (const int& x, const int& y) ecl_assert_throw_decl(StandardException) {
	ecl_assert_throw( ( ( x >= 0 ) && (x <= num_of_cols) ), StandardException(LOC,OutOfRangeError) );
	ecl_assert_throw( ( ( y >= 0 ) && (y <= num_of_rows) ), StandardException(LOC,OutOfRangeError) );

	return cells[num_of_cols*y+x];
}

template<typename CellType>
CellType& GridMap<CellType,DynamicGridMapStorage,DynamicGridMapStorage>
::operator() (const double& x, const double& y) ecl_assert_throw_decl(StandardException) {

	ecl_assert_throw( ( ( x >= origin[0] ) && (x <= limits[0]) ), StandardException(LOC,OutOfRangeError) );
	ecl_assert_throw( ( ( y >= origin[1] ) && (y <= limits[1]) ), StandardException(LOC,OutOfRangeError) );

	int cell_x = static_cast<int>( ( x - origin[0] )/scale_multiplier );
	int cell_y = static_cast<int>( ( y - origin[1] )/scale_multiplier );
	return cells[num_of_cols*cell_y+cell_x];
}

/*****************************************************************************
** Implementation [GridMap][Conversions]
*****************************************************************************/

template<typename CellType>
linear_algebra::Vector2i GridMap<CellType,DynamicGridMapStorage,DynamicGridMapStorage>
::cellCoordinates (const double& x, const double& y) const ecl_assert_throw_decl(StandardException) {

	ecl_assert_throw( ( ( x >= origin[0] ) && (x <= limits[0]) ), StandardException(LOC,OutOfRangeError) );
	ecl_assert_throw( ( ( y >= origin[1] ) && (y <= limits[1]) ), StandardException(LOC,OutOfRangeError) );

	linear_algebra::Vector2i cell_coordinates;
	cell_coordinates << static_cast<int>( ( x - origin[0] )/scale_multiplier ),
						static_cast<int>( ( y - origin[1] )/scale_multiplier );
	return cell_coordinates;
}

template<typename CellType>
ecl::linear_algebra::Vector2d GridMap<CellType,DynamicGridMapStorage,DynamicGridMapStorage>
::worldCoordinates (const int& x, const int& y) const ecl_assert_throw_decl(StandardException) {

	ecl_assert_throw( ( ( x >= 0 ) && (x <= num_of_cols) ), StandardException(LOC,OutOfRangeError) );
	ecl_assert_throw( ( ( y >= 0 ) && (y <= num_of_rows) ), StandardException(LOC,OutOfRangeError) );

	linear_algebra::Vector2d world_coordinates;
	world_coordinates << origin[0] + scale_multiplier*static_cast<double>(x),
						 origin[1] + scale_multiplier*static_cast<double>(y);
	return world_coordinates;
}


template<typename CellType>
void GridMap<CellType,DynamicGridMapStorage,DynamicGridMapStorage>
::operator = (const GridMap<CellType,DynamicGridMapStorage,DynamicGridMapStorage> & gridMap )
{
	if( num_of_rows != gridMap.rows() ||
		num_of_cols != gridMap.cols() ||
		origin[0] != gridMap.originVector()[0] ||
		origin[1] != gridMap.originVector()[1] ||
		scale_multiplier != gridMap.cellWidth() )
	{
		remap( gridMap.rows(), gridMap.cols(), gridMap.originVector()[0], gridMap.originVector()[1], gridMap.cellWidth() );
	}

	std::copy( gridMap.begin(), gridMap.end(), begin());
}



} // namespace ecl

#endif /* ECL_MAPS_GRID_MAP_DYNAMIC_HPP_ */


