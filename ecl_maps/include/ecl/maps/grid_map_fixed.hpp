/**
 * @file /include/ecl/maps/grid_map_fixed.hpp
 *
 * @brief Basic (non-resizable) grid map container.
 *
 * @date January 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_MAPS_GRID_MAP_FIXED_HPP_
#define ECL_MAPS_GRID_MAP_FIXED_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "grid_map_commons.hpp"

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
 * @tparam Rows : number of rows allocated to the fixed grid map.
 * @tparam Cols : number of cols allocated to the fixed grid map, default is Cols=Rows.
 */
template <typename CellType, int Rows = DynamicGridMapStorage, int Cols = Rows>
class GridMap {
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
	GridMap(const double &origin_x = 0.0, const double &origin_y = 0.0, const double &scale = 1.0);
	void init(const CellType & initialValue= CellType() );

	/*********************
	** 2D Accessors
	**********************/
	CellType& operator()(const int& x, const int& y) ecl_assert_throw_decl(StandardException);
	CellType& operator()(const double& x, const double& y) ecl_assert_throw_decl(StandardException);

	/*********************
	** 1D Iterators
	**********************/
	size_type size() const { return Rows*Cols; } /**< @brief Grid map size (in 1 dimension), i.e. total number of cells. **/
	iterator begin() { return cells; } /**< @brief One dimensional iterator to the start of the underlying array of cells. **/
	const_iterator begin() const { return cells; } /**< @brief One dimensional const iterator to the start of the underlying array of cells. **/
	iterator end() { return cells+Rows*Cols; } /**< @brief One dimensional iterator to the end of the underlying array of cells. **/
	const_iterator end() const { return cells+Rows*Cols; } /**< @brief One dimensional const iterator to the end of the underlying array of cells. **/
	reference front() { return cells[0]; } /**< @brief Reference to the first grid cell. **/
	const_reference front() const { return cells[0]; } /**< @brief Const reference to the first grid cell. **/
	reference back() { return cells[Rows*Cols-1]; } /**< @brief Reference to the last grid cell. **/
	const_reference back() const { return cells[Rows*Cols-1]; } /**< @brief Const reference to the last grid cell. **/

	/*********************
	** Conversions
	**********************/
	linear_algebra::Vector2i cellCoordinates(const double &x, const double &y ) const ecl_assert_throw_decl(StandardException);
	linear_algebra::Vector2d worldCoordinates(const int &x, const int &y ) const ecl_assert_throw_decl(StandardException);

	/*********************
	** Utility
	**********************/
	double cellWidth() const { return scale_multiplier; } /**< @brief Width of a single cell. **/
	size_type rows() const { return Rows; } /**< @brief Number of rows in the grid map. **/
	size_type cols() const { return Cols; } /**< @brief Number of cols in the grid map. **/

private:
	double scale_multiplier;
	linear_algebra::Vector2d origin;
	linear_algebra::Vector2d limits;
	CellType cells[Rows*Cols];
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
 * Note that the size of the grid map is specified through the template
 * parameters. Subseqently the real world coverage is determined via these
 * and the specified real world input args.
 *
 * - Real Domain : [origin_x, origin_x + Cols*scale]
 * - Grid Domain : [    0,             Cols        ]
 * - Real Range  : [origin_y, origin_y + Rows*scale]
 * - Grid Range  : [    0,             Rows        ]
 *
 * Also note that the grid map is filled with the default cell type object (ie.
 * constructor with () call).
 */
template <typename CellType, int Rows, int Cols>
GridMap<CellType,Rows,Cols>::GridMap(const double &origin_x, const double &origin_y,  const double &scale) :
	scale_multiplier(scale)
{
	ecl_compile_time_assert( (Rows >= 0) && (Cols >= 0) );

	origin << origin_x, origin_y;
	limits << origin[0] + scale_multiplier*Cols, origin[1] + scale_multiplier*Rows;
}

/**
 * This allows resetting of the grid map at a later stage. This currently just fills
 * the map with the same default cell type object. (We're assuming here that the
 * coordinates of the grid map do not change).
 */
template <typename CellType, int Rows, int Cols>
void GridMap<CellType,Rows,Cols>::init(const CellType & initialValue ) {
	for ( int i = 0; i < Rows*Cols; ++i ) {
		cells[i] = initialValue;//CellType();
	}
}

/*****************************************************************************
** Implementation [GridMap][Access]
*****************************************************************************/

template <typename CellType, int Rows, int Cols>
CellType& GridMap<CellType,Rows,Cols>::operator() (const int& x, const int& y) ecl_assert_throw_decl(StandardException) {
	ecl_assert_throw( ( ( x >= 0 ) && (x <= Cols) ), StandardException(LOC,OutOfRangeError) );
	ecl_assert_throw( ( ( y >= 0 ) && (y <= Rows) ), StandardException(LOC,OutOfRangeError) );

	return cells[Cols*y+x];
}

template <typename CellType, int Rows, int Cols>
CellType& GridMap<CellType,Rows,Cols>::operator() (const double& x, const double& y) ecl_assert_throw_decl(StandardException) {

	ecl_assert_throw( ( ( x >= origin[0] ) && (x <= limits[0]) ), StandardException(LOC,OutOfRangeError) );
	ecl_assert_throw( ( ( y >= origin[1] ) && (y <= limits[1]) ), StandardException(LOC,OutOfRangeError) );

	int cell_x = static_cast<int>( ( x - origin[0] )/scale_multiplier );
	int cell_y = static_cast<int>( ( y - origin[1] )/scale_multiplier );
	return cells[Cols*cell_y+cell_x];
}

/*****************************************************************************
** Implementation [GridMap][Conversions]
*****************************************************************************/

template <typename CellType, int Rows, int Cols>
linear_algebra::Vector2i GridMap<CellType,Rows,Cols>::cellCoordinates (const double& x, const double& y) const ecl_assert_throw_decl(StandardException) {

	ecl_assert_throw( ( ( x >= origin[0] ) && (x <= limits[0]) ), StandardException(LOC,OutOfRangeError) );
	ecl_assert_throw( ( ( y >= origin[1] ) && (y <= limits[1]) ), StandardException(LOC,OutOfRangeError) );

	linear_algebra::Vector2i cell_coordinates;
	cell_coordinates << static_cast<int>( ( x - origin[0] )/scale_multiplier ),
						static_cast<int>( ( y - origin[1] )/scale_multiplier );
	return cell_coordinates;
}

template <typename CellType, int Rows, int Cols>
ecl::linear_algebra::Vector2d GridMap<CellType,Rows,Cols>::worldCoordinates (const int& x, const int& y) const ecl_assert_throw_decl(StandardException) {

	ecl_assert_throw( ( ( x >= 0 ) && (x <= Cols) ), StandardException(LOC,OutOfRangeError) );
	ecl_assert_throw( ( ( y >= 0 ) && (y <= Rows) ), StandardException(LOC,OutOfRangeError) );

	linear_algebra::Vector2d world_coordinates;
	world_coordinates << origin[0] + scale_multiplier*static_cast<double>(x),
						 origin[1] + scale_multiplier*static_cast<double>(y);
	return world_coordinates;
}


} // namespace ecl

#endif /* ECL_MAPS_GRID_MAP_FIXED_HPP_ */


