/**
 * @file /include/ecl/maps/grid_map_commons.hpp
 *
 * @brief commons for gridmaps
 *
 * @date Dec 2010
 *
 * @author einstein
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_MAPS_GRID_MAP_COMMONS_HPP_
#define ECL_MAPS_GRID_MAP_COMMONS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/errors/compile_time_assert.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/linear_algebra.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Enums
*****************************************************************************/
/**
 * @brief A simple flag for denoting the storage type of the grid map.
 *
 * This simply denotes whether the gridmap is dynamic or fixed memory.
 * Currently it is only used under the hood on the Array container. Users
 * should not need to use this directly.
 */
enum GridMapStorageType {
    DynamicGridMapStorage = 0,/**< Dynamic **/
    FixedGridMapStorage = 1   /**< Fixed **/
};


/*****************************************************************************
** Interface [GrdiMap]
*****************************************************************************/
} // namespace ecl

#endif /* ECL_MAPS_GRID_MAP_COMMONS_HPP_ */


