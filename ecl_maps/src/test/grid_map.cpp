/**
 * @file /src/test/grid_map.cpp
 *
 * @brief Grid maps locked onto real world co-ordinate maps.
 *
 * @date January 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <iostream>
#include <gtest/gtest.h>
#include <ecl/linear_algebra.hpp>
#include <ecl/formatters/floats.hpp>
#include "../../include/ecl/maps/grid_map.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using std::cout;
using std::endl;
using ecl::RightAlign;
using ecl::Format;
using ecl::GridMap;
using ecl::linear_algebra::Vector2d;
using ecl::linear_algebra::Vector2i;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace maps {
namespace tests {

/*****************************************************************************
** Cells
*****************************************************************************/

class Cell {
public:
	Cell() : i(0), j(0) {};
	unsigned int i;
	unsigned int j;
};


} // namespace tests
} // namespace maps
} // namespace ecl

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::maps::tests::Cell;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(GridMapTests,allEggsInOneBasket) {
	// Haven't got around to running this properly through gtests yet.
	SUCCEED();
}
/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    Format<double> format; format.width(8); format.precision(2); format.align(RightAlign);

    cout << endl;
    cout << "***********************************************************" << endl;
    cout << "                 Construction" << endl;
    cout << "***********************************************************" << endl;
    cout << endl;

    std::cout << "Creating a 5x5 grid map over the world area [0.0,0.0]->[10.0,10.0]" << std::endl;
    GridMap<Cell,5,5> map(0.0,0.0,2.0);

    typedef GridMap<Cell,5,5> Map;

    cout << endl;
    cout << "***********************************************************" << endl;
    cout << "                  Iteration" << endl;
    cout << "***********************************************************" << endl;
    cout << endl;

    Map::iterator iter;
    int i = 0;
    int j = 0;
    for ( iter = map.begin(); iter != map.end(); ++iter ) {
    	iter->i = i; iter->j = j;
    	++j;
    	if ( j == map.cols() ) {
    		j = 0;
    		++i;
    	}
    }
    Map::const_iterator const_iter;
    std::cout << std::endl;
    for ( const_iter = map.begin(); const_iter != map.end(); ++const_iter ) {
    	std::cout << "[" << const_iter->i << "," << const_iter->j << "]" << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Front: " << "[" << map.front().i << "," << map.front().j << "]" << std::endl;
    std::cout << "Back: " << "[" << map.back().i << "," << map.back().j << "]" << std::endl;
    std::cout << std::endl;

    for ( j = map.rows(); j > 0; --j ) {
    	unsigned int index = j - 1;
    	for ( i = 0; i < map.cols(); ++i ) {
    	    std::cout << "[" << map(i,index).i << "," << map(i,index).j << "]";
    	}
    	std::cout << std::endl;
    }
    std::cout << std::endl;

    cout << endl;
    cout << "***********************************************************" << endl;
    cout << "                  Transform" << endl;
    cout << "***********************************************************" << endl;
    cout << endl;

    Vector2i cell_coords = map.cellCoordinates(1.5,2.3);
    std::cout << "Cell coordinates [1.5,2.3]->[" << cell_coords[0] << "," << cell_coords[1] << "]" << std::endl;

    Vector2d world_coords = map.worldCoordinates(2,3);
    std::cout << "World coordinates [2,3]   ->[" << world_coords[0] << "," << world_coords[1] << "]" << std::endl;

    cout << endl;
    cout << "***********************************************************" << endl;
    cout << "                      Passed" << endl;
    cout << "***********************************************************" << endl;
    cout << endl;

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}


