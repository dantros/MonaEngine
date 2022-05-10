#include "EnvironmentData.hpp"
#include <limits>
#include <algorithm>
#include "../Core/Log.hpp"
#include "../Utilities/FuncUtils.hpp"
namespace Mona{        

    // debe haber pasado test de withinBoundaries
    float EnvironmentData::_getTerrainHeight(float x, float y, HeightMap terrain) {
        MONA_ASSERT(terrain.withinBoundaries(x, y), "Vertex must be within boundaries of the mesh");
        return 0;
    }

}