#include "phys/world.h"
#include <cmath>

namespace phys{

void World::clear(){
    bodies_.clear();
    pairs_.clear();
    aabbs_.clear();
    manifolds_.clear();
    prevManifolds_.clear();
}

}