#pragma once
#include "NodesDistance.h"

/**
* function that makes a partition of the customers using a Voronoi diagram.
* 
* input:
* nodes: reference to an initialized struct node containing the informations about the VRPTW instance
* n_part: number of seeds used. It is not equal to the number of final groups of the partition
* find_distance: reference to a NodesDistance istance, determines the distance usend in the algorithm
* 
* output:
* n_part or n_part + 1 groups forming the partition
* 
*/
std::vector<std::vector<int>> voronoi_part(nodes& nodes, int n_part, NodesDistance &find_distance, int level);

std::vector<std::vector<int>> voronoi_partV2(nodes& nodes, int n_part, NodesDistance& find_distance, int level);


/**
* function that uses voronoi_part() on groups that are too large, creating more groups.
* The number of seeds implicitly tell groups size, groups that are larger then this
* size are paritioned again
* 
* input:
* nodes: reference to an initialized struct node containing the informations about the VRPTW instance
* n_part: number of seeds used. It is not equal to the number of final groups of the partition
* find_distance: reference to a NodesDistance istance, determines the distance usend in the algorithm
*
* output:
* partition containing groups of acceptable size
*/
std::vector<std::vector<int>> iterative_voronoi_part(nodes& nodes, int n_part, NodesDistance &find_distance, int level);
