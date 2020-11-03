#pragma once
#pragma once
#include "NodesDistance.h"

/**
* NodesDistance's derived class
* distance is implemented as Euclidean distance
*
*/
class Spatial3d : public NodesDistance
{
public:
	
	/**
	* constructor from struct nodes
	*
	* input
	* node: reference to an existing struct nodes
	*
	*/
	Spatial3d(nodes& node);

	/**
	* implements NodesDistance's virtual function
	*
	* input:
	* from_id: customer id
	* to_id: customer id
	*
	* output:
	* euclidian distance between customer[from_id] and customer[to_id]
	*
	*/
	double get_distance(int from_id, int to_id) override;

protected:

	//square matrix containing the distances between all pairs of customers
	std::vector<std::vector<double>> spatial3d_matrix;

private:
	double euclidean3d_distance(int from_id, int to_id);
};