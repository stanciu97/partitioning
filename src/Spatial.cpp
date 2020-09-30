#include "Spatial.h"
#include <iostream>
#include <array>


Spatial::Spatial(std::string file) : NodesDistance::NodesDistance(file)
{
	this->init();
}

Spatial::Spatial(nodes &node) : NodesDistance::NodesDistance(node)
{
	this->init();
}

void Spatial::init()
{
	this->spatial_matrix.resize(this->size);

	//initialize each entry of the matrix with the Euclidian distance between pairs of customers
	for (auto i = 0; i < this->size; i++)
	{
		this->spatial_matrix[i].resize(size);
		for (auto j = 0; j < size; j++)
		{
			this->spatial_matrix[i][j] = euclidean_distance(i, j);
		}
	}
}

double Spatial::euclidean_distance(int from_id, int to_id)
{
	return std::sqrt(std::pow((this->node.coord[from_id][0] - this->node.coord[to_id][0]), 2) + std::pow((this->node.coord[from_id][1] - this->node.coord[to_id][1]), 2));
}

double Spatial::get_distance(int from_id, int to_id)
{
	return this->spatial_matrix[from_id][to_id];
}