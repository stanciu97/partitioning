#include "Spatial3d.h"
#include <iostream>
#include <array>

Spatial3d::Spatial3d(nodes& node) : NodesDistance::NodesDistance(node)
{
	this->spatial3d_matrix.resize(this->size);

	for (auto i = 0; i < this->size; i++)
	{
		this->spatial3d_matrix[i].resize(this->size);
		for (auto j = 0; j < this->size; j++)
		{
			this->spatial3d_matrix[i][j] = this->euclidean3d_distance(i, j);
		}
	}
}

double Spatial3d::euclidean3d_distance(int from_id, int to_id)
{
	auto temp = std::pow(this->node.coord[from_id][0] - this->node.coord[to_id][0] , 2);
	temp += std::pow(this->node.coord[from_id][1] - this->node.coord[to_id][1], 2);
	temp += std::pow((this->node.time_window[from_id][0] + this->node.time_window[from_id][1] / 2.0) -
		(this->node.time_window[to_id][0] + this->node.time_window[to_id][1] / 2.0), 2);
	return std::sqrt(temp);
}

double Spatial3d::get_distance(int from_id, int to_id)
{
	return this->spatial3d_matrix[from_id][to_id];
}