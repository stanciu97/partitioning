#include "SpatioTemporal.h"
#include <array>
#include <iostream>

SpatioTemporal::SpatioTemporal(std::string file) : Spatial(file)
{
	this->init();
}

SpatioTemporal::SpatioTemporal(nodes &node) : Spatial(node) 
{
	this->init();
}

SpatioTemporal::SpatioTemporal(std::string file, double k1, double k2, double k3, double alpha1) : Spatial(file)
{
	if (k1 < k2 && k2 < k3)
	{
		this->k1 = k1;
		this->k2 = k2;
		this->k3 = k3;
	}

	if (alpha1 >= 0.0 && alpha1 <= 1.0)
	{
		this->alpha1 = alpha1;
		this->alpha2 = 1.0 - alpha1;
	}

	this->init();
}

SpatioTemporal::SpatioTemporal(nodes &node, double k1, double k2, double k3, double alpha1) : Spatial(node)
{
	if (k1 < k2 && k2 < k3)
	{
		this->k1 = k1;
		this->k2 = k2;
		this->k3 = k3;
	}

	if (alpha1 >= 0.0 && alpha1 <= 1.0)
	{
		this->alpha1 = alpha1;
		this->alpha2 = 1.0 - alpha1;
	}

	this->init();
}

void SpatioTemporal::init()
{
	//find max, min euclidian distance and max time window width exluding the depot
	for (auto i = 0; i < this->size; i++)
	{
		for (auto j = 0; j < this->size; j++)
		{
			//excluding depot from best min and max spatial distance candidate
			if (i == 1 && j == 2)
			{
				this->max_distance = this->spatial_matrix[i][j];
				this->min_distance = this->max_distance;
			}
			else if (i != j && i != 0 && j != 0)
			{
				if (this->spatial_matrix[i][j] > this->max_distance)
					this->max_distance = this->spatial_matrix[i][j];
				if (this->spatial_matrix[i][j] < this->min_distance)
					this->min_distance = this->spatial_matrix[i][j];
			}
		}

		auto temp_width = this->node.time_window[i][1] - this->node.time_window[i][0];

		if (i == 0)
		{
			this->max_width = temp_width;
		}

		else if (temp_width > this->max_width)
		{
			this->max_width = temp_width;
		}

	}

	this->temporal_matrix.resize(this->size);

	//initial min and max temporal distance
	auto temp_min = this->temporal_distance(1, 2);
	auto temp_max = temp_min;

	for (auto i = 0; i < size; i++)
	{

		for (auto j = i; j < size; j++)
		{
			if (i == 0)
				this->temporal_matrix[j].resize(this->size);

			if (i != j)
			{
				//the distance is not unidirectional and the paper indicates the max as the right candidate, but results corrispond taking the min.
				this->temporal_matrix[i][j] = std::min(this->temporal_distance(i, j), this->temporal_distance(j, i));				
				this->temporal_matrix[j][i] = this->temporal_matrix[i][j];

				//excluding depot from best min and max temporal distance candidate
				if (temporal_matrix[i][j] < temp_min && i != 0 && j != 0)
				{
					temp_min = temporal_matrix[i][j];
				}

				if (temporal_matrix[i][j] > temp_max && i != 0 && j != 0)
				{
					temp_max = temporal_matrix[i][j];
				}
			}
			//main diagonal is 0 (there is no distance if client[i] = client[j])
			else
			{
				this->temporal_matrix[i][j] = 0.0;
			}
		}
	}
	this->max_time = temp_max;
	this->min_time = temp_min;

	this->spatio_temporal_matrix.resize(this->size);


	for (auto i = 0; i < this->size; i++)
	{
		spatio_temporal_matrix[i].resize(this->size);

		for (auto j = 0; j < this->size; j++)
		{
			if (i != j)
			{
				//spatiotemoral distance as sum of scaled and weighted spatial and temporal distance
				this->spatio_temporal_matrix[i][j] =
					this->alpha1 * (this->spatial_matrix[i][j] - this->min_distance) / (this->max_distance - this->min_distance) +
					this->alpha2 * (this->temporal_matrix[i][j] - this->min_time) / (this->max_time - this->min_time);
			}
			
			else
			{
				this->spatio_temporal_matrix[i][j] = 0;
			}
		}
	}
}

double SpatioTemporal::temporal_distance(int from_id, int to_id)
{
	//arrival time_window
	double marked_tw_from[2];
	auto tw_offset = double(this->node.service_time[from_id]) + this->spatial_matrix[from_id][to_id];

	/*arrival time_window considering client[from_id] service time and time_window plus travel time to client[to_id]
	 if [a;b] is client[from_id] time_window then [a';b'] is the arrival time_window where  
	 a' = a + service_time[from_id] + travel_time[from_id][to_id]
	 b' = b + service_time[from_id] + travel_time[from_id][to_id]
	 */
	marked_tw_from[0] = double(this->node.time_window[from_id][0]) + tw_offset;
	marked_tw_from[1] = double(this->node.time_window[from_id][1]) + tw_offset;

	//client[to_id] real time window 
	double tw_to[2];
	tw_to[0] = double(this->node.time_window[to_id][0]);
	tw_to[1] = double(this->node.time_window[to_id][1]);

	//arrival to the next client could be early, good or late

	auto early_arrival = [this, tw_to](double marked_t)
	{
		return
			this->k2 * (marked_t * marked_t) / 2.0 +
			this->k1 * tw_to[1] * marked_t -
			(
				this->k1 * tw_to[0] * marked_t +
				this->k2 * tw_to[0] * marked_t
				);
	};

	auto good_arrival = [this, tw_to](double marked_t)
	{
		return
			this->k1 * -1.0 * (marked_t * marked_t) / 2.0 +
			this->k1 * tw_to[1] * marked_t;

	};

	auto late_arrival = [this, tw_to](double marked_t)
	{
		return
			this->k3 * -1.0 * (marked_t * marked_t) / 2.0 +
			this->k3 * tw_to[1] * marked_t;
	};

	// combination of early, good and late arrival
	return this->k1 * this->max_width -
		(
			early_arrival(std::min(marked_tw_from[1], double(tw_to[0]))) - early_arrival(std::min(marked_tw_from[0], double(tw_to[0]))) +
			good_arrival(std::max(std::min(marked_tw_from[1], double(tw_to[1])), double(tw_to[0]))) - good_arrival(std::min(std::max(marked_tw_from[0], double(tw_to[0])), double(tw_to[1]))) +
			late_arrival(std::max(marked_tw_from[1], double(tw_to[1]))) - late_arrival(std::max(marked_tw_from[0], double(tw_to[1])))
			) / (marked_tw_from[1] - marked_tw_from[0])
		;
}

double SpatioTemporal::get_distance(int from_id, int to_id)
{
	return this->spatio_temporal_matrix[from_id][to_id];
}

double SpatioTemporal::get_spatial_distance(int from_id, int to_id)
{
	return this->spatial_matrix[from_id][to_id];
}

double SpatioTemporal::get_temporal_distance(int from_id, int to_id)
{
	return this->temporal_matrix[from_id][to_id];
}