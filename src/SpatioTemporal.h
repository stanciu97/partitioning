#pragma once
#include "Spatial.h"

/**
* Spatial's derived class.
* distance is implemented as "spatiotemporal", both spatial and temporal
* informations are used for the definition of this distance.
* 
*/

class SpatioTemporal : public Spatial
{
public:

	/**
	* constructor from file
	* input
	* file: file name as "name.extension"
	*
	*/
	SpatioTemporal(std::string file);

	/**
	* constructor from struct nodes
	*
	* input
	* node: reference to an existing struct nodes
	*
	*/
	SpatioTemporal(nodes &node);

	/**
	* constructors that change the default parameters
	* 
	* input:
	* node or file : initialize problem's structure
	* k1: multiplier in case of arrival during the time window, should be 1
	* k2: multiplier in case of early arrival, should be greater then k1. It means that there is a penalty in case of early arrival, associated to the waste of time
	* k3: multiplier in case of late arrival, the customer can not be served. The penalty rate should be higher then k2
	* alpha1: multiplier associated with the spatial_distance, total distance is alpha1 * spatial_distance + alpha2 * temporal_distance
	*		  alpha1 + alpha2 should be 1. If alpha1 > alpha2 means that spatial_distance is more important in total_distance
	* 
	*/
	SpatioTemporal(nodes &node, double k1, double k2, double k3, double alpha1);
	SpatioTemporal(std::string file, double k1, double k2, double k3, double alpha1);

	/**
	* ovverides Spatial's get_distance function
	* 
	* input:	
	* from_id: customer id 
	* to_id: customer id
	* 
	* output:
	* spatiotemporal distance beteeen customer[from_id] and customer[to_id]
	* 
	*/
	double get_distance(int from_id, int to_id) override;

	/**
	* Spatial's get_distance funtion
	* 
	* input:
	* from_id: customer id 
	* to_id: customer id
	* 
	* output:
	* Euclidian distance beteeen customer[from_id] and customer[to_id]
	* 
	*/
	double get_spatial_distance(int from_id, int to_id);

	/**
	* Temporal distance
	*
	* input:
	* from_id: customer id
	* to_id: customer id
	*
	* output:
	* Temporal distance beteeen customer[from_id] and customer[to_id] based on good, eary and late arrival
	*
	*/
	double get_temporal_distance(int from_id, int to_id);

private:
	void init();
	double temporal_distance(int from, int to);
	double k1 = 1.0, k2 = 1.5, k3 = 2.0;
	double alpha1 = 0.5, alpha2 = 1.0 - alpha1;

	//max and min temporal distance between customers (depot is excluded), necessary for scaling the values in [0;1]
	double max_time = 0;
	double min_time = 0;

	//max and min spatial distance between customers (depot is exluded), necessary for scaling the values in [0;1]
	double max_distance = 0;
	double min_distance = 0;

	//max time_window width
	double max_width = 0;

	//square matrix containing the temporal distances between all pairs of customers
	std::vector<std::vector<double>> temporal_matrix;

	//square matrix containing the spatiotemporal distances between all pairs of customers
	std::vector<std::vector<double>> spatio_temporal_matrix;
};
