#pragma once
#include <vector>
#include <string>
#include <array>

/**
* contains all the informations about a VRPTW instance
* 
* each vector's size is equal to the number of customers + depot
* 
*/
struct nodes
{
	std::vector<int> id;										 
	std::vector<std::array<int, 2>> coord;						
	std::vector<std::array<int, 2>> time_window;				
	std::vector<int> demand;
	std::vector<int> service_time;
	int vehicles = 0;
	int capacity = 0;
};


/**
* initializes a struct nodes based on the data of a file. The file is
* expected to have the same layout of Solomon's instances. 
* 
* input:
* node: reference to the to be initialized structure
* file: file name as "name.extension"
* 
*/
void init_nodes(nodes &node, std::string file);


/**
* initializes a struct node for a sub problem based on an already initialized struct node
* 
* input
* node: reference to the already initialized structure
* sub_nodes: reference to the to be initialized structure
* sub_id: contains the customers' ids of the desired sub problem
* 
*/
void init_sub_nodes(nodes &node, nodes &sub_nodes, std::vector<int>& sub_id);


/**
* virtual and base class for the to be implemented variants of distance between customers
* 
* Its aim is to allow the same algorithm to run with different kinds of distance without changing the code. 
* It also makes easier the comparison between the same algorithm with different distances. Also adding
* a new distance adds no changes to the existing code
* 
*/
class NodesDistance
{
public:
	/**
	* constructor from file
	* 
	* input
	* file: file name as "name.extension"
	*
	*/
	NodesDistance(std::string file);

	/**
	* constructor from struct nodes
	*
	* input
	* nodes: reference to an existing struct nodes
	*
	*/
	NodesDistance(nodes& nodes);

	/*
	* virtual function for the to be implemented distance
	* 
	* input:
	* from_id: customer id
	* tp_id: customer id
	* 
	* output:
	* distance between customer[from_id] and customer[to_id]
	* 
	*/
	virtual double get_distance(int from_id, int to_id) = 0;

	/**
	* size of the problem
	* 
	* output:
	* number of customers + number of depots
	*/
	int get_size();

protected:
	nodes node;
	int size = 0;
};

