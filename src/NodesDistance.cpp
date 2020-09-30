#include "NodesDistance.h"
#include <fstream>

void init_nodes(nodes &node, std::string file)
{
	int size = 0;

	//resize all vectors in the structure according to the new_size parameter; 
	auto resize = [&size, &node](int new_size)
	{
		size = new_size;
		node.id.resize(size);
		node.coord.resize(size);
		node.time_window.resize(size);
		node.demand.resize(size);
		node.service_time.resize(size);
	};

	std::ifstream input(file);
	if (input.is_open())
	{
		resize(100);
		//number of lines read
		int read = 0;
		std::string line;
		std::string word;

		while (std::getline(input, line) && read < 7)
		{
			if (read == 3)
			{
				input >> node.vehicles;
				input >> node.capacity;
			}
			read++;
		}

		//number of lines read which is equal to the number of read customers
		read = 0;
		while (input >> word && (word.compare("") != 0))
		{
			node.id[read] = std::stoi(word);
			input >> node.coord[read][0];
			input >> node.coord[read][1];
			input >> node.demand[read];
			input >> node.time_window[read][0];
			input >> node.time_window[read][1];
			input >> node.service_time[read];

			read++;
			if (read >= size)
				resize(size * 2);
		}
		if (size != read)
			resize(read);

		input.close();
	}
}

void init_sub_nodes(nodes& node, nodes& sub_nodes, std::vector<int>& sub_id)
{
	//number of ids of the sub-problem (depot is not included)
	auto size = sub_id.size();

	sub_nodes.vehicles = node.vehicles;
	sub_nodes.capacity = node.capacity;

	//reserve (size + 1) space for all vectors in the sub_nodes structure. The extra one is needed for the depot 
	sub_nodes.id.reserve(size + 1);
	sub_nodes.coord.reserve(size + 1);
	sub_nodes.time_window.reserve(size + 1);
	sub_nodes.demand.reserve(size + 1);
	sub_nodes.service_time.reserve(size + 1);

	//insert depot informations in 0. Initial problem has depot in 0, so do the sub-problems
	sub_nodes.id.push_back(node.id[0]);
	sub_nodes.coord.push_back(node.coord[0]);
	sub_nodes.time_window.push_back(node.time_window[0]);
	sub_nodes.demand.push_back(node.demand[0]);
	sub_nodes.service_time.push_back(node.service_time[0]);

	//insert customers information in the structure
	for (int i = 0; i < size; i++)
	{
		sub_nodes.id.push_back(sub_id[i]);
		sub_nodes.coord.push_back(node.coord[sub_id[i]]);
		sub_nodes.time_window.push_back(node.time_window[sub_id[i]]);
		sub_nodes.demand.push_back(node.demand[sub_id[i]]);
		sub_nodes.service_time.push_back(node.service_time[sub_id[i]]);
	}
}

NodesDistance::NodesDistance(std::string file)
{
	init_nodes(this->node, file);
	this->size = node.id.size();
}

NodesDistance::NodesDistance(nodes& nodes)
{
	this->node = nodes;
	this->size = node.id.size();
}

int NodesDistance::get_size()
{
	return this->size;
}




