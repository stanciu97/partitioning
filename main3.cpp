#include <iostream>
#include <vector>
#include <array>
#include <fstream>
#include <string>
#include <chrono>
#include <set>
#include <random>
#include <ctime>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*
struttura contenente i dati letti da file:
	coord contiene la posizione di un cliente in coordinate x y;
	time_window contiene i vincoli temporali di ogni cliente;
	demand rappresenta la quantità di merce richiesta da un singolo cliente;
	service_time rappresenta il tempo necessario per completare un'operazione presso un cliente;
	Gli identificativi dei clienti corrispondoono alla posizione all'interno dei vettori, dunque le informazioni
	del cliente con id 1 saranno nella poszione 1 della struttura.
	vehicles rappresenta il numero di veicoli disponibili;
	capacity rappresenta la capacità di ogni veicolo;
	size invece è il numero totale di nodi (magazzino + clienti).

*/

struct nodes
{
	std::vector<std::array<int, 2>> coord;
	std::vector<std::array<int, 2>> time_window;
	std::vector<int> demand;
	std::vector<int> service_time;
	int vehicles;
	int capacity;
	int size;
};

class NodesDistance
{
public:
	NodesDistance();
	NodesDistance(std::string file);
	virtual double get_distance(int from_id, int to_id) = 0;
	int get_size();
protected:
	nodes node;
	int size = 0;
private:
	void resize(int new_size);
};

NodesDistance::NodesDistance() : NodesDistance("input.txt") {};

NodesDistance::NodesDistance(std::string file)
{
	std::ifstream input(file);
	if (input.is_open())
	{
		resize(100);
		int read = 0;
		std::string line;
		std::string word;
		while (std::getline(input, line) && read < 7)
		{
			if (read == 3)
			{
				input >> this->node.vehicles;
				input >> this->node.capacity;
			}
			read++;
		}

		read = 0;
		while (input >> word && (word.compare("") != 0))
		{
			input >> this->node.coord[read][0];
			input >> this->node.coord[read][1];
			input >> this->node.demand[read];
			input >> this->node.time_window[read][0];
			input >> this->node.time_window[read][1];
			input >> this->node.service_time[read];

			read++;
			if (read >= this->size)
				resize(size * 2);
		}
		if (this->size != read)
			resize(read);

		input.close();
	}
}

void NodesDistance::resize(int new_size)
{
	this->size = new_size;
	this->node.coord.resize(size);
	this->node.time_window.resize(size);
	this->node.demand.resize(size);
	this->node.service_time.resize(size);
}

int NodesDistance::get_size()
{
	return this->size;
}

class Spatial : public NodesDistance
{
public:
	Spatial();
	Spatial(std::string file);
	bool is_good();
	double get_distance(int from_id, int to_id) override;

protected:
	
	std::vector<std::vector<double>> spatial_matrix;
	
	double max_distance = 0;
	double min_distance = 0;

	double max_width = 0;

private:
	bool init = false;
	double euclidean_distance(int from_id, int to_id);
};

Spatial::Spatial() : Spatial("input.txt") {}

Spatial::Spatial(std::string file) : NodesDistance::NodesDistance(file)
{

	if (this->size > 0)
		this->init = true;

	this->spatial_matrix.resize(this->size);

	for (auto i = 0; i < this->size; i++)
	{
		this->spatial_matrix[i].resize(size);
		for (auto j = 0; j < size; j++)
		{
			this->spatial_matrix[i][j] = euclidean_distance(i, j);

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
}


double Spatial::euclidean_distance(int from_id, int to_id)
{
	return std::sqrt(std::pow((this->node.coord[from_id][0] - this->node.coord[to_id][0]), 2) + std::pow((this->node.coord[from_id][1] - this->node.coord[to_id][1]), 2));
}

bool Spatial::is_good()
{
	return this->init;
}

double Spatial::get_distance(int from_id, int to_id)
{
	return this->spatial_matrix[from_id][to_id];
}


class SpatioTemporal : public Spatial
{
public:
	SpatioTemporal();
	SpatioTemporal(std::string file);
	SpatioTemporal(std::string file, double k1, double k2, double k3);
	SpatioTemporal(std::string file, double k1, double k2, double k3, double alpha1);
	double get_distance(int from_id, int to_id) override;
	double get_spatial_distance(int from_id, int to_id);
	double get_temporal_distance(int from_id, int to_id);

private:
	double temporal_distance(int from, int to);
	double k1 = 1.0, k2 = 1.5, k3 = 2.0;
	double alpha1 = 0.5, alpha2 = 1.0 - alpha1;
	double max_time = 0;
	double min_time = 0;
	std::vector<std::vector<double>> temporal_matrix;
	std::vector<std::vector<double>> spatio_temporal_matrix;
};


SpatioTemporal::SpatioTemporal() : SpatioTemporal("input.txt") {}

SpatioTemporal::SpatioTemporal(std::string file) : SpatioTemporal(file, 1.0, 1.5, 2.0){}

SpatioTemporal::SpatioTemporal(std::string file, double k1, double k2, double k3) : SpatioTemporal(file, k1, k2, k3, 0.5){}


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

	this->temporal_matrix.resize(this->size);

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
				this->temporal_matrix[i][j] = std::min(this->temporal_distance(i, j), this->temporal_distance(j, i));				// nella ricerca è indicato il massimo come valore da prendere per la distanza temporale (che non è unidirezionale), i dati corrispondo prendendo però il minimo
				this->temporal_matrix[j][i] = this->temporal_matrix[i][j];

				if (temporal_matrix[i][j] < temp_min && i != 0 && j != 0)
				{
					temp_min = temporal_matrix[i][j];
				}

				if (temporal_matrix[i][j] > temp_max && i != 0 && j != 0)
				{
					temp_max = temporal_matrix[i][j];
				}
			}

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
	double marked_tw_from[2];
	auto tw_offset = double(this->node.service_time[from_id]) + this->spatial_matrix[from_id][to_id];
	marked_tw_from[0] = double(this->node.time_window[from_id][0]) + tw_offset;
	marked_tw_from[1] = double(this->node.time_window[from_id][1]) + tw_offset;

	double tw_to[2];
	tw_to[0] = double(this->node.time_window[to_id][0]);
	tw_to[1] = double(this->node.time_window[to_id][1]);
	

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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


class Nodes
{
public:
	Nodes();
	Nodes(std::string file);
	bool is_good();
	double get_distance(int from_id, int to_id);
	std::array<int, 2> get_time_window(int index);
	int get_size();
	double get_max_width();
	int get_service_time(int index);
	std::array<double, 2> get_max_min_distance();
private:
	void resize(int newSize);
	double euclidean_distance(int from_id, int to_id);
	std::vector<std::vector<double>> time_matrix;
	std::vector<std::array<int, 2>> coord;
	std::vector<std::array<int, 2>> time_window;
	std::vector<int> demand;
	std::vector<int> service_time;
	std::string path_name;
	int vehicles = 0;
	int capacity = 0;
	double max_time = 0;
	double min_time = 0;
	double max_width = 0;
	bool init = false;
	int size = 0;
};

Nodes::Nodes() : Nodes("input.txt") {};

Nodes::Nodes(std::string file)
{
	path_name.assign(file);
	std::ifstream input(path_name);
	if (input.is_open())
	{
		resize(100);
		init = true;
		int read = 0;
		std::string line;
		std::string word;
		while (std::getline(input, line) && read < 7)
		{
			if (read == 3)
			{
				input >> this->vehicles;
				input >> this->capacity;
			}
			read++;
		}

		read = 0;
		while (input >> word && (word.compare("") != 0))
		{
			input >> this->coord[read][0];
			input >> this->coord[read][1];
			input >> this->demand[read];
			input >> this->time_window[read][0];
			input >> this->time_window[read][1];
			input >> this->service_time[read];
			auto temp = this->time_window[read][1] - this->time_window[read][0];
			if (temp > this->max_width)
			{
				this->max_width = temp;
			}

			read++;
			if (read >= this->size)
				resize(size * 2);
		}
		if(this->size != read)
			resize(read);

		input.close();

		this->time_matrix.resize(size);

		for (auto i = 0; i < size; i++)
		{
			this->time_matrix[i].resize(size);
			for (auto j = 0; j < size; j++)
			{
				this->time_matrix[i][j] = euclidean_distance(i, j);

				if (i == 1 && j == 2)
				{
					this->max_time = this->time_matrix[i][j];
					this->min_time = this->max_time;
				}
				else if( i != j && i != 0 && j != 0)
				{
					if (this->time_matrix[i][j] > this->max_time)
						this->max_time = this->time_matrix[i][j];
					if (this->time_matrix[i][j] < this->min_time)
						this->min_time = this->time_matrix[i][j];
				}

			}

		}

	}
}

void Nodes::resize(int newSize)
{
		this->size = newSize;
		this->coord.resize(size);
		this->time_window.resize(size);
		this->demand.resize(size);
		this->service_time.resize(size);
}

bool Nodes::is_good()
{
	return this->init;
}

double Nodes::euclidean_distance(int from_id, int to_id)
{
	return std::sqrt(std::pow( (this->coord[from_id][0] - this->coord[to_id][0]) , 2) + std::pow( (this->coord[from_id][1] - this->coord[to_id][1]) , 2));
}

double Nodes::get_distance(int from_id, int to_id)
{
	return this->time_matrix[from_id][to_id];
}

double Nodes::get_max_width()
{
	return this->max_width;
}

int Nodes::get_service_time(int index)
{
	return this->service_time[index];
}

int Nodes::get_size()
{
	return this->size;
}

std::array<int, 2> Nodes::get_time_window(int index)
{
	return this->time_window[index];
}

std::array<double, 2> Nodes::get_max_min_distance()
{
	return { this->max_time, this->min_time };
}

/*class SpatioTemporal
{
public:
	SpatioTemporal();
	SpatioTemporal(std::string file);
	SpatioTemporal(std::string file, double k1, double k2, double k3);
	double get_temporal_distance(int from_id, int to_id);
	double get_spatio_temporal_distance(int from_id, int to_id);
	int get_size();
	Nodes nodes;
private:
	double temporal_distance(int from, int to);
	double k1 = 1.0, k2 = 1.5, k3 = 2.0;
	double alpha1 = 0.5, alpha2 = 0.5;
	int size = 0;
	double max_time = 0;
	double min_time = 0;
	std::vector<std::vector<double>> temporal_matrix;
	std::vector<std::vector<double>> spatio_temporal_matrix;
};

SpatioTemporal::SpatioTemporal() : SpatioTemporal("input.txt") {};

SpatioTemporal::SpatioTemporal(std::string file) : SpatioTemporal(file, 1.0, 1.5, 2.0)  {}

SpatioTemporal::SpatioTemporal(std::string file, double k1, double k2, double k3) : nodes(file)
{
	if (k1 < k2 && k2 < k3)
	{
		this->k1 = k1;
		this->k2 = k2;
		this->k3 = k3;
	}

	this->size = nodes.get_size();
	this->temporal_matrix.resize(this->size);
	
	auto temp_min = this->temporal_distance(1, 2);
	auto temp_max = temp_min;

	for (auto i = 0; i < size; i++)
	{

		for (auto j = i; j < size; j++)
		{
			if(i == 0)
				this->temporal_matrix[j].resize(this->size);

			if (i != j)
			{
				this->temporal_matrix[i][j] = std::min(this->temporal_distance(i, j), this->temporal_distance(j, i));				// nella ricerca è indicato il massimo come valore da prendere per la distanza (che non è unidirezionale), i dati corrispondo prendendo però il minimo
				this->temporal_matrix[j][i] = this->temporal_matrix[i][j];

				if(temporal_matrix[i][j] < temp_min && i != 0 && j != 0)
				{
					temp_min = temporal_matrix[i][j];
				}

				if(temporal_matrix[i][j] > temp_max && i != 0 && j != 0)
				{
					temp_max = temporal_matrix[i][j];
				}
			}

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
				auto distance = this->nodes.get_max_min_distance();
				this->spatio_temporal_matrix[i][j] = 
					this->alpha1 * (this->nodes.get_distance(i, j) - distance[1]) / (distance[0] - distance[1]) +
					this->alpha2 * (this->get_temporal_distance(i, j) - this->min_time) / (this->max_time - this->min_time);
			}

			else
			{
				this->spatio_temporal_matrix[i][j] = 0;
			}

		}
	}
	
}

double SpatioTemporal::temporal_distance(int from, int to)
{
	auto tw_from = this->nodes.get_time_window(from), tw_to = this->nodes.get_time_window(to);
	double marked_tw_from[2];
	marked_tw_from[0] = tw_from[0] + double(this->nodes.get_service_time(from)) + nodes.get_distance(from, to);
	marked_tw_from[1] = tw_from[1] + double(this->nodes.get_service_time(from)) + nodes.get_distance(from, to);

	auto early_arrival = [this, tw_to](double marked_t) 
	{
		return
		this->k2* (marked_t * marked_t) / 2.0 +
		this->k1 * tw_to[1] * marked_t -
		(
			this->k1 * tw_to[0] * marked_t +
			this->k2 * tw_to[0] * marked_t
		);
	};

	auto good_arrival = [this, tw_to](double marked_t)
	{
		return
		-this->k1 * (marked_t * marked_t) / 2.0 +
		this->k1 * tw_to[1] * marked_t;

	};

	auto late_arrival = [this, tw_to](double marked_t)
	{
		return
		this->k3 * -1.0 * (marked_t * marked_t) / 2.0 +
		this->k3 * tw_to[1] * marked_t;
	};

	return this->k1 * this->nodes.get_max_width() -
		(
			early_arrival(std::min(marked_tw_from[1], double(tw_to[0]))) - early_arrival(std::min(marked_tw_from[0], double(tw_to[0]))) +
			good_arrival( std::max(std::min(marked_tw_from[1], double(tw_to[1])), double(tw_to[0])) ) - good_arrival( std::min( std::max(marked_tw_from[0], double(tw_to[0])) , double(tw_to[1])) ) +
			late_arrival(std::max(marked_tw_from[1] , double(tw_to[1]))) - late_arrival(std::max(marked_tw_from[0] , double(tw_to[1])))
		) / ( marked_tw_from[1] - marked_tw_from[0] )
		;

	

}

double SpatioTemporal::get_temporal_distance(int from_id, int to_id)
{
	return this->temporal_matrix[from_id][to_id];
}

double SpatioTemporal::get_spatio_temporal_distance(int from_id, int to_id)
{
	return this->spatio_temporal_matrix[from_id][to_id];
}

int SpatioTemporal::get_size()
{
	return this->size;
}*/

class GeneticEvolution
{
public :
	GeneticEvolution(NodesDistance& nodes);
	GeneticEvolution(NodesDistance& nodes, int n_generations, int p_size, double crossover, double crossover_mutation, double candidate_mutation);
private:
	NodesDistance* nodes;
	int number_of_generations = 300;
	int population_size = 100;
	double crossover = 0.65;
	double crossover_mutation = 0.2;
	double candidate_mutation = 0.05;
};

GeneticEvolution::GeneticEvolution(NodesDistance& nodes) : GeneticEvolution(nodes, 300, 100, 0.65, 0.2, 0.05) {};

GeneticEvolution::GeneticEvolution(NodesDistance& nodes, int n_generations, int p_size, double crossover, double crossover_mutation, double candidate_mutation)
{
	this->nodes = &nodes;

	if (n_generations > 0)
		this->number_of_generations = n_generations;
	if (p_size > 0)
		this->population_size = p_size;
	if (crossover >= 0 && crossover <= 1)
		this->crossover = crossover;
	if (crossover_mutation >= 0 && crossover_mutation <= 1)
		this->crossover_mutation = crossover_mutation;
	if (candidate_mutation >= 0 && candidate_mutation <= 1)
		this->candidate_mutation = candidate_mutation;


}

class KMedoidMean
{
public :
	KMedoidMean(int n_groups, NodesDistance& nodes);
	KMedoidMean(int n_groups, NodesDistance& nodes, std::set<int> initial_nodes);


private:
	NodesDistance *nodes;
	std::uniform_int_distribution<int>* nodes_id_generator;
	std::default_random_engine* default_engine;
};


KMedoidMean::KMedoidMean(int n_groups, NodesDistance &nodes)
{
	this->nodes = &nodes;
	std::set<int> initial_nodes;

	this->default_engine = new std::default_random_engine(std::time(0));
	this->nodes_id_generator = new std::uniform_int_distribution<int> (1, this->nodes->get_size());

	while (initial_nodes.size() != n_groups)
	{
		initial_nodes.insert(this->nodes_id_generator->operator()(*this->default_engine));
	}
}

int main()
{
	/*auto start = std::chrono::high_resolution_clock::now();
	 
	SpatioTemporal prova("c1_10_1.txt");

	auto stop = std::chrono::high_resolution_clock::now();

	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

	std::cout << duration.count() << std::endl;

	std::cout << std::endl;

	std::cout << prova.get_size() << std::endl;

	for (int i = 1; i < prova.get_size(); i++)
	{

		for (int j = 1; j < prova.get_size(); j++)
		{
			std::cout << prova.nodes.get_distance(i, j) << " ";
		}
		std::cout << std::endl;

		for (int j = 1; j < prova.get_size(); j++)
		{
			std::cout << prova.get_temporal_distance(i, j) << " ";
		}
		std::cout << std::endl;

		for (int j = 1; j < prova.get_size(); j++)
		{
			//if(prova.get_spatio_temporal_distance(i,j) < 0 || prova.get_spatio_temporal_distance(i,j) >= 1)
				std::cout<<prova.get_spatio_temporal_distance(i, j)<<" ";
		}
		std::cout << std::endl;
		std::cout << std::endl;
	}
	*/

	SpatioTemporal prova("c1_10_1.txt");

	KMedoidMean genetic(6, prova);
	std::cout << "test \n";

	return 0;
}