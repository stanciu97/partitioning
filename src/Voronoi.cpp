#include "Voronoi.h"
#include "Spatial.h"
#include "SpatioTemporal.h"
#include "ConsecutiveRandoms.h"
#include <libqhullcpp/RboxPoints.h>
#include <libqhullcpp/QhullError.h>
#include <libqhullcpp/QhullQh.h>
#include <libqhullcpp/QhullFacet.h>
#include <libqhullcpp/QhullFacetSet.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullLinkedList.h>
#include <libqhullcpp/QhullVertex.h>
#include <libqhullcpp/QhullRidge.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <libqhullcpp/Qhull.h>

#include <vector>
#include <string>
#include <queue>
#include <algorithm>
#include <chrono>

using orgQhull::Qhull;
using orgQhull::QhullError;
using orgQhull::QhullFacet;
using orgQhull::QhullFacetSet;
using orgQhull::QhullFacetList;
using orgQhull::QhullQh;
using orgQhull::RboxPoints;
using orgQhull::QhullVertex;
using orgQhull::QhullVertexSet;
using orgQhull::QhullRidge;
using orgQhull::QhullRidgeSet;
using orgQhull::QhullRidgeSetIterator;


Voronoi::Voronoi(nodes& node, NodesDistance& distance, bool use_balance)
{
	this->node = &node;
	this->distance = &distance;
    this->balanced = use_balance;

    std::string points_3d("");

    //append all points in a string
    for (int i = 0; i < this->node->id.size(); i++)
    {
        points_3d += std::to_string(this->node->coord[i][0]) + " " + std::to_string(this->node->coord[i][1]) + " ";
        double half = (this->node->time_window[i][0] + this->node->time_window[i][1]) / 2.0;
        points_3d += std::to_string(half) + " ";
    }

    RboxPoints rbox;

    //contains: number of dimension (3d points), number of points, points 3d coord
    std::istringstream is(("3 " + std::to_string(this->node->id.size()) + " " + points_3d));

    //initialize rbox and qhull and compute Voronoi diagram
    rbox.appendPoints(is);
    std::stringstream output;
    this->qhull.runQhull(rbox, "v Qbb");   // "v Qbb" indicates to qhull to compute the Voronoi diagram

    // set correspondence in indexed_vertex between customer ids and vertex index used by qhull
    auto vertex = (this->qhull.vertexList()).toStdVector();

    this->indexed_vertex.resize(vertex.size());

    for (int i = 0; i < vertex.size(); i++)
    {
        this->indexed_vertex[vertex[i].point().id()] = vertex[i];
    }
}

std::vector<std::vector<int>> Voronoi::voronoi_part(int n_part)
{
   return this->voronoi_part(n_part, this->n_iter);
}

std::vector<std::vector<int>> Voronoi::voronoi_part(int n_part, int n_iter)
{
    //keep track about best solution
    double cost = 0;
    std::vector<std::vector<int>>* actual_groups = new std::vector<std::vector<int>>();

    for (int n_times = 0; n_times < this->n_iter; n_times++)
    {
        double temp_cost = 0;

        ConsecutiveRandoms<int> rand(1, this->node->id.size() - 1);

        //generate n_part different seeds
        std::set<int> partition;
        while (partition.size() < n_part)
        {
            partition.insert(rand.generate());
        }

        std::vector<std::list<int>> queue(n_part, std::list<int>());
        std::vector<std::vector<int>>* groups = new std::vector<std::vector<int>>(n_part, std::vector<int>());

        std::vector<bool> inserted(this->node->id.size(), false);

        //exclude depot from clusters
        inserted[0] = true;

        int index = 0;
        for (auto i = partition.begin(); i != partition.end(); i++)
        {
            queue[index].push_back(*i);
            groups->at(index).push_back(this->node->id[*i]);
            inserted[*i] = true;
            index++;
        }

        //number of already insterted elements 
        int total_size = n_part;
        int last_size = 0;

        int node_id;
        double min_distance;

        //if last_size equals total_size means no group has grown, so no voronoi neighbours are available (first phase)
        while (last_size != total_size)
        {
            last_size = total_size;

            //add element to a cluster
            for (int i = 0; i < groups->size(); i++)
            {
                temp_cost += this->grow_cluster(total_size, groups->at(i), inserted, queue[i]);
            }
        }

        //assign left elements to one of the existing groups, without adding to the actual groups
        std::vector<std::list<int>> assign_group;
        std::vector<int> gravity;
        this->queue_left(*groups, inserted, assign_group, gravity);

        //when possible elements are added as in first phase, otherwise left noded are added to the nearer group
        while (total_size < (this->node->id.size() - 1))
        {

            for (auto i = 0; i < n_part; i++)
            {
                //use assign_group to re-initialize the same procedure as in the first phase
                if (assign_group[i].size() > 0)
                {
                    auto front = assign_group[i].front();
                    if (!inserted[front])
                    {
                        queue[i].push_back(front);
                        groups->at(i).push_back(this->node->id[front]);
                        temp_cost += this->distance->get_distance(this->node->id[front], gravity[i]);
                        total_size++;
                        inserted[front] = true;
                    }
                    assign_group[i].pop_front();
                }
            }

            while (last_size != total_size)
            {
                last_size = total_size;

                for (int i = 0; i < groups->size(); i++)
                {
                    temp_cost += this->grow_cluster(total_size, groups->at(i), inserted, queue[i]);
                }
            }
        }

        if (n_times == 0)
        {
            cost = temp_cost;
            delete actual_groups;
            actual_groups = groups;
        }
        else if (temp_cost < cost)
        {
            cost = temp_cost;
            delete actual_groups;
            actual_groups = groups;
        }

    }

    return *actual_groups;
}

std::vector<std::vector<int>> Voronoi::voronoi_part_bubble(int n_part)
{
    std::vector<int> seed;

    std::vector<std::vector<int>>* groups = nullptr;
    std::pair<int, double> solution_cost;

    for (auto iter = 0; iter < 100; iter++)
    {

        seed = this->generate_seed(n_part);

        bool changed = true;
        std::vector<std::vector<int>>* first_group = new std::vector<std::vector<int>>(n_part, std::vector<int>());
        auto first_solution_cost = (this->balanced) ? this->balanced_partition(seed, *first_group) : this->strongest_partition(seed, *first_group);
        this->balance(seed, *first_group);
        std::pair<int, double> temp_cost;
        do
        {
            std::vector<std::vector<int>>* temp_groups = new std::vector<std::vector<int>>(n_part, std::vector<int>());
            
            temp_cost = (this->balanced) ? this->balanced_partition(seed, *temp_groups) : this->strongest_partition(seed, *temp_groups);

            if ((!this->balanced && temp_cost.first < first_solution_cost.first) ||(this->balanced && temp_cost.first < first_solution_cost.first && temp_cost.second < first_solution_cost.second))
            {
                delete first_group;
                first_group = temp_groups;
                first_solution_cost = temp_cost;
            }
            else
            {
                delete temp_groups;
            }

            changed = this->balance(seed, *temp_groups);
                
        } while (changed);

        if (iter == 0)
        {
            groups = first_group;
            solution_cost = first_solution_cost;
        }
        else if ( (!this->balanced && first_solution_cost.first < solution_cost.first) || (this->balanced && first_solution_cost.first < solution_cost.first && first_solution_cost.second < solution_cost.second))
        {
            delete groups;
            groups = first_group;
            solution_cost = first_solution_cost;
        }
        else
        {
            delete first_group;
        }

    }
    std::cout << solution_cost.first << " " << solution_cost.second <<" "<< this->max << std::endl;
    return *groups;
}

std::pair<int, double> Voronoi::strongest_partition(std::vector<int>& seed, std::vector<std::vector<int>>& group)
{
    auto n_part = seed.size();

    auto compare = [](std::pair<int, double> a, std::pair<int, double> b) {return a.first != b.first && a.second <= b.second;};
    std::vector<std::set<std::pair<int, double>, decltype(compare)>> candidate(n_part, std::set<std::pair<int, double>, decltype(compare)>(compare));

    std::vector<bool> inserted(this->node->id.size(), false);
  
    this->init_groups(inserted, seed, group, candidate);
    
    int size = 1;
    double cost = 0;

    while (size > 0)
    {
        int best_group = 0;
        int best_head = 0;
        double distance = 0;
        bool can_compare = false;

        size = 0;

        for (auto i = 0; i < candidate.size(); i++)
        {
            if (candidate[i].size() > 0)
            {
                auto temp_distance = candidate[i].begin()->second;

                if (!can_compare)
                {
                    best_group = i;
                    best_head = candidate[i].begin()->first;
                    distance = temp_distance;
                    can_compare = true;
                }
                else if (temp_distance < distance)
                {
                    best_group = i;
                    best_head = candidate[i].begin()->first;
                    distance = temp_distance;
                }
                size += candidate[i].size();
            }
        }

        if (!inserted[best_head] && size > 0)
        {
            cost += distance;
            group[best_group].push_back(best_head);
            candidate[best_group].erase(candidate[best_group].begin());
            inserted[best_head] = true;
            update_candidate(inserted, best_head, candidate[best_group]);
        }
        else if (size > 0)
        {
            candidate[best_group].erase(candidate[best_group].begin());
        }
    }

    for (auto i = 0; i < inserted.size(); i++)
    {
        if (!inserted[i])
        {
            auto min = this->distance->get_distance(seed[0], i);
            auto selected = 0;
            for (auto j = 1; j < seed.size(); j++)
            {
                auto temp_min = this->distance->get_distance(seed[j], i);
                if (temp_min < min)
                {
                    min = temp_min;
                    selected = j;
                }
            }
            group[selected].push_back(i);
            inserted[i] = true;
            cost += min;
        }
    }
    
    int min_partition = group[0].size();
    int max_partition = min_partition;
    for (auto i = 1; i < group.size(); i++)
    {
        auto temp_min = group[i].size();
        auto temp_max = temp_min;
        if (temp_min < min_partition)
            min_partition = temp_min;
        if (temp_max > max_partition)
            max_partition = temp_max;
    }

    return std::pair(max_partition - min_partition, cost);
}

std::pair<int, double> Voronoi::balanced_partition(std::vector<int>& seed, std::vector<std::vector<int>>& group)
{
    auto n_part = seed.size();

    auto compare = [](std::pair<int, double> a, std::pair<int, double> b) {return a.first != b.first && a.second <= b.second;};
    std::vector<std::set<std::pair<int, double>, decltype(compare)>> candidate(n_part, std::set<std::pair<int, double>, decltype(compare)>(compare));

    std::vector<bool> inserted(this->node->id.size(), false);

    this->init_groups(inserted, seed, group, candidate);

    int size = 1;
    double cost = 0;

    do
    {
        size = 0;
        for (auto i = 0; i < group.size(); i++)
        {
            if (candidate[i].size() > 0)
            {
                auto head = candidate[i].begin();
                if (!inserted[head->first])
                {
                    group[i].push_back(head->first);
                    inserted[head->first] = true;
                    update_candidate(inserted, head->first, candidate[i]);
                    cost += head->second;
                }
                candidate[i].erase(head);
                size += candidate[i].size();
            }
        }

    } while (size > 0);

    for (auto i = 0; i < inserted.size(); i++)
    {
        if (!inserted[i])
        {
            auto min = this->distance->get_distance(seed[0], i);
            auto selected = 0;
            for (auto j = 1; j < seed.size(); j++)
            {
                auto temp_min = this->distance->get_distance(seed[j], i);
                if (temp_min < min)
                {
                    min = temp_min;
                    selected = j;
                }
            }
            group[selected].push_back(i);
            inserted[i] = true;
            cost += min;
        }
    }

    int min_partition = group[0].size();
    int max_partition = min_partition;
    for (auto i = 1; i < group.size(); i++)
    {
        auto temp_min = group[i].size();
        auto temp_max = temp_min;
        if (temp_min < min_partition)
            min_partition = temp_min;
        if (temp_max > max_partition)
            max_partition = temp_max;
    }

    return std::pair(max_partition - min_partition, cost);
}

bool Voronoi::balance(std::vector<int>& seed, std::vector<std::vector<int>>& groups)
{
    double cost = 0;
    bool changed = false;

    for (auto i = 0; i < groups.size(); i++)
    {
        //set actual seed distance from al group members
        double distance = 0.0;
        for (auto j = 0; j < groups[i].size(); j++)
        {
            distance += this->distance->get_distance(groups[i][0], groups[i][j]);
        }

        //search for better seed
        for (auto j = 1; j < groups[i].size(); j++)
        {
            double temp_distance = 0.0;
            for (auto k = 0; k < groups[i].size(); k++)
            {
                temp_distance += this->distance->get_distance(groups[i][j], groups[i][k]);
            }
            if (temp_distance < distance)
            {
                distance = temp_distance;
                seed[i] = groups[i][j];
                changed = true;
            }
        }
        cost += distance;
    }

    return changed;
}

std::vector<int> Voronoi::find_neighbours(int id)
{
    auto facet = this->indexed_vertex[id].neighborFacets().toStdVector();

    std::vector<int> n;

    for (int i = 0; i < facet.size(); i++)
    {
        auto points = facet[i].vertices().toStdVector();

        for (int j = 0; j < points.size(); j++)
        {
            n.push_back(points.at(j).point().id());
        }
    }

    return n;
}

double Voronoi::grow_cluster(int &total_size, std::vector<int>& group, std::vector<bool>& inserted, std::list<int>& queue)
{

    //find voronoi neighbours of a point (customer)
    auto voronoi_neighbours = [this](std::set<int>& set, int point_id)
    {
        auto facet = this->indexed_vertex[point_id].neighborFacets().toStdVector();

        for (int i = 0; i < facet.size(); i++)
        {
            auto points = facet[i].vertices().toStdVector();

            for (int j = 0; j < points.size(); j++)
            {
                auto point = points.at(j).point().id();
                set.insert(point);
            }
        }
    };

    int node_id;
    double min_distance = 0;

    std::set<int> neighbours;
    auto size = queue.size();
    for (auto j = 0; j < size; j++)
    {
        auto head = queue.front();
        queue.pop_front();
        voronoi_neighbours(neighbours, head);
        bool have_distance = false;
        //select nearer customer between all neighbors exluding the already inserted ones
        for (auto k = neighbours.begin(); k != neighbours.end(); k++)
        {
            if (!inserted[*k])
            {
                auto temp = this->distance->get_distance(this->node->id[head], this->node->id[*k]);

                if (!have_distance)
                {
                    min_distance = temp;
                    node_id = *k;
                    have_distance = true;
                }
                else if (temp < min_distance)
                {
                    min_distance = temp;
                    node_id = *k;
                }
            }
        }
        //insert the valid customer to the cluster
        if (have_distance)
        {
            queue.push_back(node_id);
            group.push_back(this->node->id[node_id]);
            inserted[node_id] = true;
            total_size++;
        }
    }
           
    return min_distance;
}

void Voronoi::queue_left(std::vector<std::vector<int>>& groups, std::vector<bool>& inserted, std::vector<std::list<int>>& assign_group, std::vector<int>& gravity)
{
    auto group_distance = [this](int customer_id, std::vector<int>& group)
    {
        double acc_distance = 0;
        for (auto i = 0; i < group.size(); i++)
        {
            acc_distance += this->distance->get_distance(customer_id, group[i]);
        }
        return acc_distance;
    };

    //find gravity points of groups
    auto n_part = groups.size();
    gravity.reserve(n_part);
    for (auto i = 0; i < n_part; i++)
    {
        int candidate = groups[i][0];
        double actual_distance = group_distance(candidate, groups[i]);
        for (int j = 1; j < groups[i].size(); j++)
        {
            auto temp_distance = group_distance(groups[i][j], groups[i]);
            if (temp_distance < actual_distance)
            {
                actual_distance = temp_distance;
                candidate = groups[i][j];
            }
        }
        gravity.push_back(candidate);
    }

    for (int i = 0; i < n_part; i++)
    {
        assign_group.push_back(std::list<int>());
    }

    //assign each left element to one of the existing groups
    for (auto i = 1; i < this->node->id.size(); i++)
    {
        if (!inserted[i])
        {
            int group_id = 0;
            double actual_group_distance = this->distance->get_distance(this->node->id[i], gravity[group_id]);
            for (auto j = 1; j < n_part; j++)
            {
                auto temp_distance = this->distance->get_distance(this->node->id[i], gravity[j]);
                if (temp_distance < actual_group_distance)
                {
                    actual_group_distance = temp_distance;
                    group_id = j;
                }
            }
            assign_group[group_id].push_back(i);
        }
    }
}

std::vector<int> Voronoi::generate_seed(int n_part)
{
    ConsecutiveRandoms<int> rand(1, this->node->id.size() - 1);

    std::vector<int> seed;
    seed.reserve(n_part);

    std::set<int> partition;

    while (partition.size() < n_part)
    {
        auto complete = partition.insert(rand.generate());
        if (complete.second)
        {
            seed.push_back(*complete.first);
        }
    }

    return seed;
}

template <typename T>
void Voronoi::update_candidate(std::vector<bool>& inserted, int customer, T& candidate)
{
    auto neigh = this->find_neighbours(customer);
    for (auto i = 0; i < neigh.size(); i++)
    {
        if (!inserted[neigh[i]])
        {
            auto dist = this->distance->get_distance(customer, neigh[i]);
            auto add = candidate.insert(std::pair(neigh[i], dist));
            if (!add.second && add.first->second > dist)
            {
                candidate.erase(add.first);
                i--;
            }
        }
    }
}

template<typename T>
void Voronoi::init_groups(std::vector<bool>& inserted, std::vector<int>& seed, std::vector<std::vector<int>>& group, std::vector<T>& candidate)
{
    //exclude depot from clusters
    inserted[0] = true;

    for (auto i = 0; i < seed.size(); i++)
    {
        group[i].push_back(seed[i]);
        inserted[seed[i]] = true;
        update_candidate(inserted, seed[i], candidate[i]);
    }

}


