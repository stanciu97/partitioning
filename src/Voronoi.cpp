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
#include <set>
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


Voronoi::Voronoi(nodes& node, NodesDistance& distance)
{
	this->node = &node;
	this->distance = &distance;

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
