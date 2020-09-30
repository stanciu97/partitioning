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
#include <list>

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


std::vector<std::vector<int>> iterative_voronoi_part(nodes& nodes, int n_part, NodesDistance &find_distance)
{
    //define threshold. If the size of a cluster is greater then the threshold, it is not accepted
    int medium_size = (int)(nodes.id.size() / n_part);
    medium_size += (int)(medium_size * 0.3);

    std::vector<std::vector<int>> accepted_part;

    //inital partition
    auto temp_part = voronoi_part(nodes, n_part, find_distance);
 
    for (int i = 0; i < temp_part.size(); i++)
    {
        auto size = temp_part[i].size();

        if (size > medium_size)
        {
            struct::nodes temp_nodes;
            init_sub_nodes(nodes, temp_nodes, temp_part[i]);
            auto re_part = voronoi_part(temp_nodes, 2, find_distance);
            temp_part.reserve(re_part.size());
            //new clusters must be checked, so they are added to the initial partion
            temp_part.insert(temp_part.end(), re_part.begin(), re_part.end());
        }
        else
        {
            accepted_part.push_back(temp_part[i]);
        }
    }

   return accepted_part;
}

std::vector<std::vector<int>> voronoi_part(nodes &nodes, int n_part, NodesDistance &find_distance)
{
    //find voronoi neighbours of a point (customer)
    auto voronoiNeighbors = [](std::vector<orgQhull::QhullVertex*>& vertex, std::set<int>& set, int point_id)
    {
        auto facet = vertex[point_id]->neighborFacets().toStdVector();

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

    std::string points_3d("");

    //append all points in a string
    for (int i = 0; i < nodes.id.size(); i++)
    {
        points_3d += std::to_string(nodes.coord[i][0]) + " " + std::to_string(nodes.coord[i][1]) + " ";
        double half = (nodes.time_window[i][0] + nodes.time_window[i][1]) / 2.0;
        points_3d += std::to_string(half) + " ";
    }

    RboxPoints rbox;
    Qhull qhull;

    //contains: number of dimension (3d points), number of points, points 3d coord
    std::istringstream is(("3 " + std::to_string(nodes.id.size()) + " " + points_3d));
       
    //initialize rbox and qhull and compute Voronoi diagram
    rbox.appendPoints(is);
    std::stringstream output;
    qhull.runQhull(rbox, "v Qbb");   // "v Qbb" indicates to qhull to compute the Voronoi diagram

    // set correspondence in indexed_vertex between customer ids and vertex index used by qhull
    auto vertex = (qhull.vertexList()).toStdVector();

    std::vector<orgQhull::QhullVertex*> indexed_vertex(vertex.size());

    for (int i = 0; i < indexed_vertex.size(); i++)
    {
        indexed_vertex[vertex[i].point().id()] = &vertex[i];
    }

    ConsecutiveRandoms<int> rand(1, nodes.id.size() - 1);

    //generate n_part different seeds
    std::set<int> partition;
    while (partition.size() < n_part)
    {
        partition.insert(rand.generate());
    }

    std::vector<std::list<int>> queue(n_part, std::list<int>());
    std::vector<std::vector<int>> groups(n_part, std::vector<int>());

    std::vector<bool> inserted(nodes.id.size(), false);

    //exclude depot from clusters
    inserted[0] = true;

    int index = 0;
    for (auto i = partition.begin(); i != partition.end(); i++)
    {
        queue[index].push_back(*i);
        groups[index].push_back(nodes.id[*i]);
        inserted[*i] = true;
        index++;
    }

    int total_size = n_part;
    int last_size = 0;

    int node_id;
    double min_distance;

    while (last_size != total_size)
    {
        last_size = total_size;

        //search voronoi neighbors of each seed
        for (int i = 0; i < groups.size(); i++)
        {
            std::set<int> neighbors;
            auto size = queue[i].size();
            for (auto j = 0; j < size; j++)
            {
                auto head = queue[i].front();
                queue[i].pop_front();
                voronoiNeighbors(indexed_vertex, neighbors, head);
                bool have_distance = false;
                //select nearer customer between all neighbors exluding the already inserted ones
                for (auto k = neighbors.begin(); k != neighbors.end(); k++)
                {
                    if (!inserted[*k])
                    {
                        if (!have_distance)
                        {
                            min_distance = find_distance.get_distance(nodes.id[head], nodes.id[*k]);
                            node_id = *k;
                            have_distance = true;
                        }
                        else
                        {
                            auto temp = find_distance.get_distance(nodes.id[head], nodes.id[*k]);
                            if (temp < min_distance)
                            {
                                min_distance = temp;
                                node_id = *k;
                            }
                        }
                    }
                }
                //insert the valid customer to the cluster
                if (have_distance)
                {
                    queue[i].push_back(node_id);
                    groups[i].push_back(nodes.id[node_id]);
                    inserted[node_id] = true;
                    total_size++;
                }
            }
        }
    }

    //Define a new cluster made of exluded customers 
    if (total_size < nodes.id.size())
    {
        groups.push_back(std::vector<int>());
        for (int i = 0; i < nodes.id.size(); i++)
        {
            if (!inserted[i])
            {
                inserted[i] = true;
                groups[groups.size() - 1].push_back(nodes.id[i]);
            }
        }
    }

    return groups;
}
