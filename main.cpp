#//! user_eg3_r.cpp -- Invoke rbox and qhull from C++

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

#include <cstdio>   /* for printf() of help message */
#include <ostream>
#include <stdexcept>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <set>
#include <queue>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h> 
#include <list>
#include <math.h>

using std::cerr;
using std::cin;
using std::cout;
using std::endl;

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

struct node
{
    int id;
    int request;
    std::pair<int, int> coord;
    std::pair<int, int> tw;
    int wait = 0;
    int arrival = 0;
    int service;
};

/*int main(int argc, char** argv);*/

int travelTime(node& from, node& to);

int user_eg3(int argc, char** argv, std::string& s, std::vector<node>& nodes);

char prompt[] = "\n========\n\
user_eg3 commands... -- demonstrate calling rbox and qhull from C++.\n\
\n\
user_eg3 is statically linked to reentrant qhull.  If user_eg3 fails\n\
immediately, it is probably linked to the non-reentrant, qhull library.\n\
\n\
Commands:\n\
  eg-100                 Run the example in qh-code.htm\n\
  rbox \"200 D4\" ...    Generate points from rbox\n\
  qhull \"d p\" ...      Run qhull with options and produce output\n\
  qhull-cout \"o\" ...   Run qhull with options and produce output to cout\n\
  qhull \"T1\" ...       Run qhull with level-1 trace to cerr\n\
                         Use 'qhull-cout' for option 'Tz' (trace to cout)\n\
  facets                 Print facets when done\n\
\n\
For example\n\
  user_eg3 rbox qhull\n\
  user_eg3 rbox qhull T1\n\
  user_eg3 rbox qhull d\n\
  user_eg3 rbox \"10 D2\"  \"2 D2\" qhull  \"s p\" facets\n\
\n\
";


void voronoiNeighbors(std::vector<orgQhull::QhullVertex*>& vertex, std::set<int>& set, int pointId, int level, std::vector<bool>& routed);

void voronoiNeighbors(std::vector<orgQhull::QhullVertex*>& vertex, std::set<int>& set, int pointId, int level, std::vector<bool>& routed)
{
    std::queue<int> queue;
    std::set<int> done;
    queue.push(pointId);
    done.insert(pointId);
    auto len = queue.size();
    int ring = 0;
    int dequeued = 0;
    //immediate neighbors
    while (ring<level && !queue.empty())
    {
        auto id = queue.front();
        queue.pop();
        auto facet = vertex[id]->neighborFacets().toStdVector();
        for (int i = 0; i < facet.size(); i++)
        {
            auto points = facet[i].vertices().toStdVector();

            for (int j = 0; j < points.size(); j++)
            {
                auto point = points.at(j).point().id();
                if (!routed[point])
                {
                    set.insert(point);
                }
                if (done.find(point) == done.end())
                {
                    queue.push(point);
                    done.insert(point);
                }
            }
        }
        dequeued++;
        if (dequeued == len)
        {
            dequeued = 0;
            len = queue.size();
            ring++;
        }
    }

}

/*--------------------------------------------
-user_eg3-  main procedure of user_eg3 application
*/
int main(int argc, char** argv) {

    QHULL_LIB_CHECK;

   std::vector<double> v;

    std::string line;

    std::ifstream myfile("c1_10_1.txt");
    int i = 0;

    std::vector<node> nodes;
    int nnode = 0;

    std::string s("");

    if (myfile.is_open())
    {
        while (std::getline(myfile, line))
        {
            if (i > 7)
            {
                int j = 0;// 0 1 2 4-5 5->6
                std::string word;
                while (myfile >> word && (word.compare("") != 0))
                {
                    if (j == 0)
                    {
                        node n;
                        nodes.push_back(n);
                        nodes[nnode].id = nnode;
                    }
                    else if (j == 1)
                    {
                        s += (word + " ");
                        nodes[nnode].coord.first = stoi(word);
                    }
                    else if (j == 2)
                    {
                        s += (word + " ");
                        nodes[nnode].coord.second = stoi(word);
                    }
                    else if (j == 3)
                    {
                        nodes[nnode].request = std::stoi(word);
                    }
                    else if (j == 4)
                    {
                        std::string word2;
                        myfile >> word2;
                        s += (std::to_string((std::stod(word) + std::stod(word2)) / 2.0) + " ");
                        nodes[nnode].tw.first = std::stoi(word);
                        nodes[nnode].tw.second = std::stoi(word2);
                    }
                    else if (j == 5)
                    {
                        nodes[nnode].service = std::stoi(word);
                        nnode++;
                        j = -1;
                    }
                    j++;
                }
            }
            i++;
        }
        
        for (int i = 0; i < nodes.size(); i++)
        {
            cout << nodes[i].id << " " << nodes[i].request << " " << nodes[i].coord.first << " " << nodes[i].coord.second << " " << nodes[i].tw.first << " " << nodes[i].tw.second << " " << nodes[i].service << endl;
        }
    }
    else
    {
        cout << "ci sono problemi";
    }
    cout << s;



    if (argc == 1) {
        cout << prompt;
        return 0;
    }
    try {
        return user_eg3(argc, argv, s, nodes);
    }
    catch (QhullError& e) {
        cerr << e.what() << std::endl;
        return e.errorCode();
    }
}
int travelTime(node& from, node& to)
{
    return int(sqrt(pow(from.coord.first - to.coord.first, 2) + pow(from.coord.second - to.coord.second, 2)) + 0.5);
}
//main

int user_eg3(int argc, char** argv, std::string& s, std::vector<node>& nodes)
{
    if (strcmp(argv[1], "eg-100") == 0) {
        RboxPoints rbox("100");
        Qhull q(rbox, "");
        QhullFacetList facets = q.facetList();
        cout << facets;
        return 0;
    }
    bool printFacets = false;
    RboxPoints rbox;
    Qhull qhull;
    int readingRbox = 0;
    int readingQhull = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "rbox") == 0) {
            if (readingRbox != 0 || readingQhull != 0) {
                cerr << "user_eg3 -- \"rbox\" must be first" << endl;
                return 1;
            }
            readingRbox++;
        }
        else if (strcmp(argv[i], "qhull") == 0
            || strcmp(argv[i], "qhull-cout") == 0) {
            if (readingQhull) {
                cerr << "user_eg3 -- only one \"qhull\" or \"qhull-cout\" allowed." << endl;
                return 1;
            }
            if (strcmp(argv[i], "qhull-cout") == 0) {
                qhull.setOutputStream(&cout);
            }
            if (rbox.isEmpty()) {
                if (readingRbox) {
                    rbox.appendPoints("10 D2");
                }
                else {
                    std::istringstream is(("3 1001 " + s)); // To be passed to rbox: produces a unit square where 2 is the dimension 4 is the count and the points follow. This will also accept any valid rbox flags. 
                    std::stringstream output;
                    rbox.appendPoints(is);
                }
            }
            readingQhull++;
            readingRbox = 0;
        }
        else if (strcmp(argv[i], "facets") == 0) {
            printFacets = true;
        }
        else if (readingRbox) {
            readingRbox++;
            cerr << "rbox " << argv[i] << endl;
            rbox.appendPoints(argv[i]);
            if (rbox.hasRboxMessage()) {
                cerr << "user_eg3 " << argv[i] << " -- " << rbox.rboxMessage();
                return rbox.rboxStatus();
            }
        }
        else if (readingQhull) {
            if (readingQhull == 1) {
                qhull.runQhull(rbox, argv[i]);
                qhull.outputQhull();
            }
            else {
                qhull.outputQhull(argv[i]);
            }
            readingQhull++;
            if (qhull.hasQhullMessage()) {
                cerr << "\nResults of " << argv[i] << "\n" << qhull.qhullMessage();
                qhull.clearQhullMessage();
            }
        }
        else {
            cerr << "user_eg3 error: Expecting qhull, qhull-cout, or rbox.  Got " << argv[i] << endl;
            return 1;
        }
    }//foreach argv
    if (readingRbox) {
        cout << rbox;
        return 0;
    }
    if (readingQhull == 1) { // e.g., rbox 10 qhull
        qhull.runQhull(rbox, "");
        qhull.outputQhull();
        if (qhull.hasQhullMessage()) {
            cerr << "\nResults of qhull\n" << qhull.qhullMessage();
            qhull.clearQhullMessage();
        }
    }
    if (qhull.hasOutputStream()) {
        return 0;
    }
    if (printFacets) {
        QhullFacetList facets = qhull.facetList();
        cout << "\nFacets created by Qhull::runQhull()\n" << facets;
    }
    QhullFacetList prova = qhull.facetList();

   // QhullFacet facet = prova.first();
     
    //auto vec = rbox.toStdVector();
     
    /*for (int i = 0; i < vec.size(); i++)
    {
        //cout << vec[i] << endl;
    }*/

    //auto point = qhull.points();
    
    auto listaV = qhull.vertexList();
    std::vector<QhullVertex> vec1 =  listaV.toStdVector();
    auto vec0 = vec1[0];
    //cout << vec0 << endl;
    //cout<<listaV.contains(vec0);
    //auto points = qhull.points();
    //cout<<points.at(0).id();

    //cout << endl << vec1[10];
    auto facet = qhull.facetList();
    //cout<<facet.printVertices();
    auto vec = listaV.toStdVector();
    //cout<<points.at(0);
    //cout<<vec0.point();

    auto base = qhull.facetList();
    qhull.beginFacet();
    //auto face = base[0].qh.voronoiVertex();
    //cout<<base.toStdVector()[0].getCenter();
    auto vert = qhull.vertexList().toStdVector();
    
    //cout << endl << vert.at(71);
    //cout << base[1].ridges();
   // cout << endl << endl << base;

   // for(int i = 0; i< vec1.size(); i++)
    //{
      //  auto vecsupp = vec1[i].point();

//        cout << vec1[i] << endl;

       /* for (int j = 0; j < vecsupp.size(); j++)
        {
            cout << vecsupp[j] << endl;
        }
        cout << endl;*/

  //  }
    /*auto point = vertice.point();
    auto vec = point.toStdVector();
    cout << vertice << endl;
    for(int i = 0; i<vec.size(); i++)
    {
        cout << vec[i] << endl;
    }*/

    auto lista = qhull.vertexList();
    auto listafacet = qhull.facetList().toStdVector();
    //cout<<qhull.facetList()<<endl;
    lista.begin();
    auto vectorlista = lista.toStdVector();

 

     auto altro = listafacet[0].neighborFacets().toStdVector();
     //cout << altro[0];
   

   std::vector<orgQhull::QhullVertex*> indexedVertex(vectorlista.size());

   

    //cout <<vectorlista[0]<<endl<< vectorlista[0].id()<<endl<<vectorlista[0].point().id();

    for (int i = 0; i < indexedVertex.size(); i++)
    {
        indexedVertex[vectorlista[i].point().id()] = &vectorlista[i];
    }

   //cout << indexedVertex[0]->neighborFacets().toStdVector().size()<<endl;

    //cout << indexedVertex[1]->neighborFacets().toStdVector()[5].vertices();  //p1(v1001) p783(v998) p755(v955) p694(v899)
    //cout<<indexedVertex[0]->neighborFacets().at(0).ridges();

    

    std::vector<bool> routed(indexedVertex.size(), false);

    routed[0] = true;

   /* voronoiNeighbors(indexedVertex, neighbors, 0, 2, routed);

    for (int i = 0; i < routed.size(); i++)
    {
        //cout << routed[i] << endl;
    }

    for (auto it = neighbors.begin(); it != neighbors.end(); it++)
    {
        cout << endl<< *it << endl;
    }*/
  
    //cout<<r.size();

    std::vector<int> orderedTwStart(nodes.size());

    for (int i = 0; i < orderedTwStart.size(); i++)
    {
        orderedTwStart[i] = i;
    }
    
    std::sort(orderedTwStart.begin(), orderedTwStart.end(), [nodes](int a, int b) {return nodes[a].tw.first < nodes[b].tw.first;}); //ordino gli id dei nodi in base al contenuto del vettore nodes (dunque ho due vettori distinti)

    for (int i = 0; i < nodes.size(); i++)
    {
        cout << orderedTwStart[i]<<" "<<nodes[orderedTwStart[i]].tw.first<<" vettore originale: "<<" "<<nodes[i].id <<" "<<nodes[i].tw.first<< endl;
    }



    srand(time(NULL));

    std::vector<std::list<int>> routes;
    std::vector<int> inserted;
    bool gotNode = false;
    int actualNode = 0;

    std::cout << orderedTwStart.size() << endl << routed.size() << endl;

    std::set<int> neighbors;
    voronoiNeighbors(indexedVertex, neighbors, actualNode, 1, routed);
    auto maxDim = 5;//neighbors.size();

    /***************************************************/
    bool ok = true;

    std::vector<std::vector<int>> route5;
    std::vector<int> nextStart(maxDim, 0);

    int count = 0;
    for (auto it = neighbors.begin(); count < maxDim && it != neighbors.end(); it++)
    {
        std::vector<int> temp;
        temp.push_back(*it);
        route5.push_back(temp);
        routed[*it] = true;
        count++;
    }

    int iterazione = 0;

    int test0;
    bool newCand = false;
    bool keepGoing = true;
    int seen = 0;

    while (ok)
    {
        for (int i = 0; i < maxDim; i++)
        {
            std::set<int> neighbors2;

            count = 0;

            int actualNode = route5[i][iterazione];

            voronoiNeighbors(indexedVertex, neighbors2, actualNode, 2, routed);

            if (neighbors2.empty())
            {
                keepGoing = true;
            }

            else
            {
                auto random = rand() % (neighbors2.size());

                keepGoing = true;

                for (auto it = neighbors2.begin(); (keepGoing && it != neighbors2.end()); it++)
                {
                    if (nodes[actualNode].tw.first + nodes[actualNode].service < nodes[*it].tw.first && (nodes[*it].tw.first - nodes[actualNode].tw.first < 110 ))
                    {
                        actualNode = *it;
                        keepGoing = false;
                    }
                    else if (nextStart[i] == 0)
                    {
                        nextStart[i] = *it;
                        routed[*it] = true;
                    }
                    count++;
                }

                if (!keepGoing)
                {
                    route5[i].push_back(actualNode);
                    routed[actualNode] = true;
                }
            }

            if (keepGoing)
            {
                if (nextStart[i] != 0)
                {
                    std::cout << std::endl << "yes" << std::endl;
                    route5[i].push_back(nextStart[i]);
                    nextStart[i] = 0;
                    keepGoing = false;
                }

                for (auto j = seen; keepGoing && j < orderedTwStart.size(); (++seen, j++))
                {
                    if (!routed[orderedTwStart[j]])
                    {
                        actualNode = orderedTwStart[j];
                        route5[i].push_back(actualNode);
                        routed[actualNode] = true;
                        keepGoing = false;
                    }
                }

                if (seen == orderedTwStart.size())
                    ok = false;
            }
        }

        iterazione++;
    }

    std::cout << endl << iterazione << endl;


    for (int i = 0; i < route5.size(); i++)
    {
        for (int j = 0; j < route5[i].size(); j++)
        {
            std::cout << route5[i][j] << " ";
        }

        std::cout << std::endl;
    }

    std::ofstream myfile;
    myfile.open("example.txt");
    std::string input = { "input" };

    std::cout << "sizes:" << endl;

    /*int*/ count = 0;




    for (int i = 0; i < route5.size(); i++)
    {

        auto route = route5[i];
        std::cout << route.size() << std::endl;
        
            std::ofstream dataInput;
            dataInput.open(input + std::to_string(count) + ".txt");
            dataInput << endl << endl << endl << endl << "250 200" << endl << endl << endl << endl << endl << "0 250 250 0 0 1824 0" << endl;
            for (auto j = route.begin(); j != route.end(); j++)
            {
                myfile << *j << " ";

                dataInput << nodes[*j].id << " " << nodes[*j].coord.first << " " << nodes[*j].coord.second << " " << nodes[*j].request << " " << nodes[*j].tw.first << " " << nodes[*j].tw.second << " " << nodes[*j].service;
                dataInput << std::endl;

            }
            dataInput.close();
            count++;

        myfile << endl;
    }

    myfile.close();

    std::ofstream dataInputExtra;
    /*dataInputExtra.open(input + std::to_string(count) + ".txt");
    dataInputExtra << endl << endl << endl << endl << "250 200" << endl << endl << endl << endl << endl << "0 250 250 0 0 1824 0" << endl;
    for (int i = 0; i < routed.size(); i++)
    {
        if (!routed[i])
        {
            dataInputExtra << nodes[i].id << " " << nodes[i].coord.first << " " << nodes[i].coord.second << " " << nodes[i].request << " " << nodes[i].tw.first << " " << nodes[i].tw.second << " " << nodes[i].service;
            dataInputExtra << std::endl;
        }
    }


    dataInputExtra.close();*/



    return 0;

    /***************************************************/



    for (int i = 0; i < orderedTwStart.size(); i++)
    {
            actualNode = orderedTwStart[i];

            if (routed[actualNode])
                std::cout << actualNode << " risulta già inserito" << endl;

       if (!routed[actualNode])
       {
           std::list<int> route;
           int carico = 0;
           route.insert(route.begin(), actualNode);
           inserted.push_back(actualNode);
           nodes[actualNode].arrival = std::max(int(sqrt(pow(nodes[actualNode].coord.first - nodes[0].coord.first, 2) + pow(nodes[actualNode].coord.second - nodes[0].coord.second, 2)) + 0.5), nodes[actualNode].tw.first);
           nodes[actualNode].wait = nodes[0].tw.second - (nodes[actualNode].arrival + nodes[actualNode].service);
           carico += nodes[actualNode].request;
           routed[actualNode] = true;
           bool canRoute = true;
           while (canRoute)
           {
               std::set<int> neighbors;
               voronoiNeighbors(indexedVertex, neighbors, actualNode, 2, routed);
               int count = 0;
               
               if (neighbors.empty())
               {
                   gotNode = false;
                   canRoute = false;
               }

               else
               {
                   auto random = rand() % (neighbors.size());

                   for (auto it = neighbors.begin(); count <= random; it++)
                   {
                       if (random == count)
                       {
                           actualNode = *it;   
                       }
                       count++;
                   }

                   gotNode = true;

                   //try to insert before first node of the route

                   int arrival = std::max(travelTime(nodes[0], nodes[actualNode]), nodes[actualNode].tw.first);
                   auto toNext = travelTime(nodes[actualNode], nodes[route.front()]) + nodes[actualNode].service;
                   int wait;
                   int delta = -1;          //serve per capire quanto vale il minimo costo di inserimento
                   auto nextArrival = nodes[route.front()].arrival;
                   std::list<int>::iterator pos;            //tiene traccia della potenziale posizione di inserimento, -1 indica che non ci sono posizioni

                   arrival = std::max(arrival, nodes[actualNode].tw.first);
                   if (arrival >= nodes[actualNode].tw.first /*&& arrival <= nodes[actualNode].tw.second*/ && (arrival + toNext) <= nextArrival)
                   {
                       delta = arrival + toNext - nodes[actualNode].service;
                       pos = route.begin();
                       wait = nodes[route.front()].arrival - arrival - nodes[actualNode].service;
                   }

                   //try to insert before each node of the route except the first one 


                  auto iter = route.begin();
                  auto prec = iter;
                  iter++;
                  while (iter != route.end())
                  {
                      int newArrival = std::max(nodes[*prec].arrival + nodes[*prec].service + travelTime(nodes[*prec], nodes[actualNode]), nodes[actualNode].tw.first);
                      int needTime = newArrival - nodes[*prec].arrival - nodes[*prec].service + travelTime(nodes[actualNode], nodes[*iter]) + nodes[actualNode].service;
                      
                      if (newArrival >= nodes[actualNode].tw.first /*&& newArrival <= nodes[actualNode].tw.second*/ && needTime <= nodes[*prec].wait)
                      {
                          auto newDelta = needTime - nodes[actualNode].service;
                          if ( (delta == -1 && newDelta >= 0) || (delta != -1 && newDelta < delta))
                          {
                              delta = newDelta;
                              pos = iter;
                              arrival = newArrival;
                              wait = nodes[*iter].arrival - arrival - nodes[actualNode].service;

                          }
                      }
                      prec++;
                      iter++;
                  }

                   //try to insert after last node of the route

                  auto last = route.back();
                  int newArrival = std::max(nodes[last].arrival + nodes[last].service + travelTime(nodes[last], nodes[actualNode]), nodes[actualNode].tw.first);
                  int needTime = newArrival - nodes[last].arrival - nodes[last].service + travelTime(nodes[actualNode], nodes[0]) + nodes[actualNode].service;
                  if (newArrival >= nodes[actualNode].tw.first /*&& newArrival <= nodes[actualNode].tw.second*/ && needTime <= nodes[last].wait)
                  {
                      auto newDelta = needTime - nodes[actualNode].service;
                      if ( (delta == -1 && newDelta >= 0) || (delta != -1 && newDelta < delta))
                      {
                          delta = newDelta;
                          pos = route.end();
                          arrival = newArrival;
                          wait = nodes[0].tw.second - arrival - nodes[actualNode].service;
                      }
                  }


                  if (delta == -1 || carico > 24000)
                  {
                      canRoute = false;
                  }
                  else
                  {
                      route.insert(pos, actualNode);
                      inserted.push_back(actualNode);
                      routed[actualNode] = true;
                      nodes[actualNode].arrival = arrival;
                      nodes[actualNode].wait = wait;
                      carico += nodes[actualNode].request;

                      if (pos != route.begin())
                      {
                          pos--;
                          nodes[*pos].wait = nodes[actualNode].arrival - nodes[*pos].arrival - nodes[*pos].service;
                      }
                  }
               }
           }

           routes.push_back(route);
       }


    }


    //std::ofstream myfile;

    std::sort(inserted.begin(), inserted.end(), [](int a, int b) {return a < b;});



    myfile.open("example.txt");
    //std::string input = { "input" };

    std::cout << "sizes:" << endl;

    /*int*/ count = 0;

    std::list<int> extraRoute;

    //std::ofstream dataInputExtra;
    dataInputExtra.open("extra.txt");
    dataInputExtra << endl << endl << endl << endl << "250 200" << endl << endl << endl << endl << endl << "0 250 250 0 0 1824 0" << endl;

    for (int i = 0; i < routes.size(); i++)
    {
        
        auto route = routes[i];
        std::cout << route.size() << std::endl;
        if (route.size() > 100)
        {
            std::ofstream dataInput;
            dataInput.open(input + std::to_string(count) + ".txt");
            dataInput << endl << endl << endl << endl << "250 200" << endl << endl << endl << endl << endl << "0 250 250 0 0 1824 0" << endl;
            for (auto j = route.begin(); j != route.end(); j++)
            {
                myfile << *j << " ";
                
                dataInput << nodes[*j].id << " " << nodes[*j].coord.first << " " << nodes[*j].coord.second << " " << nodes[*j].request << " " << nodes[*j].tw.first << " " << nodes[*j].tw.second << " " << nodes[*j].service;
                dataInput << endl;
                
            }
            dataInput.close();
            count++;
        }

        else
        {
            for (auto j = route.begin(); j != route.end(); j++)
            {

                dataInputExtra << nodes[*j].id << " " << nodes[*j].coord.first << " " << nodes[*j].coord.second << " " << nodes[*j].request << " " << nodes[*j].tw.first << " " << nodes[*j].tw.second << " " << nodes[*j].service;
                dataInputExtra << endl;
            }
        }

        myfile << endl;
    }

    myfile.close();
    dataInputExtra.close();

    /*std::string newinput =  input + std::to_string(count) + ".txt" ;

    std::rename("extra.txt",  newinput.c_str());*/

    return 0;
}

//user_eg3

