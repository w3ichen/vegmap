#ifndef ASTAR_SEARCH_H
#define ASTAR_SEARCH_H

// Enum for heuristic function selection
enum HeuristicFunc
{
    euclidean,
    maxmin
};

// A* search algorithm class
class Astar_search
{
public:
    // Constructor
    Astar_search(double weight = 1.0, bool verbose = false);

    // Main search method
    bool search(int robotposeX, int robotposeY, int goalX, int goalY, int *map);
};

#endif // ASTAR_SEARCH_H