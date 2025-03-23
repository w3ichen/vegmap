#include <math.h>
#include <queue>
#include <cstdio>
#include <unordered_set>
#include <iostream>
#include <unordered_map>
#include <cassert>
#include <memory>
#include <stdbool.h>

// Macro functions
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y - 1) * XSIZE + (X - 1))
#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif
#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

// Constants
#define NUMOFDIRS 8
enum HeuristicFunc
{
    euclidean,
    maxmin
};
HeuristicFunc heuristic_func = HeuristicFunc::maxmin;

class Astar_search
{

public:
    bool verbose;        // For debugging
    double weight = 1.0; // Weight for heuristic
    Astar_search(double weight = 1.0, bool verbose = false) : verbose(verbose), weight(weight)
    {
        // Constructor
        x_size = 0;
        y_size = 0;
        collision_thresh = 100.0;
    };

    bool search(int robotposeX, int robotposeY, int goalX, int goalY, int *map)
    {
        // With the graph and heuristic, do a forward A* search from start to goal node to get the shortest path

        if (verbose)
            printf("Starting A* search from (%d,%d) to (%d,%d)\n",
                   robotposeX, robotposeY, goalX, goalY);

        // Data structures
        // Open list as min-heap
        static std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_minHeap;
        // Closed set as unordered set for O(1) lookup
        static std::unordered_set<Node, NodeHash> closed_unorderedSet;

        // Create start node and add to open list
        Node start_node = {robotposeX, robotposeY, 0.0 /*g(s)*/};
        start_node.parent = nullptr;
        open_minHeap.push(start_node);
        node_map[start_node] = start_node;

        // (1) While OPEN != empty
        while (!open_minHeap.empty())
        {
            // (2) Remove s with the smallest f(s) = g(s) + h(s) from OPEN
            Node curr_node = open_minHeap.top();
            open_minHeap.pop();

            // Check if goal reached
            if (curr_node.x == goalX && curr_node.y == goalY)
            {
                backtrack_search_path(curr_node);

                // Found the goal node
                printf("[PASS] Found the goal node!!!\n");
                return 0; // 0=true
            }

            // Skip if already visited
            if (closed_unorderedSet.find(curr_node) != closed_unorderedSet.end())
                continue;
            // (3) Insert s into CLOSED
            closed_unorderedSet.insert(curr_node);

            // (4) For every successor s' of s such that s' not in CLOSED
            for (int dir = 0; dir < NUMOFDIRS; dir++)
            {
                int newx = curr_node.x + dX[dir];
                int newy = curr_node.y + dY[dir];

                if (!is_valid_node(newx, newy, closed_unorderedSet, map))
                    continue; // Skip if not valid

                Node new_node = {newx, newy};
                double h;

                // Get heuristic value
                if (heuristic_func == HeuristicFunc::maxmin)
                    h = maxmin_heuristic(newx, newy, goalX, goalY);
                else
                    h = euclidean_heuristic(newx, newy, goalX, goalY);

                double map_cost = map[GETMAPINDEX(newx, newy, x_size, y_size)];
                double new_g_s = curr_node.g_s + map_cost;
                // (5) if g(s') > c(s',s) + g(s)
                // If new_node exists in node_map
                if (node_map.find(new_node) != node_map.end())
                {
                    if (node_map[new_node].v_s < new_g_s)
                        continue; // Skip if new cost is greater than existing cost
                }

                // (6) g(s') = c(s',s) + g(s);
                node_map[new_node].g_s = new_g_s;
                node_map[new_node].v_s = new_g_s;
                new_node.g_s = new_g_s;
                new_node.h_s = h * weight;

                // (7) Insert s' into OPEN
                open_minHeap.push(new_node);
                node_map[new_node].parent = std::make_shared<Node>(curr_node);
            }
        }
        if (verbose)
            printf("[FAIL] Goal not found\n");
        return -1; // -1=false
    }

private:
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dY[NUMOFDIRS] = {-1, 0, 1, -1, 1, -1, 0, 1};
    int x_size, y_size;
    double collision_thresh;

    // Structs
    struct Node
    {
        // (x,y)
        int x;
        int y;
        double g_s = INFINITY; // g(s) = cost
        double h_s = INFINITY; // h(s) = heuristic
        double v_s = INFINITY; // v(s) = cached value
        // Parent node
        std::shared_ptr<Node> parent = nullptr;

        // Constructors
        Node() = default;
        Node(int x_, int y_) : x(x_), y(y_) {}
        Node(int x_, int y_, double g_s_) : x(x_), y(y_), g_s(g_s_) {}
        Node(int x_, int y_, double g_s_, double h_s_) : x(x_), y(y_), g_s(g_s_), h_s(h_s_) {}

        // Equality operator required for unordered_set
        bool operator==(const Node &other) const
        {
            return x == other.x && y == other.y;
        }
        // Greater than operator required for min-heap
        bool operator>(const Node &other) const
        {
            return (g_s + h_s) > (other.g_s + other.h_s); // Min-heap based on cost + heuristic
        }
    };
    // Hash function required for unordered_set
    struct NodeHash
    {
        std::size_t operator()(const Node &node) const
        {
            // Creates unique hash using XOR and bit shift
            return std::hash<int>()(node.x) ^ (std::hash<int>()(node.y) << 1);
        }
    };

    // Data structures
    std::vector<Node> search_path;
    std::unordered_map<Node, Node, NodeHash> node_map; // Stores node data for O(1) lookup

    void backtrack_search_path(Node &curr_node)
    {
        this->search_path = {}; // Reset

        // Reconstruct path
        Node current = curr_node;
        while (node_map[current].parent != nullptr)
        {
            // Insert to the front of the vector
            search_path.insert(search_path.begin(), current);
            current = *node_map[current].parent;
        }
    }

    bool is_valid_node(int newx, int newy, const std::unordered_set<Node, NodeHash> &closed_unorderedSet = {}, int *map)
    {
        // Check if node is visited
        bool visited = closed_unorderedSet.find({newx, newy}) != closed_unorderedSet.end();
        if (visited)
            return false;
        // Check if node is in map
        bool in_map = newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size;
        if (!in_map)
            return false;
        // Check if node is free
        int map_cost = map[GETMAPINDEX(newx, newy, x_size, y_size)];
        bool is_free = map_cost >= 0 && map_cost < collision_thresh;
        if (!is_free)
            return false;

        return true;
    }

    // HEURISTIC FUNCTIONS
    double euclidean_heuristic(int x1, int y1, int x2, int y2)
    {
        // Formula: sqrt((x1 - x2)^2 + (y1 - y2)^2)
        return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }
    double maxmin_heuristic(int x1, int y1, int x2, int y2)
    {
        // Formula: max(dx, dy) + 0.4 * min(dx, dy)
        double dx = abs(x1 - x2);
        double dy = abs(y1 - y2);
        return MAX(dx, dy) + 0.4 * MIN(dx, dy);
    }
};