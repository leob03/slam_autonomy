#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

// NOTE: Changed all doubles to floats (also in .hpp)!

mbot_lcm_msgs::path2D_t search_for_path(mbot_lcm_msgs::pose2D_t start,
                                             mbot_lcm_msgs::pose2D_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    cell_t startCell = global_position_to_grid_cell(Point<float>(start.x, start.y), distances);
    cell_t goalCell = global_position_to_grid_cell(Point<float>(goal.x, goal.y), distances);
    
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    Node* startNode = new Node(startCell.x, startCell.y);
    Node* goalNode = new Node(goalCell.x, goalCell.y);

    mbot_lcm_msgs::path2D_t path;
    path.utime = start.utime;

    PriorityQueue open;
    std::vector<Node*> closed;
    std::vector<cell_t> path_cells;

    std::vector<Node*> children;
    bool found_path = false;

    startNode -> h_cost = h_cost(startNode, goalNode, distances);
    startNode -> g_cost = 0.0;
    open.push(startNode);

    Node* current = open.pop();


    int loop_count = 0;
    while(!found_path && loop_count < 100000){ // attempt 100000 times
        loop_count++;

        closed.push_back(current);
        children = expand_node(current, distances, params);

        for (Node* child : children){
            float g = current -> g_cost + g_cost(current, child, distances, params);
            
            if (open.is_member(child)){
                // child is in open
                Node* open_child = open.get_member(child);
                if (g < open_child -> g_cost) {
                    open_child -> parent = current;
                    open_child -> g_cost = g; // child has parent that reaches it with lower accumulated cost then prior parent
                }
            }
            else{
                // child is not in open
                child -> g_cost = g;
                child -> h_cost = h_cost(child, goalNode, distances);
                child -> parent = current;
                if (std::find(closed.begin(), closed.end(), child) == closed.end()) {
                    open.push(child);
                }
            }
        }

        current = open.pop();
        if (current -> cell == goalNode -> cell) {
            found_path = true;
        }
        
    }


    if (found_path)
    {
        auto nodePath = extract_node_path(goalNode, startNode);
        path.path = extract_pose_path(nodePath, distances);
        // Remove last pose, and add the goal pose
        path.path.pop_back();
        path.path.push_back(goal);
    }

    else printf("[A*] Didn't find a path\n");
    path.path_length = path.path.size();
    return path;
}



float h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{

    float dx = abs(goal -> cell.x - from -> cell.x);
    float dy = abs(goal -> cell.y - from -> cell.y);

    if (dx <= dy) {
        return (dx + dy) + ((1.41421356 - 2) * dx);
    }
    return (dx + dy) + ((1.41421356 - 2) * dy);

}
float g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    float g_cost = 0.0;

    float dx = abs(from -> cell.x - goal -> cell.x);
    float dy = abs(from -> cell.y - goal -> cell.y);
    float distance = distances(goal -> cell.x, goal -> cell.y);
    
    if (distance > params.minDistanceToObstacle && distance < params.maxDistanceWithCost) {
        g_cost = pow(params.maxDistanceWithCost - distance, params.distanceCostExponent);
    }

    if (dx == 1.0 & dy == 1.0){
        g_cost += 1.41421356; // diagonal
    } else{
        g_cost += 1;
    }

    return g_cost;

}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    std::vector<Node*> children;

    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};

    int child_x;
    int child_y;

    for (int i = 0; i < 8; i++) {
        child_x = node->cell.x + xDeltas[i];
        child_y = node->cell.y + yDeltas[i];

        if (distances.isCellInGrid(child_x, child_y)) {
            float distance = distances(child_x, child_y);
            if (distance > params.minDistanceToObstacle) {
                // only consider child not too close to obstacle
                Node *child = new Node(child_x, child_y);
                children.push_back(child);
            }
        }
    }

    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    std::vector<Node*> path;

    Node *current = goal_node;
    while (current -> parent != nullptr) {
        path.push_back(current);
        current = current -> parent;
    }
    path.push_back(current);

    // return path; // without prune
    return prune_node_path(path); // prune
}
// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose2D_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    std::vector<mbot_lcm_msgs::pose2D_t> path;
    bool no_heading = true;
    for (int i = 0; i < nodes.size(); i++){
        Point<float> global_pose = grid_position_to_global_position(nodes[i] -> cell, distances);
        mbot_lcm_msgs::pose2D_t pose = {0};
        pose.x = global_pose.x;
        pose.y = global_pose.y;
        if (no_heading) {
            pose.theta = 0.0;
            no_heading = false;
        }
        else {
            pose.theta = atan2(pose.y - path.back().y, pose.x - path.back().x) + path.back().theta;
        }
        path.push_back(pose);
    }
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;

}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath)
{
    std::vector<Node*> new_node_path;

    if (nodePath.size() < 3) {
        return nodePath;
    }

    new_node_path.push_back(nodePath[0]); // push start
    int hook = 1;
    while (hook + 1 < nodePath.size()) {
        if ((nodePath[hook + 1] -> cell.x - nodePath[hook] -> cell.x != nodePath[hook] -> cell.x - nodePath[hook - 1] -> cell.x)
            || (nodePath[hook + 1] -> cell.y - nodePath[hook] -> cell.y != nodePath[hook] -> cell.y - nodePath[hook - 1] -> cell.y)) {
            new_node_path.push_back(nodePath[hook]);
        }
        hook++;
    }
    new_node_path.push_back(nodePath[hook]); // push goal

    return new_node_path;
}
