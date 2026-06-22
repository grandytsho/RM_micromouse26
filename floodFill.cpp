#include "one.h"
#include "two.h"
#include <EEPROM.h>
#include <Arduino.h>
#include <Wire.h>

struct Cell {
    int distance; 
    bool left = false, right = false, up = false, down = false;
    bool visited = false;
    int f = 9999;//total
    int g = 9999;//normal cost
    int h = 9999;//juristic cost
    int parent_x = -1;
    int parent_y = -1;
    bool closed = false;
};

struct Pos {
    int x, y;
}; 

struct AStarNode {
    int x, y, f, h;
    bool operator>(const AStarNode& other) const {
        if (f == other.f) return h > other.h;
        return f > other.f;
    }
};

int cur_x = 0; 
int cur_y = 0; 
int start_x = 0; 
int start_y = 0; 

int goal_x = 7; 
int goal_y = 8; 

Cell maze[MAZE_SIZE][MAZE_SIZE]; 
Cell copy_maze[MAZE_SIZE][MAZE_SIZE]; 
heading current_heading = North;
bool returning_to_start = false;
int cells_visited = 0;
int times_moved = 0;
bool speedrun = false;


void turnTo(int target_heading) {
    int diff = target_heading - current_heading;
    if (diff == 1 || diff == -3) {
        turn(-90.0f);
    } else if (diff == -1 || diff == 3) {
        turn(90.0f);
    } else if (diff == 2 || diff == -2) {
        turn(180.0f)
    }
    current_heading = (heading)target_heading;
}

void copyMap()
{
    for(int i = 0; i<MAZE_SIZE; i++)
    {
        for(int j = 0; j<MAZE_SIZE; j++)
        {
            copy_maze[i][j] = maze[i][j]; 
        }
    }
}

std::vector<int> getAStarPath(Cell maze[MAZE_SIZE][MAZE_SIZE]) {
    for(int x = 0; x < MAZE_SIZE; x++) {
        for(int y = 0; y < MAZE_SIZE; y++) {
            maze[x][y].g = 9999;
            maze[x][y].f = 9999;
            maze[x][y].closed = false;
            maze[x][y].parent_x = -1;
            maze[x][y].parent_y = -1;
            maze[x][y].h = std::abs(x-goal_x) + std::abs(y-goal_y); 
        }
    }

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> pq;
    maze[0][0].g = 0;
    maze[0][0].f = maze[0][0].h;
    pq.push({0, 0, maze[0][0].f, maze[0][0].h, current_heading});

    while(!pq.empty()) {
        AStarNode current = pq.top();
        pq.pop();
        
        int cx = current.x;
        int cy = current.y;

        heading sim_heading = current.dir; 

        if(maze[cx][cy].closed) continue;
        maze[cx][cy].closed = true;

        if(!maze[cx][cy].up && cy < 15) {
            int g_value; 
            if(sim_heading == North)
                g_value = maze[cx][cy].g +  straight_cost; 
            else
                g_value = maze[cx][cy].g + turn_cost; 
            if(g_value < maze[cx][cy+1].g) {
                maze[cx][cy+1].g = g_value;
                maze[cx][cy+1].f = g_value + maze[cx][cy+1].h;
                maze[cx][cy+1].parent_x = cx;
                maze[cx][cy+1].parent_y = cy;
                pq.push({cx, cy+1, maze[cx][cy+1].f, maze[cx][cy+1].h, North});
            }
        }
        if(!maze[cx][cy].down && cy > 0) {
            int g_value; 
            if(sim_heading == South)
                g_value = maze[cx][cy].g + straight_cost; 
            else
                g_value = maze[cx][cy].g + turn_cost; 
            if(g_value < maze[cx][cy-1].g) {
                maze[cx][cy-1].g = g_value;
                maze[cx][cy-1].f = g_value + maze[cx][cy-1].h;
                maze[cx][cy-1].parent_x = cx;
                maze[cx][cy-1].parent_y = cy;
                pq.push({cx, cy-1, maze[cx][cy-1].f, maze[cx][cy-1].h, South});
            }
        }
        if(!maze[cx][cy].right && cx < 15) {
            int g_value; 
            if(sim_heading == East)
                g_value = maze[cx][cy].g + straight_cost; 
            else
                g_value = maze[cx][cy].g + turn_cost; 
            if(g_value < maze[cx+1][cy].g) {
                maze[cx+1][cy].g = g_value;
                maze[cx+1][cy].f = g_value + maze[cx+1][cy].h;
                maze[cx+1][cy].parent_x = cx;
                maze[cx+1][cy].parent_y = cy;
                pq.push({cx+1, cy, maze[cx+1][cy].f, maze[cx+1][cy].h, East});
            }
        }
        if(!maze[cx][cy].left && cx > 0) {
            int g_value; 
            if(sim_heading == West)
                g_value = maze[cx][cy].g + straight_cost;
            else
                g_value = maze[cx][cy].g + turn_cost; 
            if(g_value < maze[cx-1][cy].g) {
                maze[cx-1][cy].g = g_value;
                maze[cx-1][cy].f = g_value + maze[cx-1][cy].h;
                maze[cx-1][cy].parent_x = cx;
                maze[cx-1][cy].parent_y = cy;
                pq.push({cx-1, cy, maze[cx-1][cy].f, maze[cx-1][cy].h, West});
            }
        }
    }

    std::vector<int> path;
    if(goal_x != -1) {
        int px = goal_x, py = goal_y;
        while(px != 0 || py != 0) { 
            int prx = maze[px][py].parent_x;
            int pry = maze[px][py].parent_y;
            if(py > pry) path.push_back(North);
            else if(px > prx) path.push_back(East);
            else if(py < pry) path.push_back(South);
            else if(px < prx) path.push_back(West);
            px = prx;
            py = pry;
        }
        std::reverse(path.begin(), path.end());
    }
    return path;
}

void closeAll(Cell maze[MAZE_SIZE][MAZE_SIZE]) {
    for(int x = 0; x < MAZE_SIZE; x++) {
        for(int y = 0; y < MAZE_SIZE; y++) { 
            if(!maze[x][y].visited) {
                maze[x][y].up = true; 
                maze[x][y].down = true; 
                maze[x][y].left = true; 
                maze[x][y].right = true;
                if(y < 15) maze[x][y+1].down = true;
                if(y > 0)  maze[x][y-1].up = true;
                if(x < 15) maze[x+1][y].left = true;
                if(x > 0)  maze[x-1][y].right = true;
            }
        }
    }
}

void executeDiagonalSpeedrun(const std::vector<int>& path) {
    int n = path.size();
    int i = 0;

    while (i < n) {
        if (i + 1 < n && path[i] != path[i+1]) {
            int first = path[i];
            int second = path[i+1];
            int diff = std::abs(first - second);
            if (diff == 1 || diff == 3) {
                int pairs = 0;
                int j = i;
                while (j + 1 < n && path[j] == first && path[j+1] == second) {
                    pairs++;
                    j += 2;
                }
                times_moved += pairs;
                if (pairs > 0) {
                    turnTo(first); 
                    int turn_diff = second - first;
                    bool turn_right = (turn_diff == 1 || turn_diff == -3);
                    API::moveForwardHalf();
                    if (turn_right) API::turnRight45();
                    else API::turnLeft45();
                    int half_diagonals = (pairs * 2) - 1;
                    for (int k = 0; k < half_diagonals; k++) {
                        API::moveForwardHalf();
                    }
                    if (turn_right) API::turnRight45();
                    else API::turnLeft45();
                    API::moveForwardHalf();
                    current_heading = (heading)second;
                    i = j; 
                    continue;
                }
            }
        }
        turnTo(path[i]);
        int straight_count = 1;
        while (i + 1 < n && path[i] == path[i+1]) {
            straight_count++;
            i++;
        }
        API::moveForward(straight_count);
        times_moved += straight_count; 
        i++;
    }
}

void floodfill() {   
    std::queue<Pos> q; 
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            maze[i][j].distance = MAZE_SIZE * MAZE_SIZE; 
        }
    }
    if(returning_to_start) {
        maze[start_x][start_y].distance = 0;
        q.push({0,0}); 
    } else {
        maze[goal_x][goal_y].distance = 0; 
        q.push({goal_x,goal_y}); 
    }

    while (!q.empty()) {
        int cx = q.front().x;
        int cy = q.front().y;
        q.pop();
        int cur_dist = maze[cx][cy].distance; 

        if((!maze[cx][cy].up) && cy < 15) {
            if(maze[cx][cy+1].distance > cur_dist + 1) {
                maze[cx][cy+1].distance = cur_dist + 1; 
                q.push({cx, cy+1}); 
            }
        }
        if((!maze[cx][cy].down) && cy > 0) {
            if(maze[cx][cy-1].distance > cur_dist + 1) {
                maze[cx][cy-1].distance = cur_dist + 1; 
                q.push({cx, cy-1}); 
            }
        }
        if((!maze[cx][cy].right) && cx < 15) {
            if(maze[cx+1][cy].distance > cur_dist + 1) {
                maze[cx+1][cy].distance = cur_dist + 1; 
                q.push({cx+1, cy}); 
            }
        }
        if((!maze[cx][cy].left) && cx > 0) {
            if(maze[cx-1][cy].distance > cur_dist + 1) {
                maze[cx-1][cy].distance = cur_dist + 1; 
                q.push({cx-1, cy}); 
            }
        }
    }
}

void calculateCost(std::vector<int> best_path) {   
    int total_cost = 0; 
    for(size_t i = 0; i < best_path.size() - 1; i++) {
        int first = best_path.at(i);  
        int second = best_path.at(i+1); 
        if(first == second) total_cost += straight_cost; 
        else total_cost += turn_cost; 
    }
    BT.println("The cost of final path is: " + std::to_string(total_cost)); 
}

bool updateMap() {
    bool left = isWallLeft(); 
    bool right = isWallRight(); 
    bool front  = isWallFront();
    bool mapUpdated = false;
    bool n = false, s = false, e = false, w = false; 

    if(current_heading == North) { n = front; e = right; w = left; }
    else if(current_heading == South) { s = front; e = left; w = right; }
    else if(current_heading == East) { e = front; n = left; s = right; }
    else if(current_heading == West) { w = front; n = right; s = left; }

    if(n && !maze[cur_x][cur_y].up) {
        maze[cur_x][cur_y].up = true; 
        if((cur_y < 15) && !maze[cur_x][cur_y+1].down) {
            maze[cur_x][cur_y+1].down = true; 
        }
        mapUpdated = true; 
    }
    if(s && !maze[cur_x][cur_y].down) {
        maze[cur_x][cur_y].down = true; 
        if((cur_y > 0) && !maze[cur_x][cur_y-1].up) {
            maze[cur_x][cur_y-1].up = true; 
        }
        mapUpdated = true; 
    }
    if(e && !maze[cur_x][cur_y].right) {
        maze[cur_x][cur_y].right = true; 
        if((cur_x < 15) && !maze[cur_x+1][cur_y].left) {
            maze[cur_x+1][cur_y].left = true; 
        }
        mapUpdated = true; 
    }
    if(w && !maze[cur_x][cur_y].left) {
        maze[cur_x][cur_y].left = true; 
        if((cur_x > 0) && !maze[cur_x-1][cur_y].right) {
            maze[cur_x-1][cur_y].right = true; 
        }
        mapUpdated = true; 
    }
    return mapUpdated;
}

void mousemove() {   
    int min_dist = MAZE_SIZE * MAZE_SIZE + 1; 
    heading target_heading = current_heading;
    std::vector<int> v;

    if(!maze[cur_x][cur_y].visited) {
        maze[cur_x][cur_y].visited = true;
        cells_visited++;
    } 

    if(!maze[cur_x][cur_y].up && cur_y < 15) {
        int dist = maze[cur_x][cur_y+1].distance;
        if(dist < min_dist) {   
            min_dist = dist; 
            v.clear();
            v.push_back(North); 
        }
    }
    if(!maze[cur_x][cur_y].down && cur_y > 0) {
        int dist = maze[cur_x][cur_y-1].distance;
        if(dist < min_dist) {   
            min_dist = dist; 
            v.clear();
            v.push_back(South); 
        }
    }
    if(!maze[cur_x][cur_y].right && cur_x < 15) {
        int dist = maze[cur_x+1][cur_y].distance;
        if(dist < min_dist) {   
            min_dist = dist; 
            v.clear();
            v.push_back(East); 
        }
    }
    if(!maze[cur_x][cur_y].left && cur_x > 0) {
        int dist = maze[cur_x-1][cur_y].distance;
        if(dist < min_dist) {   
            min_dist = dist; 
            v.clear();
            v.push_back(West); 
        }
    }

    if (!v.empty()) {
        int min_turn = 4;
        for(size_t i = 0 ; i < v.size(); i++) {
            int turn_diff = std::abs((int)current_heading - v[i]);
            if (turn_diff == 3) turn_diff = 1; 
            if (turn_diff < min_turn) {
                min_turn = turn_diff;
                target_heading = (heading)v[i];
            }
        }
    }

    int diff = target_heading - current_heading;
    if (diff == 1 || diff == -3) {
        turn(-90.0f);
    } else if (diff == -1 || diff == 3) {
        turnLeft(90.0f);
    } else if (diff == 2 || diff == -2) {
       turn(-180);
    }
    
    current_heading = target_heading;
    centerUntilDistance(25);
    times_moved++; 
    
    if (current_heading == North) cur_y++;
    else if (current_heading == East) cur_x++;
    else if (current_heading == South) cur_y--;
    else if (current_heading == West) cur_x--;
}

int executeFloodFill() {  
    BT.print("Initializing flood fill"); 
    floodfill(); 
    while(maze[cur_x][cur_y].distance != 0) {
        if(updateMap()) {
            floodfill(); 
        }
        mousemove(); 
    }

    BT.print("goal reached!");
    BT.print("Number of cells visited: " + std::to_string(cells_visited));
    BT.print("Number of moves: " + std::to_string(times_moved)); 

    if(!maze[cur_x][cur_y].visited)
    {
        maze[cur_x][cur_y].visited = true;
        cells_visited++; 
    }
    
    copyMap(); 
    closeAll(copy_maze);

    returning_to_start = false; 
    times_moved = 0; 
    speedrun = true; 

    std::vector<int> path = getAStarPath(copy_maze); 

    //executeDiagonalSpeedrun(path); 
    
    BT.print("Final moves taken: " + std::to_string(times_moved));
    calculateCost(path); 

    return 0; 
}