#include "one.h"
#include "two.h"
#include <Arduino.h>
#include <Wire.h>
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

extern float distance;

const int MAZE_SIZE = 8;
const int straight_cost = 1; 
const int turn_cost = 2; 

enum heading { North = 0, East, South, West };

struct Cell {
    int distance; 
    bool left = false, right = false, up = false, down = false;
    bool visited = false;
    int f = 9999;
    int g = 9999;
    int h = 9999;
    int parent_x = -1;
    int parent_y = -1;
    bool closed = false;
};

struct Pos {
    int x, y;
}; 

struct AStarNode {
    int x, y, f, h;
    heading dir; 
    bool operator>(const AStarNode& other) const {
        if (f == other.f) return h > other.h;
        return f > other.f;
    }
};


extern int cur_x; 
extern int cur_y; 
extern int start_x; 
extern int start_y; 
extern int goal_x; 
extern int goal_y; 

extern Cell maze[MAZE_SIZE][MAZE_SIZE]; 
extern Cell copy_maze[MAZE_SIZE][MAZE_SIZE]; 
extern heading current_heading;
extern bool returning_to_start;
extern int cells_visited;
extern int times_moved;
extern bool speedrun;


void turnTo(int target_heading); 
void copyMap(); 
void floodfill();
void calculateCost(std::vector<int> best_path);
bool updateMap();
void mousemove();
void closeAll(Cell maze[MAZE_SIZE][MAZE_SIZE]); 
std::vector<int> getAStarPath(Cell maze[MAZE_SIZE][MAZE_SIZE]);
void moveNormal(std::vector<int> path);
void moveFast(std::vector<int> path);
int executeFloodFill();

