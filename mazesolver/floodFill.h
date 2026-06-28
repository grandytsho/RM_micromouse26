// #include "one.h"
// #include "two.h"
// #include <EEPROM.h>
// #include <Arduino.h>
// #include <Wire.h>

// const int MAZE_SIZE = 6;

// struct Cell {
//     int distance; 
//     bool left = false, right = false, up = false, down = false;
//     bool visited = false;
//     int f = 9999;//total
//     int g = 9999;//normal cost
//     int h = 9999;//juristic cost
//     int parent_x = -1;
//     int parent_y = -1;
//     bool closed = false;
// };

// struct Pos {
//     int x, y;
// }; 

// struct AStarNode {
//     int x, y, f, h;
//     bool operator>(const AStarNode& other) const {
//         if (f == other.f) return h > other.h;
//         return f > other.f;
//     }
// };

// enum heading {North = 0 , East, South, West} ;

// void turnTo(int target_heading); 
// void copyMap(); 
// std::vector<int> getAStarPath(Cell maze[MAZE_SIZE][MAZE_SIZE]);
// void closeAll(Cell maze[MAZE_SIZE][MAZE_SIZE]); 
// void executeDiagonalSpeedrun(const std::vector<int>& path); 
// void floodfill();
// void calculateCost(std::vector<int> best_path);
// bool updateMap();
// void mousemove();
// int executeFloodFill();