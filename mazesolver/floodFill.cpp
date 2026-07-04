#include <vector>
#include "floodFill.h"

float distance = 25; 

int cur_x = 0; 
int cur_y = 0; 
int start_x = 0; 
int start_y = 0; 

int goal_x = 1; 
int goal_y = 0; 

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
        turn(180.0f); 
    }
    current_heading = (heading)target_heading;
}

void copyMap() {
    for(int i = 0; i < MAZE_SIZE; i++) {
        for(int j = 0; j < MAZE_SIZE; j++) {
            copy_maze[i][j] = maze[i][j]; 
        }
    }
}

void floodfill() {   
    std::queue<Pos> q; 
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            maze[i][j].distance = MAZE_SIZE * MAZE_SIZE; 
        }
    }
        maze[goal_x][goal_y].distance = 0; 
        q.push({goal_x, goal_y}); 

    while (!q.empty()) {
        int cx = q.front().x;
        int cy = q.front().y;
        q.pop();
        int cur_dist = maze[cx][cy].distance; 

        if((!maze[cx][cy].up) && cy < (MAZE_SIZE - 1)) {
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
        if((!maze[cx][cy].right) && cx < (MAZE_SIZE - 1)) {
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
    BT.println("The cost of final path is: " + String(total_cost)); 
}

bool updateMap() {
    bool left = isWallLeft(); 
    bool right = isWallRight(); 
    bool front = isWallFront();
    bool mapUpdated = false;
    bool n = false, s = false, e = false, w = false; 

    // 1. Log current position and raw sensor data
    BT.print("updateMap at (");
    BT.print(cur_x); BT.print(","); BT.print(cur_y);
    BT.print(") | Heading: ");
    BT.println(current_heading);
    BT.println("Sensors -> L: " + String(left) + " | F: " + String(front) + " | R: " + String(right));

    // 2. Translate relative sensors to absolute cardinal directions
    if(current_heading == North) { n = front; e = right; w = left; }
    else if(current_heading == South) { s = front; e = left; w = right; }
    else if(current_heading == East) { e = front; n = left; s = right; }
    else if(current_heading == West) { w = front; n = right; s = left; }

    // 3. Update the maze arrays and log ONLY newly discovered walls
    if(n && !maze[cur_x][cur_y].up) {
        maze[cur_x][cur_y].up = true; 
        if((cur_y < (MAZE_SIZE - 1)) && !maze[cur_x][cur_y+1].down) {
            maze[cur_x][cur_y+1].down = true; 
        }
        mapUpdated = true; 
        BT.println("  -> Discovered NEW Wall: NORTH");
    }
    if(s && !maze[cur_x][cur_y].down) {
        maze[cur_x][cur_y].down = true; 
        if((cur_y > 0) && !maze[cur_x][cur_y-1].up) {
            maze[cur_x][cur_y-1].up = true; 
        }
        mapUpdated = true; 
        BT.println("  -> Discovered NEW Wall: SOUTH");
    }
    if(e && !maze[cur_x][cur_y].right) {
        maze[cur_x][cur_y].right = true; 
        if((cur_x < (MAZE_SIZE - 1)) && !maze[cur_x+1][cur_y].left) {
            maze[cur_x+1][cur_y].left = true; 
        }
        mapUpdated = true; 
        BT.println("  -> Discovered NEW Wall: EAST");
    }
    if(w && !maze[cur_x][cur_y].left) {
        maze[cur_x][cur_y].left = true; 
        if((cur_x > 0) && !maze[cur_x-1][cur_y].right) {
            maze[cur_x-1][cur_y].right = true; 
        }
        mapUpdated = true; 
        BT.println("  -> Discovered NEW Wall: WEST");
    }

    // 4. Log if the map state actually changed
    if(mapUpdated) {
        BT.println("Map changed! Requesting new floodfill...");
    } else {
        BT.println("No new walls found.");
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

    if(!maze[cur_x][cur_y].up && cur_y < (MAZE_SIZE - 1)) {
        int dist = maze[cur_x][cur_y+1].distance;
        if(dist < min_dist) {   
            min_dist = dist; 
            v.clear();
            v.push_back(North);
            BT.println("Heading North");
        }
    }
    if(!maze[cur_x][cur_y].down && cur_y > 0) {
        int dist = maze[cur_x][cur_y-1].distance;
        if(dist < min_dist) {   
            min_dist = dist; 
            v.clear();
            v.push_back(South);
            BT.println("Heading South");
        }
    }
    if(!maze[cur_x][cur_y].right && cur_x < (MAZE_SIZE - 1)) {
        int dist = maze[cur_x+1][cur_y].distance;
        if(dist < min_dist) {   
            min_dist = dist; 
            v.clear();
            v.push_back(East);
            BT.println("Heading East");
        }
    }
    if(!maze[cur_x][cur_y].left && cur_x > 0) {
        int dist = maze[cur_x-1][cur_y].distance;
        if(dist < min_dist) {   
            min_dist = dist; 
            v.clear();
            v.push_back(West);
            BT.println("Heading West");
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


    turnTo(target_heading);
    

    centerUntilDistance(distance);
    times_moved++; 
    
    if (current_heading == North) cur_y++;
    else if (current_heading == East) cur_x++;
    else if (current_heading == South) cur_y--;
    else if (current_heading == West) cur_x--;
}

void closeAll(Cell maze[MAZE_SIZE][MAZE_SIZE]) {
    for(int x = 0; x < MAZE_SIZE; x++) {
        for(int y = 0; y < MAZE_SIZE; y++) { 
            if(!maze[x][y].visited) {
                maze[x][y].up = true; 
                maze[x][y].down = true; 
                maze[x][y].left = true; 
                maze[x][y].right = true;
                if(y < (MAZE_SIZE - 1)) maze[x][y+1].down = true;
                if(y > 0)  maze[x][y-1].up = true;
                if(x < (MAZE_SIZE - 1)) maze[x+1][y].left = true;
                if(x > 0)  maze[x-1][y].right = true;
            }
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

        if(!maze[cx][cy].up && cy < (MAZE_SIZE - 1)) {
            int g_value; 
            if(sim_heading == North) g_value = maze[cx][cy].g + straight_cost; 
            else g_value = maze[cx][cy].g + turn_cost; 
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
            if(sim_heading == South) g_value = maze[cx][cy].g + straight_cost; 
            else g_value = maze[cx][cy].g + turn_cost; 
            if(g_value < maze[cx][cy-1].g) {
                maze[cx][cy-1].g = g_value;
                maze[cx][cy-1].f = g_value + maze[cx][cy-1].h;
                maze[cx][cy-1].parent_x = cx;
                maze[cx][cy-1].parent_y = cy;
                pq.push({cx, cy-1, maze[cx][cy-1].f, maze[cx][cy-1].h, South});
            }
        }
        if(!maze[cx][cy].right && cx < (MAZE_SIZE - 1)) {
            int g_value; 
            if(sim_heading == East) g_value = maze[cx][cy].g + straight_cost; 
            else g_value = maze[cx][cy].g + turn_cost; 
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
            if(sim_heading == West) g_value = maze[cx][cy].g + straight_cost;
            else g_value = maze[cx][cy].g + turn_cost; 
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

void moveNormal(std::vector<int> path) {
    for(size_t i = 0; i < path.size(); i++){
        turnTo(path[i]); 
        current_heading = (heading)path[i]; 
        centerUntilDistance(distance); 
    }
}

void moveFast(std::vector<int> path){
    uint8_t i = 0; 
    while(i < path.size()){
    int current_path_heading = path[i]; 
    int number_of_same_headings = 1; 

    while(i+1 < path.size() && path[i+1] == current_path_heading){
        number_of_same_headings++; 
        i++;
    }
    if(current_path_heading != current_heading){
    turnTo(current_path_heading);}
    float total_distance = distance*number_of_same_headings; 
    centerUntilDistance(total_distance); 

    i++; 

    }
}

int executeFloodFill() {  
    BT.println("Initializing flood fill to goal"); 
    floodfill(); 


    while(maze[cur_x][cur_y].distance != 0 ) {
        if(updateMap()) {
            floodfill(); 
        }
        mousemove();
        BT.println("X: " +  String(cur_x) + " | Y: " +  String(cur_y));
    }

    BT.println("Goal reached!");
    BT.println("Cells visited: " + String(cells_visited));
    BT.println("Search moves: " + String(times_moved)); 

    if(!maze[cur_x][cur_y].visited) {
        maze[cur_x][cur_y].visited = true;
        cells_visited++; 
    }
    motorStop(); 
    copyMap(); 
    closeAll(copy_maze);
    std::vector<int> path = getAStarPath(copy_maze);

    int currentButtonPress = runmodeButtonPressCount;


    turnTo(North);


    BT.println("Press Run mode again to start the bot"); 

    delay(2000); 

    currentButtonPress = runmodeButtonPressCount; 
    while(currentButtonPress == runmodeButtonPressCount){
        delay(5);
    }
    delay(500);
    BT.println("Starting final optimized speedrun run...");
    BASE_SPEED = BASE_SPEED+50;
    moveFast(path);
    BT.println("Final speedrun moves taken: " + String(times_moved));
    calculateCost(path); 

    return 0; 
}