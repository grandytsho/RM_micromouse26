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
