#include <iostream>
#include <string>
#include <stack>
#include <cmath>
#include "chassis.h"
#include "sensor.h"

using namespace std;

// ─────────────────────────────────────────
//  Constants
// ─────────────────────────────────────────
#define MAZE_SIZE 16

// Directions: 0=North, 1=East, 2=South, 3=West
int dr[] = { 1,  0, -1,  0};
int dc[] = { 0,  1,  0, -1};

// ─────────────────────────────────────────
//  Maze Grid & Mouse State
// ─────────────────────────────────────────
int maze[MAZE_SIZE][MAZE_SIZE];

int mouseR   = 0;
int mouseC   = 0;
int facing   = 0;   // 0=North, 1=East, 2=South, 3=West
int cameFrom = -1;

stack<int> pathStack;

// ─────────────────────────────────────────
//  Global Chassis Instance
//  Configure these with your actual hardware values
// ─────────────────────────────────────────
Chassis chassis;

// ─────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────
bool isGoal(int r, int c) {
    return (r == 7 || r == 8) && (c == 7 || c == 8);
}

bool inBounds(int r, int c) {
    return r >= 0 && r < MAZE_SIZE && c >= 0 && c < MAZE_SIZE;
}

int opposite(int d) { return (d + 2) % 4; }

// ─────────────────────────────────────────
//  Maze Initialization
// ─────────────────────────────────────────
void initMaze() {
    for (int i = 0; i < MAZE_SIZE; i++)
        for (int j = 0; j < MAZE_SIZE; j++)
            maze[i][j] = -1;
}

// ─────────────────────────────────────────
//  Physical Movement via Chassis
//  Replaces API::moveForward(), API::turnLeft(), API::turnRight()
// ─────────────────────────────────────────
void physicalMoveForward() {
    chassis.moveForwardTile();  // moves exactly one cell
}

void physicalTurnLeft() {
    chassis.turnLeft();         // 90° left turn
    facing = (facing + 3) % 4;
}

void physicalTurnRight() {
    chassis.turnRight();        // 90° right turn
    facing = (facing + 1) % 4;
}

// ─────────────────────────────────────────
//  Facing Utilities
// ─────────────────────────────────────────
void faceDirection(int target) {
    while (facing != target) {
        int diff = (target - facing + 4) % 4;
        if (diff == 1) { physicalTurnRight(); }
        else           { physicalTurnLeft();  }
    }
}

// ─────────────────────────────────────────
//  Wall Sensing via Sensors
//  Replace hasWallFront/Left/Right with your actual sensor calls
//  e.g. sensor readings from sensor.cpp (IR or ToF distance sensors)
// ─────────────────────────────────────────
bool hasWallFront() {
    // TODO: Replace with actual sensor read from sensor.cpp
    // Example: return sensor.getDistanceFront() < WALL_THRESHOLD;
    return false;
}

bool hasWallLeft() {
    // TODO: Replace with actual sensor read from sensor.cpp
    // Example: return sensor.getDistanceLeft() < WALL_THRESHOLD;
    return false;
}

bool hasWallRight() {
    // TODO: Replace with actual sensor read from sensor.cpp
    // Example: return sensor.getDistanceRight() < WALL_THRESHOLD;
    return false;
}

bool wallInDirection(int d) {
    int diff = (d - facing + 4) % 4;
    switch (diff) {
        case 0: return hasWallFront();
        case 1: return hasWallRight();
        case 3: return hasWallLeft();
        case 2: {
            // Check behind: turn around, sense, turn back
            physicalTurnRight(); physicalTurnRight();
            bool w = hasWallFront();
            physicalTurnRight(); physicalTurnRight();
            return w;
        }
    }
    return true;
}

// ─────────────────────────────────────────
//  Path Scanning
// ─────────────────────────────────────────
int scanPaths(int r, int c, int from) {
    int count = 0;
    for (int d = 0; d < 4; d++) {
        if (d == from) continue;
        if (!wallInDirection(d)) count++;
    }
    return count;
}

// ─────────────────────────────────────────
//  Distance to goal center (7.5, 7.5)
// ─────────────────────────────────────────
float distToGoal(int r, int c) {
    float ddr = r - 7.5f;
    float ddc = c - 7.5f;
    return ddr * ddr + ddc * ddc;
}

// ─────────────────────────────────────────
//  Exploration Algorithm
// ─────────────────────────────────────────
void explore() {
    int r = mouseR, c = mouseC;
    int from = cameFrom;

    while (!isGoal(r, c)) {

        // 1. Scan if unvisited
        if (maze[r][c] == -1) {
            int paths = scanPaths(r, c, from);
            maze[r][c] = paths;
        }

        // 2. Collect unvisited neighbors, pick closest to goal
        int chosen = -1;
        float bestDist = 1e9;
        int checkOrder[] = {
            (facing + 3) % 4,  // Left
            facing,            // Forward
            (facing + 1) % 4   // Right
        };

        for (int i = 0; i < 3; i++) {
            int d = checkOrder[i];
            if (d == from) continue;
            int nr = r + dr[d];
            int nc = c + dc[d];
            if (inBounds(nr, nc) && !wallInDirection(d) && maze[nr][nc] == -1) {
                float dist = distToGoal(nr, nc);
                if (dist < bestDist) {
                    bestDist = dist;
                    chosen = d;
                }
            }
        }

        // 3. Found unvisited neighbor — move there
        if (chosen != -1) {
            pathStack.push(opposite(chosen));
            faceDirection(chosen);
            physicalMoveForward();      // <-- Physical movement
            r     += dr[chosen];
            c     += dc[chosen];
            from   = opposite(chosen);
            facing = chosen;
            continue;
        }

        // 4. Dead end — backtrack using stack
        if (pathStack.empty()) {
            cerr << "No path to goal found." << endl;
            return;
        }

        int backDir = pathStack.top();
        pathStack.pop();

        faceDirection(backDir);
        physicalMoveForward();          // <-- Physical backtrack movement

        facing = backDir;
        from   = opposite(backDir);
        r += dr[backDir];
        c += dc[backDir];

        if (maze[r][c] > 0) {
            maze[r][c]--;
        }
    }

    cerr << "Goal reached at (" << r << "," << c << ")!" << endl;
}

// ─────────────────────────────────────────
//  Entry Point
// ─────────────────────────────────────────
int main() {
    // ── Configure chassis with your hardware values ──
    // chassis.setChassisAttr(wheelDiameter, encRatio, wheelTrack);
    // chassis.setMotors(&backRightMotor, &backLeftMotor, &frontRightMotor, &frontLeftMotor);
    // chassis.setPID(&distPID, &anglePID, &turnPID);
    // chassis.setError(distanceError, angleError);

    cerr << "Micromouse starting..." << endl;
    initMaze();
    explore();
    return 0;
}