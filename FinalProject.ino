#include <Pololu3piPlus32U4.h>
#include <Servo.h>
using namespace Pololu3piPlus32U4;
#include "my_robot.h"
#include "sonar.h"
#include "printOLED.h"

#define w 9.6   // only width of the robot necessary for precise turns

#define NUM_ROWS 4  // maze specifications
#define NUM_COLS 9
#define NUM_SQUARES (NUM_ROWS * NUM_COLS)
#define NUM_DIRS 4
#define NUM_TRASHES 3

#define LEFT_ADJUST 1       // amounts to adjust robot movement to be more accurate
#define RIGHT_ADJUST 1.02
#define FORWARD_ADJUST 1.06

enum Direction : int8_t {NORTH, EAST, SOUTH, WEST};
enum Turn : int8_t {FORWARD, RIGHT, BACKWARD, LEFT};

/*
* Given a direction and a turn, calculate the new direction. (i.e. NORTH + LEFT -> WEST)
*/
Direction turn(Direction curr_dir, Turn change_dir) {
  return (curr_dir + change_dir) % NUM_DIRS;
}

Sonar sonar(4);   // initialize objects to interact with robot's hardware
Servo servo;
PrintOLED printOLED;
LineSensors lineSensors;
MyRobot botato(w);
Buzzer buzz;

int8_t curr_square = 0, next_square;  // initialize robot's starting position + predicted next position
Direction curr_dir = Direction::NORTH, next_dir;
Turn curr_dir_change;   // track the next turn to be made
Direction facing_dir;   // track which direction the sonar is facing

float sonar_readings[4] = {100, 100, 100, 100}; // order: forward, right, backward, left for sonar readings
unsigned int lineSensorValues[5];   // initialize array for line sensor readings

int8_t adj[NUM_SQUARES][NUM_DIRS];  // adjacency matrix to represent the maze as a graph
/* Below is what adj should look like once the entire maze is scanned
{
    {-1, 1, 9, -1},     // 0
    {-1, -1, 10, 0},
    {-1, -1, -1, -1},
    {-1, -1, -1, -1},
    {-1, 5, 13, -1},
    {-1, 6, -1, 4},     // 5
    {-1, 7, -1, 5},
    {-1, 8, 16, 6},
    {-1, -1, 17, 7},
    {0, -1, 18, -1},
    {1, 11, 19, -1},    // 10
    {-1, 12, 20, 10},
    {-1, 13, 21, 11},
    {4, -1, 22, 12},
    {-1, 15, 23, -1},
    {-1, -1, 24, 14},   // 15
    {7, -1, 25, -1},
    {8, -1, 26, -1},
    {9, -1, 27, -1},
    {10, 20, -1, -1},
    {11, 21, -1, 19},   // 20
    {12, 22, -1, 20},
    {13, -1, -1, 21},
    {14, 24, 32, -1},
    {15, -1, 33, 23},
    {16, -1, -1, -1},   // 25
    {17, -1, 35, -1},
    {18, 28, -1, -1},
    {-1, 29, -1, 27},
    {-1, 30, -1, 28},
    {-1, 31, -1, 29},   // 30
    {-1, 32, -1, 30},
    {23, 33, -1, 31},
    {24, 34, -1, 32},
    {-1, 35, -1, 33},
    {26, -1, -1, 34}    // 35
};
*/

bool visited[NUM_SQUARES];  // array to track which squares have been visited
Turn dir_priority[2][NUM_DIRS] = {{RIGHT, FORWARD, LEFT, BACKWARD}, {LEFT, FORWARD, RIGHT, BACKWARD}}; // order to prioritize direction
bool done = false;          // track whether the robot has picked up all trash and returned to start

int8_t trash_count = 0;   // track total trash picked up so far
int8_t trash_locs[NUM_TRASHES] = {-1, -1, -1};  // track locations of the trash to display at the end

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial) continue;
  servo.attach(5);
  servo.write(90);  // face forward
  delay(500);
  initialize_adj();
  initialize_visited();
  lineSensors.calibrate();
}

/*
* Initialize all values in the adjacency list to be -1.
*/
void initialize_adj() {
  for (int8_t sq = 0; sq < NUM_SQUARES; sq++) {
    for (int8_t dir = NORTH; dir < NUM_DIRS; dir++) {
      adj[sq][dir] = -1;
    }
  }
}

/*
* Initialize all values in the visited array to be false.
*/
void initialize_visited() {
  for (int8_t sq = 0; sq < NUM_SQUARES; sq++) {
    visited[sq] = false;
  }
}

void loop() {
  //put your main code here, to run repeatedly:
  if (!done) {
    Serial.println("Traversing maze!");
    traverse_maze();
  }
  // if done, do nothing
}

/*** MAZE TRAVERSAL HELPER FUNCTION ***/

/*
* Scan the maze layout, traverse the maze, and look for trash to clean.
*/
void traverse_maze() {
  // if out of bounds print error message and do nothing else
  if (curr_square < 0 || curr_square >= NUM_SQUARES || curr_dir < 0 || curr_dir >= NUM_DIRS) {
    Serial.println("ERROR! INVALID SQUARE OR DIRECTION!");
    print_curr_position();
    return;
  }

  printOLED.print_float(trash_count); // display trash_count to OLED screen

  // only scan the maze if have not yet visited the current square
  if (!visited[curr_square]) {
    collect_sonar_readings();   // scan and display the current square sonar readings
    print_sonar_readings();

    update_adj();   // update and display the robot's knowledge of the maze layout
    print_adj();
  }

  visited[curr_square] = true;  // mark current square as visited

  // predict and display the next movement
  // NOTE: this is the only hardcoding necessary to ensure the robot visits all squares
  // in 1 lap around the maze. There are alternate solutions that require no hardcoding,
  // but are slower to fully run.
  next_movement(dir_priority[curr_square == 20 || curr_square == 21]);
  print_next_movement();

  // make the predicted movement
  move();
  
  // update the robot's current position
  curr_square = next_square;    // track movement to next square
  curr_dir = next_dir;

  print_curr_position();  // display the robot's new position

  // only check for trash if not visited square yet and not picked up all trash yet
  if (!visited[curr_square] && trash_count < NUM_TRASHES) {  
    if (detect_trash()) {
      collect_trash();
    }
  }

  // if back on charging port and picked up all trash, we are done
  if (curr_square == 0 && trash_count >= NUM_TRASHES) {
    Serial.println("Done traversing the maze!");
    done = true;
    buzz.play(">g16>>c16");   // beep
    print_maze();   // display entire scanned maze to serial monitor
  }
}

/*** TRASH HELPER FUNCTIONS ***/

bool detect_trash() {
  Serial.println("Detecting trash!");
  lineSensors.read(lineSensorValues);         // save line sensor values

  // Check if the robot is on a black square
  for (int i = 0; i < 5; i++) {               // for each sensor
      Serial.print("Sensor val = ");
      Serial.println(lineSensorValues[i]);    // print its value
      if (lineSensorValues[i] > 2000) {       // if line detected
          Serial.println("Trash detected!");
          return true;
      }
  }
  return false; 
}

void collect_trash() {
  Serial.println("Collecting trash!");
  
  // spin 360
  botato.turn_left_rads(2*PI * LEFT_ADJUST, base_speed, 0.2);

  // track trash locations
  trash_locs[trash_count] = curr_square;

  // track total # trash that has been cleaned
  trash_count++;
}

/*** SCANNING MAZE HELPER FUNCTIONS ***/

/*
* Collect sonar data in front, to the left, and to the right of the robot.
*/
void collect_sonar_readings() {
  sonar_readings[FORWARD] = sonar.readDist(); // read and store forward distance

  servo.write(0);   // turn servo right
  delay(400);       // delay to finish turning servo before reading sonar
  sonar_readings[RIGHT] = sonar.readDist();   // read and store right distance
  delay(100);       // delay to finish reading sonar before turning servo

  servo.write(180); // turn servo left
  delay(400);
  sonar_readings[LEFT] = sonar.readDist();    // read and store left distance
  delay(100);

  servo.write(90);  // turn servo forward
  delay(100);
}

/*
* Update the adjacency list based on sonar readings from the current square.
*/
void update_adj() {
  for (int8_t dir = FORWARD; dir < NUM_DIRS; dir++) { // for each relative direction (turn: forward, right, backward, left)
    facing_dir = turn(curr_dir, dir);           // track cardinal direction (N/E/S/W) sonar is facing
    if (sonar_readings[dir] >= 20) {  // if no wall, track the adjacent square for this square + direction
      adj[curr_square][facing_dir] = facing_square(facing_dir);
    }
  }
}

/*
* Based on current square and facing direction, return which square the robot is facing.
*/
int8_t facing_square(Direction dir) {
  int8_t result;
  switch (dir) {
    case NORTH: // the northward square is up a row
      result = curr_square - NUM_COLS;
      break;
    case EAST:  // the eastward square is to the right 1 col
      result = curr_square + 1;
      break;
    case SOUTH: // the southward square is down a row
      result = curr_square + NUM_COLS;
      break;
    case WEST:  // the westward square is to the left 1 col
      result = curr_square - 1;
      break;
    default:    // if not a valid direction set to invalid square
      result = -1;
  }
  if (result < 0 || result >= NUM_SQUARES) {
    result = -1;  // if out of bounds set to invalid square
  }
  return result;
}

/*** MOVEMENT HELPER FUNCTIONS ***/

/*
* Determine robot's next direction change, square, and facing direction.
*/
void next_movement(Turn *dir_priority) {
  int8_t next_sq_temp;
  Direction next_dir_temp;
  
  for (int8_t dir_idx = 0; dir_idx < NUM_DIRS; dir_idx++) { // for each direction in priority order
    next_dir_temp = turn(curr_dir, dir_priority[dir_idx]);
    next_sq_temp = adj[curr_square][next_dir_temp];
    if (next_sq_temp != -1) {                   // if valid, return result
      curr_dir_change = dir_priority[dir_idx];  // track the direction we have to change
      next_square = next_sq_temp;               // track the next position
      next_dir = next_dir_temp;
      Serial.print("Want to turn ");            // display the predicted next position
      print_turn(curr_dir_change, false);
      Serial.print(" to face ");
      print_dir(next_dir, true);
      return;
    }
  }
  // if no possible movements, print error message and set to invalid position
  Serial.println("next_movement(): No possible movements!");
  next_square = -1;
  next_dir = -1;
}

/*
* Move the robot based on its predicted next position.
*/
void move() {
  // after each movement wait 0.2 s
  switch (curr_dir_change) {
    case FORWARD: 
      botato.forward(0.20 * FORWARD_ADJUST, base_speed, 0.2); 
      break;
    case RIGHT: 
      botato.turn_right_rads(PI/2 * RIGHT_ADJUST, base_speed, 0.2);
      botato.forward(0.20 * FORWARD_ADJUST, base_speed, 0.2);
      break;
    case BACKWARD:
      botato.turn_left_rads(PI * LEFT_ADJUST, base_speed, 0.2);
      botato.forward(0.20 * FORWARD_ADJUST, base_speed, 0.2);
      break;
    case LEFT:
      botato.turn_left_rads(PI/2 * LEFT_ADJUST, base_speed, 0.2);
      botato.forward(0.20 * FORWARD_ADJUST, base_speed, 0.2);
      break;
    default:
      // if invalid turn, print error message and don't move
      Serial.println("move(): INVALID MOVEMENT DIRECTION");
  }
}

/*** DEBUGGING HELPER FUNCTIONS ***/

/*
* Display the sonar readings from each direction to the serial monitor.
*/
void print_sonar_readings() {
  Serial.print("Forward = ");
  Serial.print(sonar_readings[FORWARD]);
  Serial.print(", Left = ");
  Serial.print(sonar_readings[LEFT]);
  Serial.print(", Right = ");
  Serial.print(sonar_readings[RIGHT]);
  Serial.println("\n");
}

/*
* Display the adjacency list to the serial monitor.
*/
void print_adj() {
  for (int8_t sq = 0; sq < NUM_SQUARES; sq++) {
    Serial.print(sq);
    Serial.print(": ");
    for (int8_t dir = 0; dir < NUM_DIRS; dir++) {
      Serial.print(adj[sq][dir]);
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println("-----------------------\n");
}

/*
* Display the robot's next movement to the serial monitor.
*/
void print_next_movement() {
  Serial.print("Next square = ");
  if (next_square < 0 || next_square >= NUM_SQUARES)
    Serial.print("INVALID SQUARE");
  else
    Serial.print(next_square);
  
  Serial.print(", Next direction = ");
  print_dir(next_dir, true);
}

/*
* Display the robot's current position to the serial monitor.
*/
void print_curr_position() {
  Serial.print("Current square = ");
  Serial.print(curr_square);
  Serial.print(", Current direction = ");
  print_dir(curr_dir, true);
}

/*
* Print given direction to the serial monitor with optional newline.
*/
void print_dir(Direction dir, bool newLine) {
  switch (dir) {
    case NORTH: Serial.print("NORTH"); break;
    case EAST: Serial.print("EAST"); break;
    case SOUTH: Serial.print("SOUTH"); break;
    case WEST: Serial.print("WEST"); break;
    default: Serial.print("print_dir(): INVALID DIRECTION");
  }
  if (newLine) {
    Serial.println();
  }
}

/*
* Print given turn to the serial monitor with optional newline.
*/
void print_turn(Turn turn, bool newLine) {
  switch (turn) {
    case FORWARD: Serial.print("FORWARD"); break;
    case RIGHT: Serial.print("RIGHT"); break;
    case BACKWARD: Serial.print("BACKWARD"); break;
    case LEFT: Serial.print("LEFT"); break;
    default: Serial.print("print_turn(): INVALID TURN");
  }
  if (newLine) {
    Serial.println();
  }
}

/*
* Display the entire scanned maze to the serial monitor.
*/
void print_maze() {
  char output[9][19]; // array to store maze information

  // initialize to empty maze
  for (int8_t row = 0; row < 9; row++) {
    for (int8_t col = 0; col < 19; col++) {
      output[row][col] = ' ';
    }
  }

  // populate the maze with walls and trash
  int8_t sq;
  for (int8_t row = 1; row < 9; row += 2) {
    for (int8_t col = 1; col < 19; col += 2) {
      sq = 9 * (row / 2) + (col / 2); // convert row and col to square id

      // place an x for each trash
      for (int8_t trash_idx = 0; trash_idx < NUM_TRASHES; trash_idx++) {
        if (trash_locs[trash_idx] == sq) {
          output[row][col] = 'x';
        }
      }

      // place a line for each wall (- for horizontal, | for vertical)
      if (adj[sq][NORTH] == -1) {
        output[row-1][col] = '-';
      }
      if (adj[sq][EAST] == -1) {
        output[row][col+1] = '|';
      }
      if (adj[sq][SOUTH] == -1) {
        output[row+1][col] = '-';
      }
      if (adj[sq][WEST] == -1) {
        output[row][col-1] = '|';
      }
    }
  }

  // print populated maze to serial monitor
  for (int8_t row = 0; row < 9; row++) {
    for (int8_t col = 0; col < 19; col++) {
      if (output[row][col] == '-') {
        Serial.print("---");
      } else {
        Serial.print(" ");
        Serial.print(output[row][col]);
        Serial.print(" ");
      }
    }
    Serial.println();
  }
}

// That's all, folks!