#include <ChainableLED.h>
#include <Servo.h>

#include "Platform.h"
#include "Ultrasonic.h"

Platform platform(11, 3, 2);

#define SERIAL_WAIT 0
#define TURN90 63
#define LEFT_DIFF -5
#define TURN_SPEED 40
#define MOVE_DIST 68
#define SCOOT_DIST 15
#define MOVE_SPEED 20
#define DELAY 250
#define SERVO_DELAY 1000
#define DIST_THRESHOLD 1200
#define GRID_SIZE 5
#define PHOTO_THRESHOLD 200
#define PHOTO_TURN 5
#define PHOTO_SLOW 15
#define PHOTO_FAST 30
#define PHOTO_TIMEOUT 20

ChainableLED leds(10, 11, 1);
const int start_btn = 13;
const int photo = A0;
Ultrasonic ranger(8);
Servo pan;
int x = 0, y = 0;
enum Heading {
  NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3, NULL_DIR = 5
} heading = NORTH;

bool grid[GRID_SIZE][GRID_SIZE];
bool known[GRID_SIZE][GRID_SIZE];
typedef Heading Path[GRID_SIZE*GRID_SIZE];
Path path;
size_t path_len;

Heading to_left(Heading h) {
  return (Heading) ((h + 3) % 4);
}
Heading to_right(Heading h) {
  return (Heading) ((h + 1) % 4);
}

struct Point {int x, y;};

struct Point to_delta(Heading h) {
  if (h == NULL_DIR) return Point {0, 0};
  int dist = (h & 2) ? -1 : 1;
  return (h & 1) ? Point {dist, 0} : Point {0, dist};
}

bool dir_in_grid(Heading h) {
  Point d = to_delta(h);
  int new_x = x + d.x;
  int new_y = y + d.y;
  return new_x >= 0 && new_x < GRID_SIZE && new_y >= 0 && new_y < GRID_SIZE;
}

void mark_grid(Heading h, bool state) {
  Point d = to_delta(h);
  int new_x = x + d.x;
  int new_y = y + d.y;
  if (!(new_x >= 0 && new_x < GRID_SIZE && new_y >= 0 && new_y < GRID_SIZE)) return;
  known[new_x][new_y] = true;
  grid[new_x][new_y] = state;
}

void waitForSerial() {
  #if SERIAL_WAIT
    Serial.println("waiting for serial");
    while (!Serial.available()) delay(50);
    while (Serial.available()) Serial.read();
  #else
    while (digitalRead(start_btn)) delay(50);
  #endif
}

void updateStatus() {
  Serial.println("checking status");
  if (platform.getStatus()) {
    leds.setColorRGB(0, 255, 0, 0);
  } else {
    leds.setColorRGB(0, 0, 255, 0);
  }
}

void logPos() {
  Serial.print(platform.x, DEC);
  Serial.write(", ");
  Serial.print(platform.y, DEC);
  Serial.write(", ");
  Serial.println(platform.theta, DEC);
}

void logGrid() {
  Serial.write("pos: (");
  Serial.print(x, DEC);
  Serial.write(", ");
  Serial.print(y, DEC);
  Serial.write(", ");
  Serial.print((int) heading, DEC);
  Serial.write(")\n");
  for (int x = 0; x < GRID_SIZE; x++) {
    for (int y = 0; y < GRID_SIZE; y++) {
      if (known[x][y])
        Serial.write(grid[x][y] ? "#" : ".");
      else
        Serial.write("?");
    }
    Serial.write("\n");
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  leds.init();
  pinMode(start_btn, INPUT_PULLUP);
  pinMode(photo, INPUT_PULLUP);
  pan.attach(12);
  pan.write(90);

  Serial.println("Hello!");
  leds.setColorRGB(0, 0, 0, 255);

  for (int x = 0; x < GRID_SIZE; x++) {
    for (int y = 0; y < GRID_SIZE; y++) {
      known[x][y] = false;
      grid[x][y] = false;
    }
  }
  known[0][0] = true;
  
  platform.stop();
  updateStatus();
  waitForSerial();
}

void doMove() {
  platform.driveTicks(MOVE_DIST, MOVE_SPEED, DIR_FWD);
  logPos();
  Point d = to_delta(heading);
  x += d.x;
  y += d.y;
  delay(DELAY);
}

void doTurn(byte dir) {
  platform.turnTicks(dir==DIR_LEFT ? TURN90 + LEFT_DIFF : TURN90, TURN_SPEED, dir);
  logPos();
  if (dir == DIR_LEFT) heading = to_left(heading);
  else heading = to_right(heading);
  delay(DELAY);
}


void scoot(byte dir) {
  platform.driveTicks(SCOOT_DIST, MOVE_SPEED, dir);
  delay(DELAY);
}

size_t writePath(Point p, Heading (*dir_to)[GRID_SIZE][GRID_SIZE]) {
  Serial.write("point:");
  Serial.print(p.x, DEC);
  Serial.write(", ");
  Serial.println(p.y, DEC);
  Serial.flush();
  
  Heading h = (*dir_to)[p.x][p.y];
  if ((p.x == GRID_SIZE-1 && p.y == GRID_SIZE-1) || h == NULL_DIR) return 0;
  
  Point d = to_delta(h);
  size_t len = writePath(Point {p.x - d.x, p.y - d.y}, dir_to);
  path[len++] = h;
  return len;
}

void genPath() {
  Point queue[GRID_SIZE*GRID_SIZE];
  int s=0,e=0;

  Heading dir_to[GRID_SIZE][GRID_SIZE];
  for (int x = 0; x < GRID_SIZE; x++) {
    for (int y = 0; y < GRID_SIZE; y++) {
      dir_to[x][y] = NULL_DIR;
    }
  }
  
  queue[e++] = Point {GRID_SIZE-1, GRID_SIZE-1};

  while (s != e) {
    Point pos = queue[s++];

    if (pos.x == 0 && pos.y == 0) {
      Serial.write("writing path to global");
      path_len = writePath(Point {0, 0}, &dir_to);
      return;
    }

    // n
    if (pos.y < GRID_SIZE-1 && known[pos.x][pos.y+1] && !grid[pos.x][pos.y+1] && dir_to[pos.x][pos.y+1] == NULL_DIR) {
      dir_to[pos.x][pos.y+1] = NORTH;
      queue[e++] = Point {pos.x, pos.y+1};
    }
    // s
    if (pos.y > 0 && known[pos.x][pos.y-1] && !grid[pos.x][pos.y-1] && dir_to[pos.x][pos.y-1] == NULL_DIR) {
      dir_to[pos.x][pos.y-1] = SOUTH;
      queue[e++] = Point {pos.x, pos.y-1};
    }
    // w
    if (pos.x > 0 && known[pos.x-1][pos.y] && !grid[pos.x-1][pos.y] && dir_to[pos.x-1][pos.y] == NULL_DIR) {
      dir_to[pos.x-1][pos.y] = WEST;
      queue[e++] = Point {pos.x-1, pos.y};
    }
    // e
    if (pos.x < GRID_SIZE-1 && known[pos.x+1][pos.y] && !grid[pos.x+1][pos.y] && dir_to[pos.x+1][pos.y] == NULL_DIR) {
      dir_to[pos.x+1][pos.y] = EAST;
      queue[e++] = Point {pos.x+1, pos.y};
    }
  }
}

void wander() {
  logGrid();

  scoot(DIR_REV);
  pan.write(180);
  delay(SERVO_DELAY);
  unsigned long dist = ranger.ping();
  Serial.write("dist: ");
  Serial.println(dist, DEC);
  scoot(DIR_FWD);

  
  // go right
  if (dir_in_grid(to_right(heading)) && dist > DIST_THRESHOLD) {
    mark_grid(to_right(heading), false);
    doTurn(DIR_RIGHT);
    logEncoders();
    pan.write(90);
    delay(SERVO_DELAY);
    if (ranger.ping() > DIST_THRESHOLD) {
      doMove();
    } else {
      mark_grid(heading, true);
    }
  } else {
    mark_grid(to_right(heading), true);
    pan.write(90);
    delay(SERVO_DELAY);
    if (dir_in_grid(heading) && ranger.ping() > DIST_THRESHOLD) {
      mark_grid(heading, false);
      // go fwd
      doMove();
    } else {
      mark_grid(heading, true);
      
      scoot(DIR_REV);
      pan.write(0);
      delay(SERVO_DELAY);
      dist = ranger.ping();
      scoot(DIR_FWD);
      
      if (dir_in_grid(to_left(heading)) && dist > DIST_THRESHOLD) {
        mark_grid(to_left(heading), false);
        // go left
        doTurn(DIR_LEFT);
        pan.write(90);
        delay(SERVO_DELAY);
        if (ranger.ping() > DIST_THRESHOLD) {
          doMove();
        } else {
          mark_grid(heading, true);
        }
      } else {
        mark_grid(to_left(heading), true);
        // turn back
        doTurn(DIR_LEFT);
        doTurn(DIR_LEFT);
      }
    }
  }
}

void moveDir(Heading dir) {
  if (heading != dir) {
    if (to_right(heading) == dir) {
      doTurn(DIR_RIGHT);
    } else if (to_left(heading) == dir) {
      doTurn(DIR_LEFT);
    } else {
      doTurn(DIR_RIGHT);
      doTurn(DIR_RIGHT);
    }
  }
  doMove();
}

void sparkle() {
  for (int i = 0; i < 5; i++) {
    leds.setColorRGB(0, 255, 0, 0);
    delay(250);
    leds.setColorRGB(0, 0, 255, 0);
    delay(250);
    leds.setColorRGB(0, 0, 0, 255);
    delay(250);
  }
}

void findRamp() {
  while (analogRead(photo) > PHOTO_THRESHOLD) {
    platform.turnTicks(PHOTO_TURN, TURN_SPEED, DIR_RIGHT);
    delay(DELAY);
  }
}

void climb() {
  byte dir = DIR_LEFT;
  platform.setSpeed(PHOTO_SLOW, DIR_FWD, PHOTO_FAST, DIR_FWD);
  for (;;) {
    // wait for no light
    while (analogRead(photo) <= PHOTO_THRESHOLD) delay(50);

    // switch dir
    if (dir != DIR_LEFT) {
      dir = DIR_LEFT;
      platform.setSpeed(PHOTO_SLOW, DIR_FWD, PHOTO_FAST, DIR_FWD);
    } else {
      dir = DIR_RIGHT;
      platform.setSpeed(PHOTO_FAST, DIR_FWD, PHOTO_SLOW, DIR_FWD);
    }

    delay(250);

    // wait for light or timeout
    int cnt = 0;
    while (analogRead(photo) > PHOTO_THRESHOLD && cnt++ < PHOTO_TIMEOUT) delay(50);
    if (cnt >= PHOTO_TIMEOUT) {
      platform.stop();
      return;
    }
  }
}

void loop() {
  while (x != GRID_SIZE-1 || y != GRID_SIZE-1)
    wander();

  Serial.write("generating path...\n");
  genPath();
  Serial.write("found path!\n");
  for (int i = 0; i < path_len; i++) {
    Serial.println((int) path[i], DEC);
  }
  Serial.write("path done.\n");

  sparkle();

  if (path_len > 0) {
    int i = 0;
    Heading dir;
    while (dir = path[i++], i <= path_len)
      moveDir(dir);
  } else {
    // backup strats
    doTurn(DIR_RIGHT);
    doTurn(DIR_RIGHT);
    x = y = 0;
    heading = NORTH;
    while (x != GRID_SIZE-1 || y != GRID_SIZE-1)
      wander();
  }

  sparkle();

  findRamp();

  sparkle();

  climb();

  Serial.write("finished.\n");
  while (1) {
    leds.setColorRGB(0, 255, 255, 0);
    delay(250);
    leds.setColorRGB(0, 0, 255, 255);
    delay(250);
    leds.setColorRGB(0, 255, 0, 255);
    delay(250);
  }
}

