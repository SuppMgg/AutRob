#include <DxlMaster.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <NecDecoder.h>

NecDecoder ir;

#include <MD_CirQueue.h>

struct Cell {
  int8_t x;
  int8_t y;
};

MD_CirQueue Q(10, sizeof(uint8_t[2]));

#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>

MPU6050 mpu;

uint8_t fifoBuffer[64];

float prevYaw = 0.0, current, diff, startYaw, currentYaw;
const float TARGET = 90.0;
const int BASE_SPEED = 300;
const float KP = 8.0;

#define SHUTCentre_PIN 6
#define SHUTLeft_PIN 4
#define SHUTRight_PIN 7

VL53L0X sensor_c, sensor_l, sensor_r;
uint16_t Sensor_Vl[3];
uint8_t h[5][5];
uint8_t g[5][5];
uint8_t queue[5][5];
uint8_t walls[5][5];
uint8_t visited[5][5];
uint8_t closed[5][5];
const int8_t dx[4] = {0, -1, 0, 1};
const int8_t dy[4] = {1, 0, -1, 0};
uint8_t posX = 0, posY = 0, dir = 0, prevX, prevY, rast = 140, vx, vy, goalX, goalY, vdir = dir;
bool wallF, wallL, wallR, un;

enum ID {
  id1 = 1,
  id2,
  id3,
  id4
};

DynamixelMotor motor1(ID::id1);
DynamixelMotor motor2(ID::id2);
DynamixelMotor motor3(ID::id3);
DynamixelMotor motor4(ID::id4);

void readSensors();
void searchPath();
int newPlace();
bool wallVirt(int);
bool wallIn(int);
void moveTo(int);
bool Fork();
void forward();
void right();
void left();
void around();
void vl53(VL53L0X &, int, int);
void pushCell(int8_t, int8_t);
Cell popCell();
bool hasCells();
void wave(uint8_t, uint8_t);
float getYaw();
float angleDiff(float, float);

bool n = 0;

void irIsr() {
  ir.tick();
}

void setup() {
  pinMode(SHUTCentre_PIN, OUTPUT);
  pinMode(SHUTLeft_PIN, OUTPUT);
  pinMode(SHUTRight_PIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  DxlMaster.begin(1000000);
  delay(1000);
  Q.begin();
  
  digitalWrite(SHUTCentre_PIN, LOW);
  digitalWrite(SHUTLeft_PIN, LOW);
  digitalWrite(SHUTRight_PIN, LOW);
  delay(10);
  
  vl53(sensor_c, SHUTCentre_PIN, 0x30);
  vl53(sensor_l, SHUTLeft_PIN, 0x31);
  vl53(sensor_r, SHUTRight_PIN, 0x32);
  
  motor1.enableTorque();
  motor1.wheelMode();
  motor2.enableTorque();
  motor2.wheelMode();
  
  mpu.initialize();
  delay(100);
  
  if (!mpu.testConnection()) {
    while (true) delay(500);
  }
  
  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    
    mpu.CalibrateAccel(100);
    mpu.CalibrateGyro(100);
    
    delay(2000);
    
    mpu.resetFIFO();
    delay(100);
    
    prevYaw = getYaw();
  } else while (true) delay(500);
  
  pinMode(3, OUTPUT);
  attachInterrupt(0, irIsr, FALLING);
  
  memset(walls, 0, sizeof(walls));
  
  digitalWrite(3, 1);
  delay(500);
  digitalWrite(3, 0);
}

void loop() {
  readSensors(); //чтение данных с дальномеров
  memset(queue, 0, sizeof(queue));
  memset(h, 99, sizeof(h));
  memset(g, 99, sizeof(g));
  g[posX][posY] = 0;
  uint8_t ch = 0;
  while (ch < 2){ //получение данных с ИК-пульта дважды: координата Х и Y
    while(!ir.available()){}
    switch(ir.readCommand()){
      case 152: n = 1; (ch == 0)? goalX = 0 : goalY = 0; ch++; break;
      case 162: n = 1; (ch == 0)? goalX = 1 : goalY = 1; ch++; break;
      case 98: n = 1; (ch == 0)? goalX = 2 : goalY = 2; ch++; break;
      case 226: n = 1; (ch == 0)? goalX = 3: goalY = 3; ch++; break;
      case 34: n = 1; (ch == 0)? goalX = 4 : goalY = 4; ch++; break;
      default: n = 0; (ch == 0)? goalX = 99 : goalY = 99; break;
    }
    digitalWrite(3, n); 
    delay(300);
    digitalWrite(3, 0); //мигание светодиодом - подтверждение получения сигнала
  }
  un = 0;
  wave(goalX, goalY);//заполнение массива h, хранящего эвристику для всей карты 
  searchPath();//построение пути виртуально по алгоритму

  if (walls[goalX][goalY] == 15) {
    un != un;
    Serial.println("Вершина недостижима");
  }  
  while(posX != goalX || posY != goalY || un){ //прохождение маршрута 
    readSensors(); //чтение данных с дальномеров
    moveTo(newPlace()); //переход к вершине в направлении, возвращаемом newPlace
  }
  motor1.speed(0); //остановка движения (приход к цели), ожидание новой цели
  motor2.speed(0);
}

void readSensors() {
  Sensor_Vl[0] = sensor_c.readRangeContinuousMillimeters();
  Sensor_Vl[1] = sensor_l.readRangeContinuousMillimeters();
  Sensor_Vl[2] = sensor_r.readRangeContinuousMillimeters();
  wallF = Sensor_Vl[0] < rast + 50;
  wallL = Sensor_Vl[1] < rast + 50;
  wallR = Sensor_Vl[2] < rast + 50;
}

void searchPath() {
    uint8_t minDist = 99, Reus = 99;
    prevX = posX;
    prevY = posY;
    int8_t bx;
    int8_t by;
    int8_t bdir;
    int8_t rx;
    int8_t ry;
    vx = posX;
    vy = posY;
    vdir = dir;
    memset(visited, 0, sizeof(visited));
    memset(closed, 0, sizeof(closed));
    visited[vx][vy] = 1;
    while(h[vx][vy] != 0){
      minDist = 99;
      if (!queue[vx][vy]) queue[vx][vy] = (!Fork()) ? 1 : 2;
      for (int8_t i = 0; i < 4; i++) {
          int8_t testDir = (vdir + i) % 4;
          int8_t nx = vx + dx[testDir];
          int8_t ny = vy + dy[testDir];
        if (!wallVirt(testDir, vx, vy)) {
          if (nx >= 0 && nx < 5 && ny >= 0 && ny < 5 && queue[nx][ny] != 99 && !visited[nx][ny]) {
            if (g[nx][ny] == 99) g[nx][ny] = g[vx][vy] + 1;
            if (g[nx][ny] + h[nx][ny] < minDist) {
              minDist = g[nx][ny] + h[nx][ny];
              bx = nx;
              by = ny;
              bdir = testDir;
            }
          }
        }
      }
      visited[bx][by] = 1;
      if (Fork()){
        for (int8_t i = 0; i < 4; i++) {
          int8_t testDir = (vdir + i) % 4;
          int8_t nx = vx + dx[testDir];
          int8_t ny = vy + dy[testDir];
          if (!wallVirt(testDir, vx, vy)) {
            if (nx >= 0 && nx < 5 && ny >= 0 && ny < 5 && queue[nx][ny] != 99 && nx != bx && ny != by && !visited[nx][ny]) {
              if (g[nx][ny] == 99) g[nx][ny] = g[vx][vy] + 1;
              if (g[nx][ny] + h[nx][ny] < Reus) {
                Reus = g[nx][ny] + h[nx][ny];
                rx = nx;
                ry = ny;
              }
            }
          }
        }
      }
      if (walls[vx][vy] == 15){
        for (uint8_t x = 0; x < 5; x++){
          for (uint8_t y = 0; y < 5; y++){
            if (queue[x][y] == 3) queue[x][y] = 99;
            if (queue[x][y] == 4) {
              vx = x;
              vy = y;
              Reus = 99;
              goto home_yolter;
            }
          }
        }
        for (uint8_t x = 0; x < 5; x++){
          for (uint8_t y = 0; y < 5; y++){
            if (queue[x][y] == 3) queue[x][y] = 99;
            if (queue[x][y] == 2) {
              vx = x;
              vy = y;
              queue[x][y] = 0;
              goto home_yolter;
            }
          }
        }
      }
      if (queue[vx][vy] == 2) queue[bx][by] = 3;
      if (Reus < minDist) {
        queue[vx][vy] = 4;
        vx = rx;
        vy = ry;
        Reus = 99;
        goto home_yolter;
      }
      vx = bx;
      vy = by;
      vdir = bdir;
      home_yolter:
      ;
    }
    for (uint8_t x = 0; x < 5; x++){
      for (uint8_t y = 0; y < 5; y++){
        if (g[x][y] != 99 && visited[x][y]) queue[x][y] = 1;
        if (queue[x][y] == 2 || queue[x][y] == 3) queue[x][y] = 1;
      }
    }
}

int newPlace() {
  uint8_t bestDir = 99;
  int8_t nx, ny;
  vx = posX; vy = posY;
  vdir = dir;
  closed[vx][vy] = 1;
  for (int8_t i = 0; i < 4; i++) {
    int8_t testDir = (dir + i) % 4;
    nx = posX + dx[testDir];
    ny = posY + dy[testDir];
    if (!wallIn(testDir)) {
        if (wallVirt(testDir, vx, vy)) {
          walls[posX][posY] &= ~(1 << testDir);
          if (nx >= 0 && nx < 5 && ny >= 0 && ny < 5) walls[nx][ny] &= ~(1 << ((testDir + 2) % 4));
          memset(g, 99, sizeof(g));
          g[posX][posY] = 0;
          wave(goalX, goalY);
          memset(queue, 0, sizeof(queue));
          searchPath();
          return newPlace(); 
        }
        if (nx >= 0 && nx < 5 && ny >= 0 && ny < 5 && queue[nx][ny] == 1 && !closed[nx][ny]) {
          bestDir = testDir;
          break;
        }
    }
    else if (wallIn(testDir) && !wallVirt(testDir, vx, vy)) {
      walls[posX][posY] |= (1 << testDir);
      if (nx >= 0 && nx < 5 && ny >= 0 && ny < 5) walls[nx][ny] |= (1 << ((testDir + 2) % 4));
      if (queue[nx][ny] == 1){
        memset(g, 99, sizeof(g));
        g[posX][posY] = 0;
        wave(goalX, goalY);
        memset(queue, 0, sizeof(queue));
        if (wallF && wallL && wallR){
          around();
          dir = (dir + 2) % 4;
        }
        searchPath();
        return newPlace();
      }
    }
  }
  return bestDir;
}

bool wallVirt(int testDir, uint8_t vx, uint8_t vy) {
  return (walls[vx][vy] >> testDir) & 1;
}

bool wallIn(int testDir) {
  int relDir = (testDir - dir + 4) % 4;
  switch (relDir) {
    case 0: return wallF;
    case 1: return wallR;
    case 2: return false;
    case 3: return wallL;
  }
  return false;
}

void moveTo(int targetDir) {
  if (targetDir == 99) goto qwerty;
  int turn = (targetDir - dir + 4) % 4;
  switch (turn) {
    case 0: forward(); break;
    case 1: left(); forward(); break;
    case 2: around(); forward(); break;
    case 3: right(); forward(); break;
  }
  dir = targetDir;
  posX += dx[dir];
  posY += dy[dir];
  qwerty:
  ;
}

bool Fork() {
  int freeDirs = 0;
  for (int i = 0; i < 4; i++) {
    int testDir = (vdir + i) % 4;
    if (!wallVirt(testDir, vx, vy)) freeDirs++;
  }
  return freeDirs >= 2;
}

void forward() {
  unsigned long tStart = millis();
  motor1.speed(BASE_SPEED);
  motor2.speed(-BASE_SPEED);
  while (millis() - tStart < 1800) {
    readSensors();
    if (Sensor_Vl[0] < rast - 20) break;
    motor1.speed(BASE_SPEED);
    motor2.speed(-BASE_SPEED);
    delay(20);
  }
  motor1.speed(0);
  motor2.speed(0);
  delay(500);
}

void right() {
  startYaw = getYaw();
  motor1.speed(280);
  motor2.speed(280);
  do {
    current = getYaw();
    diff = angleDiff(current, startYaw);
    delay(12);
  } while (diff < TARGET - 10);
  motor1.speed(0);
  motor2.speed(0);
  delay(100);
  prevYaw = getYaw();
}

void left() {
  startYaw = getYaw();
  motor1.speed(-280);
  motor2.speed(-280);
  do {
      current = getYaw();
      diff = angleDiff(startYaw, current);
      delay(12);
  } while (diff < TARGET - 3);
  motor1.speed(0);
  motor2.speed(0);
  delay(100);
  prevYaw = getYaw();
}

void around() {
  left();
  left();
}

void vl53(VL53L0X &sensor_n, int sensor, int val) {
  digitalWrite(sensor, HIGH);
  delay(10);
  sensor_n.init();
  sensor_n.setAddress(val);
  sensor_n.setTimeout(500);
  sensor_n.startContinuous(10);
}

void pushCell(int8_t x, int8_t y) {
  Cell cell = {x, y};
  Q.push((uint8_t*)&cell);
}

Cell popCell() {
  Cell cell;
  Q.pop((uint8_t*)&cell);
  return cell;
}

bool hasCells() {
  return !Q.isEmpty();
}

void wave(uint8_t startX, uint8_t startY){
  while (!Q.isEmpty()) {
    Cell temp;
    Q.pop((uint8_t*)&temp);
  }
  h[startX][startY] = 0;
  pushCell(startX, startY);
  while (hasCells()) {
    Cell current = popCell();
    int x = current.x;
    int y = current.y;
    for (int i = 0; i < 4; i++) {
      int nx = x + dx[i];
      int ny = y + dy[i];
      if (nx >= 0 && nx < 5 && ny >= 0 && ny < 5 && h[nx][ny] == 99 && !wallVirt(i, x, y)) {
        h[nx][ny] = h[x][y] + 1;
        pushCell(nx, ny);
      }
    }
  }
}

float getYaw() {
  static uint32_t lastGoodPacket = 0;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    lastGoodPacket = millis();
    float yawDeg = degrees(ypr[0]);
    prevYaw = yawDeg;
    return yawDeg;
  }
  return prevYaw;
}

float angleDiff(float newA, float oldA) {
  float diff = newA - oldA;
  diff = fmod(diff + 540.0, 360.0) - 180.0;
  return diff;
}
