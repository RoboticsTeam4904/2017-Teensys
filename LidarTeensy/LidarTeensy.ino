
#include <Encoder.h>
#include <FlexCAN.h>
#include <TeensyCANBase.h>

#include "line_find.h"
#include "boiler_find.h"
#include "doubly_linked_list.h"
#include "point_preprocess.h"
#include "datatypes.h"

#ifdef TIME
#include <stdlib.h>
/**
   Utility to get the amount of free RAM
   From SDFat
*/
extern "C" char* sbrk(int incr);
int get_free_ram() {
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
}
#endif

// Packet loading data
uint8_t current_packet[22];
uint8_t subpacket_idx;
bool start;
uint8_t last_idx;

// Avoiding as-fast-as-possible loops, which increase CAN utilization too much
long LOOP_TIME = 5000; // Microseconds
long last_can_loop;

// Timeout for encoder
long TIMEOUT = 10000;
long last_lidar_data;

// Current data
uint8_t alliance;
#define BLUE_ALLIANCE_MIN_ANGLE 150
#define BLUE_ALLIANCE_MAX_ANGLE 315
#define RED_ALLIANCE_MIN_ANGLE 45
#define RED_ALLIANCE_MAX_ANGLE 210
uint16_t min_angle;
uint16_t max_angle;
uint16_t * distances;
long lidar_speed;
doubly_linked_list_node<lidar_datapoint> * lidar_data_start;
doubly_linked_list_node<line> * line_data_start;
boiler_location boiler;

// CAN IDs
#define MATCH_DATA_ID 0x600
#define CAN_LIDAR_PHASE1_ID 0x620
#define CAN_LIDAR_PHASE2_ID 0x622
#define CAN_LIDAR_ENCODER_ID 0x621

uint8_t calculation_idx;

void try_load_next_lidar_bytes();
void lidar_packet_to_array();
void load_lidar_linked_list();
void serial_lidar_log();

struct encoderData {
  long lastRead;
  long pos;
  long rate;
};

Encoder leftEncoder(16, 17);
encoderData leftData;
Encoder rightEncoder(18, 19);
encoderData rightData;

void resetLeftEncoder(byte * msg) {
  if (msg[0] == 0x72 && msg[1] == 0x65 && msg[2] == 0x73 && msg[3] == 0x65 && msg[4] == 0x74 && msg[5] == 0x65 && msg[6] == 0x6e && msg[7] == 0x63) {
    leftEncoder.write(0);
    leftData.pos = 0;
    leftData.rate = 0;
  }
}

void resetRightEncoder(byte * msg) {
  if (msg[0] == 0x72 && msg[1] == 0x65 && msg[2] == 0x73 && msg[3] == 0x65 && msg[4] == 0x74 && msg[5] == 0x65 && msg[6] == 0x6e && msg[7] == 0x63) {
    rightEncoder.write(0);
    rightData.pos = 0;
    rightData.rate = 0;
  }
}

/**
  Set alliance (red/blue)
*/
void set_alliance(byte * msg) {
  alliance = msg[0];
  if (alliance == BLUE_ALLIANCE) {
    min_angle = BLUE_ALLIANCE_MIN_ANGLE;
    max_angle = BLUE_ALLIANCE_MAX_ANGLE;
  }
  else if (alliance == RED_ALLIANCE) {
    min_angle = RED_ALLIANCE_MIN_ANGLE;
    max_angle = RED_ALLIANCE_MAX_ANGLE;
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(1000); // wait for serial to load
  distances = new uint16_t[360];
  init_trig();
  start = false;
  lidar_data_start = NULL;
  line_data_start = NULL;
  calculation_idx = 0;
  lidar_speed = 0;
  boiler.x = 0;
  boiler.y = 0;
  last_can_loop = 0;
  alliance = 0;
  min_angle = 0;
  max_angle = 0;
  CAN_add_id(MATCH_DATA_ID, &set_alliance);
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);
  digitalWrite(24, HIGH);
  CAN_begin();
  leftData.lastRead = 0;
  leftData.pos = -999;
  leftData.rate = 0;
  CAN_add_id(0x610, &resetLeftEncoder);
  rightData.lastRead = 0;
  rightData.pos = -999;
  rightData.rate = 0;
  CAN_add_id(0x611, &resetRightEncoder);
}

void writeLongs(uint32_t id, long value1, long value2) {
  byte * msg = new byte[8];

  for (int i = 0; i < 4; i++) {
    msg[i] = (value1 >> i * 8) & 0xFF;
  }
  for (int i = 0; i < 4; i++) {
    msg[i + 4] = (value2 >> i * 8) & 0xFF;
  }

  digitalWrite(25, HIGH);
  CAN_write(id, msg);
  digitalWrite(25, LOW);

  delete msg;
}

void loop() {
  long loopStart = micros();
  
  long newPos = leftEncoder.read();
  if (newPos != leftData.pos) {
    leftData.rate = ((double) 1000000.0 * (newPos - leftData.pos)) / ((double) (micros() - leftData.lastRead));
    Serial.println(leftData.rate);
    Serial.println(leftData.pos);
    leftData.pos = newPos;
    leftData.lastRead = micros();
  }
  else {
    if ((micros() - leftData.lastRead) > 1000) {
      leftData.rate = 0;
    }
  }
  writeLongs(0x610, leftData.pos, leftData.rate);
  newPos = rightEncoder.read();
  if (newPos != rightData.pos) {
    rightData.rate = ((double) 1000000.0 * (newPos - rightData.pos)) / ((double) (micros() - rightData.lastRead));
    Serial.println(rightData.pos);
    rightData.pos = newPos;
    rightData.lastRead = micros();
  }
  else {
    if ((micros() - rightData.lastRead) > 1000) {
      rightData.rate = 0;
    }
  }
  writeLongs(0x611, rightData.pos, rightData.rate);

  try_load_next_lidar_bytes();

  if (lidar_speed > 180000) {
    if (calculation_idx == 0) {
      calculation_idx = 1; // Start calculation
    }
  }

  // CAN send
  if (last_can_loop > 1) { // Only send CAN data every 10ms, loop runs at 5ms (for serial read)
    writeLongs(CAN_LIDAR_PHASE1_ID, boiler.center_angle, boiler.center_dist);
    uint16_t front_distance = 0;
    uint8_t count = 0;
    for(uint16_t i = 358; i < 360; i++){
      if(distances[i] != 0){
        front_distance += distances[i];
        count++;
      }
    }
    for(uint16_t i = 0; i < 2; i++){
      if(distances[i] != 0){
        front_distance += distances[i];
        count++;
      }
    }
    front_distance = front_distance / count; // +- 1mm is small enough that we do not care
    writeLongs(CAN_LIDAR_PHASE2_ID, boiler.angle, front_distance);
    writeLongs(CAN_LIDAR_ENCODER_ID, 0, lidar_speed);
    last_can_loop = 0;
  }
  else {
    last_can_loop++;
  }
  // CAN Receive
  CAN_update();

  /*
    LiDAR calculation
    In order to stay under the 5ms limit for serial read, the calculation is split into 10 parts
    (note that each part, except the line finder, is much less than 5ms)
  */
#ifdef TIME
  long timing_start = micros();
  if (calculation_idx != 0) {
    Serial.print("calculation ");
    Serial.println(calculation_idx);
    Serial.flush(); // If code crashes, print will make it
  }
#endif
  if (calculation_idx == 1) {
    interpolate(&distances[0]);
    calculation_idx++;
  }
  else if (calculation_idx == 2) {
    load_lidar_linked_list();
    if (lidar_data_start == NULL) {
      calculation_idx = 0;
    }
    else {
      calculation_idx++;
    }
  }
  else if (calculation_idx == 3) {
    blur_points(lidar_data_start);
    calculation_idx++;
  }
  else if (calculation_idx == 4) {
    blur_points(lidar_data_start);
    calculation_idx++;
  }
  else if (calculation_idx == 5) {
    blur_points(lidar_data_start);
    calculation_idx++;
  }
  else if (calculation_idx == 6) {
    add_cartesians(lidar_data_start);
    calculation_idx++;
  }
  else if (calculation_idx == 7) {
    line_data_start = get_lines(lidar_data_start);
    if (line_data_start == NULL) {
      calculation_idx = 10;
    }
    else {
      calculation_idx++;
    }
  }
  else if (calculation_idx == 8) {
    boiler = get_boiler(line_data_start, alliance);
    calculation_idx++;
  }
  else if (calculation_idx == 9) {
    line_list_cleanup(line_data_start);
    calculation_idx++;
  }
  else if (calculation_idx == 10) {
    lidar_datapoint_list_cleanup(lidar_data_start);
    calculation_idx = 0;
  }
#ifdef TIME
  if (calculation_idx != 0) {
    Serial.print("calculation finished: ");
    Serial.print(micros() - timing_start);
    Serial.print("\t");
    Serial.println(get_free_ram());
  }
#endif

  serial_lidar_log();

  if (micros() - loopStart < LOOP_TIME) {
    delayMicroseconds(LOOP_TIME - (micros() - loopStart));
  }

  if (micros() - last_lidar_data < TIMEOUT) {
    lidar_speed = 0;
  }
}

/**
   Attempt to load the next byte from the LIDAR Serial
   into the packet array
*/
void try_load_next_lidar_bytes() {
  while (Serial1.available()) {
    last_lidar_data = micros();
    uint8_t b = Serial1.read();
    if (b == 0xFA && !start) {
      subpacket_idx = 0;
      memset(current_packet, 0, 22);
      current_packet[0] = 0xFA;
      start = true;
    }
    else if (start) {
      subpacket_idx++;
      current_packet[subpacket_idx] = b;
      if (subpacket_idx == 21) {
        start = false;
        lidar_packet_to_array();
      }
    }
  }
}

/**
   Update the distance array with the latest packet
*/
void lidar_packet_to_array() {
  uint8_t index = current_packet[1] - 0xA0;
  if (index != last_idx) {
    bool error = false;
    lidar_speed = (long) (((double) ((current_packet[3] << 8) | current_packet[2])) * 15.625);
    for (uint8_t i = 0; i < 4; i++) {
      uint8_t data_start = i * 4 + 4;
      uint16_t angle = index * 4 + i;
      if (angle > 359) {
        return; // If the angle is out of range, the packet is corrupted. Moving on.
      }
      error = (current_packet[data_start + 1] & 0x80) > 0;
      if (!error) {
        uint16_t distance = 0;
        distance = ((current_packet[data_start]) | (current_packet[data_start + 1] & 0x0F) << 8);
        distances[angle] = distance;
      }
      else {
        distances[angle] = 0;
      }
    }
    last_idx = index;
  }
}

/**
   Transfer the distance array into
*/
void load_lidar_linked_list() {
  doubly_linked_list_node<lidar_datapoint> * previous_node = NULL;
  for (int i = 0; i < 360; i++) {
    if (i < min_angle || i > max_angle) {
      if (distances[i] != 0) {
        if (previous_node == NULL) {
          previous_node = new doubly_linked_list_node<lidar_datapoint>;
          previous_node->data = new lidar_datapoint;
          previous_node->data->theta = i;
          previous_node->data->radius = distances[i];
          previous_node->next = NULL;
          previous_node->prev = NULL;
          lidar_data_start = previous_node;
        }
        else {
          doubly_linked_list_node<lidar_datapoint> * node = new doubly_linked_list_node<lidar_datapoint>;
          node->data = new lidar_datapoint;
          node->data->theta = i;
          node->data->radius = distances[i];
          node->prev = previous_node;
          previous_node->next = node;
          previous_node = node;
        }
      }
    }
  }

  if (previous_node != NULL) {
    previous_node->next = lidar_data_start;
    lidar_data_start->prev = previous_node;
  }
  else {
    lidar_data_start = NULL;
  }
}

/**
  Code for LiDAR logging
*/
void serial_lidar_log() {
  if (Serial.available()) {
    char request = Serial.read();
    if (request == '0') {
      Serial.println(lidar_speed);
      Serial.print("#");
    }
    else if (request == '1') {
      for (uint16_t i = 0; i < 360; i++) {
        if (i < min_angle || i > max_angle) {
          if (distances[i] != 0) {
            for (uint8_t j = 1; j < 3; j++) {
              if (i < pow(10, j)) Serial.print("0");
            }
            Serial.print(i);
            Serial.print(",");
            for (uint8_t j = 1; j < 4; j++) {
              if (distances[i] < pow(10, j)) Serial.print("0");
            }
            Serial.println(distances[i]);
            delayMicroseconds(2);
          }
        }
      }
      Serial.print("#");
    }
    else if (request == '2' && line_data_start != NULL) {
      doubly_linked_list_node<line> * node = line_data_start->next;
      bool finished = false;

      while (node != line_data_start && !finished) {
        for (uint8_t j = 1; j < 3; j++) {
          if (node->data->start_x < pow(10, j)) Serial.print("0");
        }
        Serial.print(node->data->start_x);
        Serial.print(",");
        for (uint8_t j = 1; j < 3; j++) {
          if (node->data->start_y < pow(10, j)) Serial.print("0");
        }
        Serial.print(node->data->start_y);
        Serial.print(",");
        for (uint8_t j = 1; j < 3; j++) {
          if (node->data->end_x < pow(10, j)) Serial.print("0");
        }
        Serial.print(node->data->end_x);
        Serial.print(",");
        for (uint8_t j = 1; j < 3; j++) {
          if (node->data->end_y < pow(10, j)) Serial.print("0");
        }
        Serial.print(node->data->end_y);
        Serial.print(",");
        delayMicroseconds(2);
        if (node == line_data_start->prev) {
          finished = true;
        }
        node = node->next;
      }
      Serial.print("#");
    }
    else if (request == '3') {
      Serial.print(boiler.x);
      Serial.print(",");
      Serial.println(boiler.y);
      Serial.print("#");
    }
  }
}
