#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>
#include <ArduinoSTL.h>
#include <BasicLinearAlgebra.h>
#include <queue>
#include <L298NX2.h>

// Pin definition
const unsigned int EN_A = 9;
const unsigned int IN1_A = 4;
const unsigned int IN2_A = 5;

const unsigned int IN1_B = 3;
const unsigned int IN2_B = 6;
const unsigned int EN_B = 7;

// Initialize both motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

// Initial speed
unsigned short theSpeedA = 55;
unsigned short theSpeedB = 55;

// Keep track of direction
unsigned int theDirection = 1;

#define PWM_PIN_MOTOR_A 9
#define MOTOR_A_N1 4
#define MOTOR_A_N2 5
#define PWM_PIN_MOTOR_B 3
#define MOTOR_B_N3 6
#define MOTOR_B_N4 7
#define RFID_Transmitter 13
#define RFID_Reciever 12

RF24 radio(10, 11);
RF24Network network(radio);
RF24Mesh mesh(radio, network);


struct Location {
  float x;
  float y;
  float z;

   double distance = sqrt(pow(tag_position.x - car_position.x, 2) + pow(tag_position.y - car_position.y, 2));
};

struct TOFData {
  byte tagID;
  byte nodeID;
  unsigned long TOF;
};

struct TagData {
  byte tagID;
  unsigned long ToFs[3];  // Storing ToF data from the master and the 2 pseudo-masters
};

unsigned long receiverTOFs[2];  // ToF data from pseudo-masters to master
Location receiverComputedLocations[2];  // The computed locations
const int MAX_TAGS = 3; // or any limit you want
TagData tags[MAX_TAGS]; // Collection of tags data.
TOFData receivedTOFs[3]; // You expect data from 3 nodes in total
byte receivedCount = 0;
Location receiverLocations[3];
const uint8_t maxRetries = 3;
long timestamp1, timestamp2;
int known_ids[] = {1};
std::vector<int> current_path;
int current_node_index = 0;
const double CLOSE_ENOUGH_THRESHOLD = 10.0;  // This value should be adjusted according to your setup.




// Definitions for the nodes and edges
class PRMNode {
public:
    int id;
    double x, y, z;
    std::vector<int> connected_nodes;

    PRMNode(int id, double x, double y, double z): id(id), x(x), y(y), z(z) {}

    void update_position(double nx, double ny, double nz) {
        x = nx; y = ny; z = nz;
    }
};

void storeTOF(byte tagID, byte nodeID, unsigned long TOF) {
  for(int i = 0; i < MAX_TAGS; i++) {
    if(tags[i].tagID == tagID) {
      tags[i].ToFs[nodeID] = TOF;
      return;
    }
  }

  // If we get here, we haven't found the tag in the collection yet
  // This means it's a new tag, and we should add it to our collection
  for(int i = 0; i < MAX_TAGS; i++) {
    if(tags[i].tagID == 0) { // Assuming 0 means the tagID hasn't been set
      tags[i].tagID = tagID;
      tags[i].ToFs[nodeID] = TOF;
      return;
    }
  }
}

void log_tof() {
  if (radio.available()) {
    char header;
    radio.read(&header, sizeof(header));

    switch(header) {
      case 'T': // Tag's timestamped response to the master
        masterReceivedTagTime = micros();
        computeAndStoreMasterTOF();
        break;
      default:
        TOFData data;
        radio.read(&data, sizeof(data));
        byte tagID; // You'd get this from the data payload; maybe add it to the TOFData struct?
        storeTOF(tagID, data.nodeID, data.TOF);
        break;
    }
  }
}
//Note: The above design assumes the Master can differentiate between messages from different tags by reading a tag-specific header byte first. If you're using addresses to differentiate between tags, you may need to integrate that into your logic.
//Remember to manage the memory properly, especially if your tags can enter and leave the environment, or if you have a large number of them. A fixed-size array as shown might not be the best solution for a more dynamic or larger environment; you might consider using a dynamic data structure (like a list) and handling potential memory fragmentation concerns, especially since microcontrollers typically have limited RAM.


void update_tag_position(int tag_id, double x, double y, double z) {
    for (auto& node : nodes) {
        if (node.id == tag_id) {
            node.update_position(x, y, z);
            break;
        }
    }
}



class PRMEdge {
  public:
    int node1, node2;
    double cost;

    PRMEdge(int node1, int node2, double cost): node1(node1), node2(node2), cost(cost) {}
};
std::vector<PRMNode> nodes;
std::vector<PRMEdge> edges;

void sync_clock() {
  long averageTime;

  // Request time from Node 1
  radio.startListening();
  uint8_t retries = 0;
  while (!radio.available(1) && retries < maxRetries) {
    delay(10);
    retries++;
  }
  if (radio.available(1)) {
    radio.read(&timestamp1, sizeof(timestamp1));
  } else {
    Serial.println("Error: Lost packet from Node 1");
  }

  // Request time from Node 2
  retries = 0;
  while (!radio.available(2) && retries < maxRetries) {
    delay(10);
    retries++;
  }
  if (radio.available(2)) {
    radio.read(&timestamp2, sizeof(timestamp2));
  } else {
    Serial.println("Error: Lost packet from Node 2");
  }
  radio.stopListening();

  // Calculate average time
  averageTime = (millis() + timestamp1 + timestamp2) / 3;

  // Send correction offset to Node 1
  radio.openWritingPipe(receiverLocations[1]);
  long offset1 = averageTime - timestamp1;
  if (!radio.write(&offset1, sizeof(offset1))) {
    Serial.println("Error: Failed to send correction to Node 1");
  }

  // Send correction offset to Node 2
  radio.openWritingPipe(receiverLocations[2]);
  long offset2 = averageTime - timestamp2;
  if (!radio.write(&offset2, sizeof(offset2))) {
    Serial.println("Error: Failed to send correction to Node 2");
  }

  delay(5000);  // Sync every 5 seconds
}



void add_node(PRMNode node) {
  nodes.push_back(node);
}

void add_edge(PRMEdge edge) {
  edges.push_back(edge);
  nodes[edge.node1].connected_nodes.push_back(edge.node2);
  nodes[edge.node2].connected_nodes.push_back(edge.node1);
}


double heuristic(int id1, int id2) {
   PRMNode& node1 = nodes[id1];
  PRMNode& node2 = nodes[id2];
  return sqrt(pow(node1.x - node2.x, 2) + pow(node1.y - node2.y, 2) + pow(node1.z - node2.z, 2));
}

class CompareCost {
  bool operator()(const PRMNode* a, const PRMNode* b) const {
    return a->f_cost > b->f_cost;
  }
};


void initializeMeshNetwork() {
  mesh.setNodeID(0);
  if (!mesh.begin()) {
    Serial.println("Failed to start mesh network!");
    while (1);
  }
  Serial.println("Mesh network started.");
}


void requestReceiverTOFs() {
  // Wait a bit before starting the process
  delay(2000);

  for(uint8_t i = 1; i <= 2; i++) {
    RF24NetworkHeader header(i, 'R'); // 'R' for "Request ToF"
    network.write(header, "REQ_TOF", 8);
    
    // Wait a bit for the reply
    delay(200);
    
    while (network.available()) {
      RF24NetworkHeader header;
      unsigned long incomingToF;  // Assuming the ToF data is sent as an unsigned long

      network.read(header, &incomingToF, sizeof(incomingToF));
      if (header.type == 'T') { // 'T' for "ToF Data"
        receiverTOFs[header.from_node - 1] = incomingToF;
      }
    }
  }
}

void computeReceiverLocations() {
    // Assuming speed of radio waves as the speed of light: 299,792,458 m/s
    double speedOfRadioWaves = 299792.458;  // in km/ms, for ease

    // Convert ToF to distance (d = speed x time)
    double d1 = receiverTOFs[0] * speedOfRadioWaves;
    double d2 = receiverTOFs[1] * speedOfRadioWaves;

    receiverComputedLocations[0] = {d1, 0, 0};  // On the x-axis

    // For the second receiver:
    double x2 = d1; // As given above
    double y2 = sqrt(d2*d2 - d1*d1);
    receiverComputedLocations[1] = {x2, y2, 0};  // In the x-y plane

    // If there's a third pseudo-master, the mathematics becomes more complex and involves solving a set of equations.
}



double calculate_distance(double time_of_flight) {
    double speed_of_light = 299792458; // in m/s
    return speed_of_light * time_of_flight;
}



void log_tag_ids(){
    for(int i = 0; i < MAX_TAGS; i++) {
        if(tags[i].tagID != 0) { // Only consider tags with valid IDs
            // Fetch the stored TOF data for each receiver (master and pseudo-master nodes)
            double tof1 = tags[i].ToFs[0];  // Time of Flight data for master node
            double tof2 = tags[i].ToFs[1];  // Time of Flight data for first pseudo-master node
            double tof3 = tags[i].ToFs[2];  // Time of Flight data for second pseudo-master node

            // Convert TOF to distances
            float d1 = calculate_distance(tof1);
            float d2 = calculate_distance(tof2);
            float d3 = calculate_distance(tof3);

            // Calculate tag positions using the distances
            calculate_tag_positions(tags[i].tagID, d1, d2, d3);
        }
    }
}




void calculate_tag_positions(tag_id, double d1, double d2, double d3) {

    // Assuming the definition for Location and receiverComputedLocations[] exists elsewhere in your code

    Location* r1 = &receiverComputedLocations[0]; // Master node, set as origin
    Location* r2 = &receiverComputedLocations[1];
    Location* r3 = &receiverComputedLocations[2];


    // Using r1 as the origin simplifies the equation for x1
    double x1 = pow(d1, 2);
    double x2 = pow(d2, 2) - pow(r2.x, 2) - pow(r2.y, 2) - pow(r2.z, 2) + x1;
    double x3 = pow(d3, 2) - pow(r3.x, 2) - pow(r3.y, 2) - pow(r3.z, 2) + x1;

    Matrix<3, 4> A = {1,-2*(r1.x),-2*(r1.y),-2*(r1.z), 
                     1,-2*(r2.x),-2*(r2.y),-2*(r2.z),
                     1,-2*(r3.x),-2*(r3.y),-2*(r3.z)};
    Matrix<3, 1> B = {x1, x2, x3};

    // Fill in A and B here...
    auto A_decomp = A;  // LUDecompose will destroy A here so we'll pass in a copy so we can refer back to A later
    auto decomp = LUDecompose(A_decomp);

    Matrix<4, 1> x_lusolve = LUSolve(decomp, B);

    // x_lusolve now contains the solution. Use it as needed.
    // Return or print the result as required
    Location TagPosition = {x_lusolve[0], x_lusolve[1], x_lusolve[2]};
    update_tag_position(tag_id, TagPosition.x, TagPosition.y, TagPosition.z);
    
    return TagPosition;
}

void start_navigation(int start_node, int target_node) {
    current_path = find_path(start_node, target_node);
    current_node_index = 0;
}

void update_navigation(Location car_position) {
    if(current_node_index < current_path.size()) {
        int target_node_id = current_path[current_node_index];

        // Drive to the current target node
        drive_to_next_node(target_node_id, car_position);

        // Find the position of the target node
        TagPosition tag_position;
        for (int i = 0; i < sizeof(tag_positions)/sizeof(tag_positions[0]); i++) {
            if (tag_positions[i].id == target_node_id) {
                tag_position = tag_positions[i];
                break;
            }
        }

        // Check if we are close enough to the current target node
        double distance = sqrt(pow(tag_position.x - car_position.x, 2) + pow(tag_position.y - car_position.y, 2));
        if (distance < CLOSE_ENOUGH_THRESHOLD) {
            // Move on to the next node in the path
            current_node_index++;
        }
    }
}

    // Calculate the angle to the next node
double calculate_angle_to_tag(const Location& tag_position, const Location& receiverComputedLocations) {
    Location referencePoint = {0, 0, 0};  // Assuming origin as reference

    double a = tag_position.distance(receiverComputedLocations);
    double b = tag_position.distance(referencePoint);
    double c = receiverComputedLocations.distance(referencePoint);

    // Apply Law of Cosines
    double cosTheta = (b*b + c*c - a*a) / (2*b*c);
    double angle_to_tag = std::acos(cosTheta);  // Angle in radians

    return angle_to_tag;  // If you want the angle in degrees, convert using: angle * (180.0/M_PI)
}

void drive_to_next_node(int node_id, Location car_position) {

       // Placeholder for car_position
   Location car_position = {0, 0, 0};
   wheelDiameter = 7.0;
   double revolutions = distance / (M_PI * wheelDiameter);

   // KP and K_ANGLE are control gains. You should define them somewhere.
   const double KP = 1.0;
   const double K_ANGLE = 1.0;
   // ... rest of your function ...


    // Find the position of the tag with the given ID
    TagPosition tag_position;
    for (int i = 0; i < sizeof(tag_positions)/sizeof(tag_positions[0]); i++) {
        if (tag_positions[i].id == node_id) {
            tag_position = tag_positions[i];
            break;
        }
    }

    // Calculate the distance to the target
    double distance = sqrt(pow(tag_position.x - car_position.x, 2) + pow(tag_position.y - car_position.y, 2));

    // Convert revolutions to motor steps or time (depending on your setup)
    int motorSteps = convertRevolutionsToSteps(revolutions);
  
    // Calculate the proportional control signal for distance
    int pwm_singal = (int)(motorSteps * KP); // KP is the proportional gain

    // Calculate the proportional control signal for angle
    // K_ANGLE is the proportional gain for the angle control
    int angle_signal = (int)(angle_to_tag * K_ANGLE);

    // Combine the two control signals to control the two motors
    // This assumes that the car turns by driving the two wheels at different speeds
    //NEED TO IMPLEMENT A DRIVE DISTANCE COMPUTATION
    if distance > 0.00
    leftMotor.forward(255);  // max speed
    rightMotor.forward(255);  // max speed

    if else distance < 0.00 
    leftMotor.backward(255);  // max speed
    rightMotor.backward(255);  // max speed

    else
    delay(motorsteps);
    leftMotor.stop();
    rightMotor.stop();
}

  void rotate(double radians) {
    double r = wheelbase / 2.0;  // radius
    double arcLength = r * radians;  // arc length for desired rotation
    double revolutions = arcLength / (M_PI * wheelDiameter);
    double wheelbase = 20.0;  // distance between tires, in cm
    double wheelDiameter = 7.0;  // diameter of the tire, in cm

    // Convert revolutions to motor steps or time (depending on your setup)
    int motorSteps = convertRevolutionsToSteps(revolutions);

    // Now rotate the robot by turning one motor forward and the other backward
    leftMotor.forward(255);  // max speed
    rightMotor.backward(255);  // max speed
    
    delay(motorSteps);  // delay for required time to achieve rotation
    
    leftMotor.stop();
    rightMotor.stop();

}

const int MAX_NODES = 100;  // Adjust based on your needs

struct PRMNode {
    int id;
    double g_cost;
    double f_cost;
    int came_from;
    int connected_nodes[MAX_NODES];
    int connected_count;
};

PRMNode nodes[MAX_NODES];

double heuristic(int start, int end) {
    // Implement your heuristic here
    return 0.1;  // placeholder
}

int find_lowest_f_cost_node(int* open_set, int open_set_size) {
    int lowest_index = -1;
    double lowest_value = std::numeric_limits<double>::infinity();

    for (int i = 0; i < open_set_size; i++) {
        if (nodes[open_set[i]].f_cost < lowest_value) {
            lowest_value = nodes[open_set[i]].f_cost;
            lowest_index = i;
        }
    }
    
    return lowest_index;
}

int find_path(int start_node, int target_node, int* path) {
    for (int i = 0; i < MAX_NODES; i++) {
        nodes[i].g_cost = std::numeric_limits<double>::infinity();
        nodes[i].f_cost = std::numeric_limits<double>::infinity();
        nodes[i].came_from = -1;
    }

    nodes[start_node].g_cost = 0;
    nodes[start_node].f_cost = heuristic(start_node, target_node);

    int open_set[10];
    int open_set_size = 0;
    open_set[open_set_size++] = start_node;

    while (open_set_size > 0) {
        int current_index = find_lowest_f_cost_node(open_set, open_set_size);
        PRMNode& current = nodes[open_set[current_index]];

        // Remove the current node from open set
        open_set[current_index] = open_set[--open_set_size];

        if (current.id == target_node) {
            int path_length = 0;
            while (&current) {
                path[path_length++] = current.id;
                current = current.came_from == -1 ? nullptr : &nodes[current.came_from];
            }

            // Reverse the path
            for (int i = 0; i < path_length / 2; i++) {
                int temp = path[i];
                path[i] = path[path_length - 1 - i];
                path[path_length - 1 - i] = temp;
            }

            return path_length;  // Return the length of the path
        }

        for (int i = 0; i < current.connected_count; i++) {
            int neighbor_id = current.connected_nodes[i];
            PRMNode& neighbor = nodes[neighbor_id];
            double tentative_g_cost = current.g_cost + heuristic(current.id, neighbor.id);

            if (tentative_g_cost < neighbor.g_cost) {
                neighbor.came_from = current.id;
                neighbor.g_cost = tentative_g_cost;
                neighbor.f_cost = neighbor.g_cost + heuristic(neighbor.id, target_node);

                // Add to open set if not already there
                bool in_open_set = false;
                for (int j = 0; j < open_set_size; j++) {
                    if (open_set[j] == neighbor_id) {
                        in_open_set = true;
                        break;
                    }
                }

                if (!in_open_set) {
                    open_set[open_set_size++] = neighbor_id;
                }
            }
        }
    }

    return 0;  // Return 0 if no path was found
}



void manageMeshNetwork() {
  mesh.update();
  mesh.DHCP();
}

void setupPRM() {
    // Add nodes for each tag position.
    for (auto& position : tag_positions) {
      add_node(PRMNode(position.id, position.x, position.y, position.z));
    }

    // Create edges between all nodes.
    for (int i = 0; i < nodes.size(); ++i) {
      for (int j = i + 1; j < nodes.size(); ++j) {
        double cost = sqrt(pow(nodes[i].x - nodes[j].x, 2) +
                           pow(nodes[i].y - nodes[j].y, 2) +
                           pow(nodes[i].z - nodes[j].z, 2));
        add_edge(PRMEdge(i, j, cost));
      }
    }
}

  void startPRMTraversal() {
    int current_node_id = 0; // Assuming 0 as the starting node
    int target_node_id = nodes.size() - 1; // Assuming the last node as the target

    // This function initializes the path traversal.
    start_navigation(current_node_id, target_node_id);
}


void setup() {
  Serial.begin(9600);
  initializeMeshNetwork();
  requestReceiverLocations();
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(addresses[0]); // Master's address
  radio.setRetries(15, 15); // Set up retries
  radio.openReadingPipe(1, address[1]);
  radio.openReadingPipe(2, address[2]);
  radio.openReadingPipe(3, addresss[4]);
  motors.setSpeed(80);
}

void loop() {
  sync_clock();
  manageMeshNetwork();
  if (needPRMSetup) {
        setupPRM();
        startPRMTraversal();
  }
  Location car_position = get_car_position();
  update_navigation(car_position);
}












