#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>

RF24 radio(10, 11);            // CE, CSN
RF24Network network(radio);
RF24Mesh mesh(radio, network);

const uint8_t TAG_ID = 1;  // Unique for each tag. Change for each different tag.

struct TagMessage {
  uint8_t tag_id;
  // Add more data if necessary, like battery level, status, etc.
};

void initializeMeshNetwork() {
  mesh.setNodeID(TAG_ID);
  if (!mesh.begin()) {
    Serial.println("Failed to start mesh network!");
    while (1);
  }
  Serial.println("Mesh network started.");
}

void sendTagData() {
  TagMessage msg;
  msg.tag_id = TAG_ID;
  // Fill in any other data

  // Send the message to the master node
  RF24NetworkHeader header(0, 'T');  // 'T' stands for "Tag Data"
  if (!network.write(header, &msg, sizeof(msg))) {
    Serial.println("Failed to send tag data.");
  }
}

void receiveUpdates() {
  while (network.available()) {
    RF24NetworkHeader header;
    network.peek(header);
    
    switch (header.type) {
      // Add cases based on the message types you expect to receive
      case 'L': {  // 'L' for "Location Data"
        Location locationUpdate;
        network.read(header, &locationUpdate, sizeof(locationUpdate));
        // Do something with the location update
        break;
      }
      // ... add more cases as needed ...
    }
  }
}

void setup() {
  Serial.begin(9600);
  initializeMeshNetwork();
}

void loop() {
  mesh.update();
  mesh.DHCP();

  // Periodically send data
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime > 5000) {  // Send every 5 seconds
    sendTagData();
    lastSendTime = millis();
  }

  // Check for incoming updates
  receiveUpdates();
}
