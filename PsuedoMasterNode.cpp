#include <RF24.h>

RF24 radio(10, 11); // CE, CSN
const byte addresses[][6] = {"MNode", "1Node", "2Node", "TagA"};

unsigned long signalTime = 0;
unsigned long receivedTagTime = 0;
const unsigned long timeSlotDuration = 10000; // For example, 10 ms
const unsigned long myTimeSlot = 1; // Pseudo-master 1 gets the first time slot, for instance. Adjust for each pseudo-master.


void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(addresses[0]);  // This pseudo-master's address
  radio.openReadingPipe(1, addresses[3]);  // Tag's address
  radio.startListening();
}

void loop() {
  if (itIsMyTurnToSend()) {  // Some logic to decide when this pseudo-master should send
    requestTagResponse();
  }
  
  if (radio.available()) {
    char receivedData;
    radio.read(&receivedData, sizeof(receivedData));
    
    if (receivedData == 'T') {  // Tag's timestamped response
      receivedTagTime = micros();
      computeAndSendTOF();
    }
  }
}

bool itIsMyTurnToSend() {
  unsigned long currentTime = millis() % (timeSlotDuration * 2); 
  // The modulo operation wraps around the time to create repeating time slots

  if (currentTime >= myTimeSlot * timeSlotDuration && currentTime < (myTimeSlot + 1) * timeSlotDuration) {
    return true;
  } else {
    return false;
  }
}

void requestTagResponse() {
  radio.stopListening();
  radio.openWritingPipe(addresses[3]);  // Tag's address
  char signal = 'S';
  radio.write(&signal, sizeof(signal));
  signalTime = micros();
  radio.startListening();
}

void computeAndSendTOF() {
  unsigned long TOF = receivedTagTime - signalTime;

  TOFData data;
  data.nodeID = 1; // Set this to the current pseudo-master's ID
  data.TOF = TOF;
  
  radio.stopListening();
  radio.openWritingPipe(addresses[0]);  // Master's address
  radio.write(&data, sizeof(data));
  radio.startListening();
}
