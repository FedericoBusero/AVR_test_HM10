/*
/*
  Arduino BLE Shield (HM-10) Testing Sketch
  by JP Liew http://jpliew.com
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.


  Changes by FedericoBusero:
  - set notification OFF (necessary for Blynk)
*/

#include <SoftwareSerial.h>

#define BUFFER_LENGTH 100

#define LED_PIN 13

SoftwareSerial ble(2, 3);       // For Uno, HM10 TX pin to Arduino Uno pin D2, HM10 RX pin to Arduino Uno pin D3
// SoftwareSerial ble(10,11);      // For Uno, HM10 TX pin to Arduino Uno pin 10, HM10 RX pin to Arduino Uno pin 11
#define SERIAL_BLE ble

// #define SERIAL_BLE Serial1 // For Mega, ... HM10 TX pin to Arduino RX ping, HM10 TX pin to Arduino RX pin

char buffer[BUFFER_LENGTH];       // Buffer to store response
int timeout = 800;          // Wait 800ms each time for BLE to response, depending on your application, adjust this value accordingly
long bauds[] = {9600, 57600, 115200, 38400, 2400, 4800, 19200}; // common baud rates, when using HM-10 module with SoftwareSerial, try not to go over 57600

long BLEAutoBaud() {
  int baudcount = sizeof(bauds) / sizeof(long);
  for (int i = 0; i < baudcount; i++) {
    for (int x = 0; x < 3; x++) { // test at least 3 times for each baud
      Serial.print("Testing baud ");
      Serial.println(bauds[i]);
      SERIAL_BLE.begin(bauds[i]);
      if (BLEIsReady()) {
        return bauds[i];
      }
    }
  }
  return -1;
}

boolean BLEIsReady() {
  BLECmd(timeout, "AT" , buffer);   // Send AT and store response to buffer
  if (strcmp(buffer, "OK") == 0) {
    return true;
  } else {
    return false;
  }
}

boolean BLECmd(long timeout, char* command, char* temp) {
  long endtime;
  boolean found = false;
  endtime = millis() + timeout; //
  memset(temp, 0, BUFFER_LENGTH);       // clear buffer
  found = true;
  Serial.print("Arduino send = ");
  Serial.println(command);
  SERIAL_BLE.print(command);

  // The loop below wait till either a response is received or timeout
  // The problem with this BLE Shield is most of the HM-10 modules do not response with CR LF at the end of the response,
  // so a timeout is required to detect end of response and also prevent the loop locking up.

  while (!SERIAL_BLE.available()) {
    if (millis() > endtime) {   // timeout, break
      found = false;
      break;
    }
  }

  if (found) {            // response is available
    int i = 0;
    while (SERIAL_BLE.available()) {   // loop and read the data
      char a = SERIAL_BLE.read();
      // Serial.print((char)a); // Uncomment this to see raw data from BLE
      temp[i] = a;        // save data to buffer
      i++;
      if (i >= BUFFER_LENGTH) break; // prevent buffer overflow, need to break
      delay(1);           // give it a 2ms delay before reading next character
    }
    Serial.print("BLE reply    = ");
    Serial.println(temp);
    while ((strlen(temp) > 0) && ((temp[strlen(temp) - 1] == 10) || (temp[strlen(temp) - 1] == 13)))
    {
      temp[strlen(temp) - 1] = 0;
    }
    return true;
  } else {
    Serial.println("BLE timeout!");
    return false;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  long baudrate = BLEAutoBaud();

  if (baudrate > 0) {
    Serial.print("Found BLE baud rate ");
    Serial.println(baudrate);
  } else {
    Serial.println("No BLE detected.");
    while (1) {};         // No BLE found, just going to stop here
  }

  // The following commands are just to demonstrate the shield is working properly,
  // in actual application, only call those that are needed by your application.
  // Check HM-10 datasheet for the description of the commands.
  BLECmd(timeout, "AT+NAME?", buffer);
  BLECmd(timeout, "AT+BAUD?", buffer);
  BLECmd(timeout, "AT+MODE?", buffer);
  BLECmd(timeout, "AT+PASS?", buffer);
  BLECmd(timeout, "AT+VERS?", buffer);
  BLECmd(timeout, "AT+RADD?", buffer);
  BLECmd(timeout, "AT+ADDR?", buffer);
  BLECmd(timeout, "AT+TYPE?", buffer);
  BLECmd(timeout, "AT+POWE?", buffer); // Show power (should be 2 or 3)
  BLECmd(timeout, "AT+NOTI?", buffer); // Check the current notification bit
  BLECmd(timeout, "AT+NOTI0", buffer); // Make sure the module doesn't send AT+CONNECT or AT+LOST on the serial line (very important for Blynk users)

  //  BLECmd(timeout,"AT+NAMEHM10",buffer); // Set the name of the module to HM10

  //  BLECmd(timeout,"AT+BAUD0",buffer); // Set baudrate to 9600
  //  BLECmd(timeout,"AT+BAUD1",buffer); // Set baudrate to 19200
  //  BLECmd(timeout,"AT+BAUD2",buffer); // Set baudrate to 38400
  //  BLECmd(timeout,"AT+BAUD3",buffer); // Set baudrate to 57600
  //  BLECmd(timeout,"AT+BAUD4",buffer); // Set baudrate to 115200

  Serial.println("----------------------");
  Serial.println("Waiting for remote connection...");
}

void printhex(unsigned char c)
{
  if (c < 0x10) {
    Serial.print("0");
  }
  Serial.print(c, HEX);
}

void loop() {
  if (SERIAL_BLE.available()) {
    char c = (char)SERIAL_BLE.read();
    if (isPrintable(c))
    {
      Serial.print(c);
    }
    else
    {
      Serial.print("[");
      printhex(c);
      Serial.print("]");
    }
    if (c == '1') digitalWrite(LED_PIN, HIGH); // if received character 1 from BLE, set PIN LED_PIN high
    if (c == '0') digitalWrite(LED_PIN, LOW); // if received character 0 from BLE, set PIN LED_PIN low
  }
}

