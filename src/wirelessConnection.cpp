#include <WiFi.h>
#include <WiFiUdp.h>
#include "wirelessConnection.h"

// Wi-Fi credentials
const char* ssid = "ESP32Server";   // Set your custom SSID
const char* password = "12345678"; // Set your custom password

// UDP configuration
const int port = 5000; // Port to listen on
WiFiUDP udp;

void wifiSetup(){
  // Set up Wi-Fi as an Access Point
  WiFi.softAP(ssid, password);
  // Display receiver's IP
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Server IP address: ");
  Serial.println(IP);
  // Start listening for UDP packets
  udp.begin(port);
  Serial.println("UDP server started");
}

void wifiReceive(){
  int packetSize = udp.parsePacket();
  if (packetSize) {  
    uint8_t packet[32];
    int len = udp.read(packet, sizeof(packet));
    Serial.print("Received Data: ");
    for (int i = 0; i < len; i++) {
      Serial.print(packet[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

