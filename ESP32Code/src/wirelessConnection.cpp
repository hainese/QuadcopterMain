#include <WiFi.h>
#include <WiFiUdp.h>
#include "wirelessConnection.h"

// Wi-Fi credentials
const char* ssid = "ESP32Drone";   // Set your custom SSID
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

std::array<uint8_t, 32> wifiReceive() {
    std::array<uint8_t, 32> packet = {}; // Initialize with 0s
    int packetSize = udp.parsePacket();
    if (packetSize) {  
        int len = udp.read(packet.data(), packet.size());
        /*Serial.print("Received packet: ");
        for (int i = 0; i < len; i++) {
            Serial.printf("%02X ", packet[i]);
        }
        Serial.println();*/
    }
    udp.flush();

    return packet; // Safe copy (RVO or move semantics apply)
}


