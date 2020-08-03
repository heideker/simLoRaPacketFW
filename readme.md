
simLoRaPacketFW
===============

This software was created through some modifications on the original SEMTECH Lora Gateway packet forwarder to simulate the presence of a LoRa shield and create a UDP socket.

Trought this modification is possible to reproduce the real radio traffic on LoRa gateway sending packets to 1777 UDP port by a special propose application.

General schemme:
===============

 +----------+
 | SPI LoRa |
 |  shield  |
 +-----+----+
       |
       X
       |
+------+------+
|  Modified   |
| LoRa Packet +--------> to LoRaWan origial stack
|  Forwarder  |
+------+------+
       |
       |
       ^
     Socket 
    UDP 1777
       ^
       |
+------+------+
| Application |
+-------------+


Structure of the packet:

struct pacoteUDP {
    uint8_t header[6]; // header,dest,source,pktnum,retry 
    uint8_t size;
    uint8_t payload[251]; 
} bufferALE;




*EOF*
