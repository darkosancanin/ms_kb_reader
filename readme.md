nRF24L01+:  
1: (square): GND  
2: (next row of 4): 3.3 VCC  
3: CE 9  
4: CSN; 8  
5: SCK: 13  
6: MOSI 11  
7: MISO: 12  
8: IRQ: not used here  
  
microsoft keyboard packet structure:  
 struct mskb_packet  
 {  
 uint8_t device_type;  
 uint8_t packet_type;  
 uint8_t model_id;  
 uint8_t unknown;  
 uint16_t sequence_id;  
 uint8_t flag1;  
 uint8_t flag2;  
 uint8_t d1;  
 uint8_t key;  
 uint8_t d3;  
 uint8_t d4;  
 uint8_t d5;  
 uint8_t d6;  
 uint8_t d7;  
 uint8_t checksum;  
 };  # ms_kb_reader 
