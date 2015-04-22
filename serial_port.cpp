#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <stdint.h>
#include <fstream>

#define BAUDRATE B38400
#define DEVICE "/dev/ttyO1"
#define _POSIX_SOURCE 1
#define FALSE 0
#define TRUE 1

#define MH_BYTE 8
#define ML_BYTE 9
#define KH_BYTE 12
#define KL_BYTE 13


volatile int STOP=FALSE;


int wait_flag=FALSE;

using namespace std;


bool capture(int fd){
  // Send the take picture command to the device.
  uint8_t picture_command[] = { 0x56, 0x00, 0x36, 0x01, 0x00 };
  write (fd, picture_command, sizeof(picture_command));
  sleep(1);
  // Validate the return value
  uint8_t success[5] = { 0x76, 0x00, 0x36, 0x00, 0x00 };
  uint8_t return_val[5] = { 0x00 };
  read( fd, &return_val, sizeof(return_val));

  for(int i = 0; i < 5; i++)
  {
    if(success[i] != return_val[i])
    {
      return 0;
    }
  }
  return 1;
}



uint16_t read_size(int fd)
{
  // Write the command out to the serial port
  uint8_t size_command[] = { 0x56, 0x00, 0x34, 0x01, 0x00 };
  write (fd, size_command, sizeof(size_command));
  sleep(1);
  // Check the returned value against our reference. First 7 bytes should match.
  uint8_t return_ref[7] = { 0x76, 0x00, 0x34, 0x00, 0x04, 0x00, 0x00 };
  uint8_t return_val[9] = { 0x00 };
  read( fd, &return_val, sizeof(return_val));

  for(int i = 0; i < 7; i++){
    if(return_val[i] != return_ref[i]){
      std::cout << "Mismatch in return value." << std::endl;
      return 0;
    }
  }

  // If all checks out, reconstruct the file size from bytes.
  uint16_t file_size = (return_val[7] << 8) | return_val[8];
  cout<<"FILE SIZE: "<<file_size<<endl;
  return file_size;
}





void read_jpeg(int fd, std::ofstream& output_file)
{
  uint8_t command[16] = { 0x56, 0x00, 0x32, 0x0C, 0x00, 0x0A, 0x00, 0x00,
                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A };
  uint16_t m = 0; // Start address
  uint16_t k = 32; // Chunk size

  uint16_t file_size = read_size(fd);
  uint16_t bytes_read = 0;
  uint8_t last_byte = 0;

  bool done = false;
  while(!done) {
    uint8_t jpeg_data[32] = { 0x00 };

    // Build the command packet
    command[MH_BYTE] = (m >> 8) & 0xff;
    command[ML_BYTE] = (m >> 0) & 0xff;
    command[KH_BYTE] = (k >> 8) & 0xff;
    command[KL_BYTE] = (k >> 0) & 0xff;

    // Send it
    write (fd, command, sizeof(command));
    sleep(1);
    // Read off header of reply
    uint8_t header[5] = { 0x00 };
    read( fd, &header, sizeof(header));
    
    if (header[0] == 0x76 && header[1] == 0x00 && header[2] == 0x32
        && header[3] == 0x00 && header[4] == 0x00){
      read( fd, &jpeg_data, sizeof(jpeg_data));
    } else {
      std::cout << "Header mismatch." << std::endl;
    }

    // Check for cross-packet end sequence
    if(last_byte == 0xFF && jpeg_data[0] == 0xD9){
      done = true;
    }

    // We're going to jump over the first byte, better write it out.
    output_file << jpeg_data[0];
    bytes_read++;

    // Write out the remaining bytes, and check to see if we've read the end.
    for(int i = 1; i < 32; i++){
      // Write data to output file...
      output_file << jpeg_data[i];
      bytes_read++;

      // Check for the end of the file.
      if(jpeg_data[i - 1] == 0xFF && jpeg_data[i] == 0xD9){
        done = true;
        break;
      }
    }

    // Stash our last byte to check for end sequence if it spans packets
    last_byte = jpeg_data[31];

    // Read footer, check for error conditions
    uint8_t footer[5] = { 0x00 };
    read( fd, &footer, sizeof(footer));
    if (!(footer[0] == 0x76 && footer[1] == 0x00 && footer[2] == 0x32
          && footer[3] == 0x00 && footer[4] == 0x00)){
      std::cout << "Mismatched footer!" << std::endl;
    }

    // Update address.
    m += 32;
    std::cout << "\rRead " << bytes_read << " of " << file_size 
              << " bytes." << std::flush;
  }
  std::cout << std::endl;
}



int main()

{

    system ("echo BB-UART1 > /sys/devices/bone_capemgr.9/slots");
    int fd;
    
    ofstream output_file;
    output_file.open ("image.jpg");
    
    fd = open (DEVICE, O_RDWR | O_NOCTTY);
    if (fd <0) 
    {
		perror(DEVICE); 
        exit (-1);
    }
    

    struct termios tty, tty_old;

    tty_old = tty;
    cfsetospeed (&tty, (speed_t)B38400);
    cfsetispeed (&tty, (speed_t)B38400);
    
  /*  tty.c_cflag     &=  ~PARENB;            // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;
    
    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN]   =  1;                  // read doesn't block
    tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
    
    cfmakeraw(&tty);

    tcflush(fd,TCIFLUSH);
    
    if ( tcsetattr ( fd, TCSANOW, &tty ) != 0) 
    {
       cout << "Error " << " from tcsetattr" << std::endl;
    }
   */   
    
   uint8_t reset[] = { 0x56, 0x00, 0x26, 0x00 };
   write (fd, reset, sizeof(reset));
   uint8_t discard[100] = { 0xff };
   sleep(1);
   read( fd, &discard, sizeof(discard));
   for (int i = 0; i < 100; i++)
   {
	    cout<<(char)discard[i]<<" ";   
   }
   cout<<endl;
    
  if(capture(fd))
  { // Success!
    std::cout << " Done." << std::endl;
    std::cout << "Reading image from camera..." << std::endl;

    // Read in the JPEG.
    read_jpeg(fd, output_file);
  }
  else 
  {
    std::cout << "Capture failed. Check connection and try again." << std::endl;
  }
   
    return 1;


}
