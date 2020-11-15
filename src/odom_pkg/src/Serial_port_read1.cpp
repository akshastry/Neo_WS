// C library headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <chrono>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

// ROS headers
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

double phi = 0.0, theta = 0.0, psi = 0.0, p = 0.0, q = 0.0, r = 0.0;
double Td, phid, thetad, psid;

// Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
int serial_port = open("/dev/ttyS4", O_RDWR);

void parse(char data[], uint8_t end);
void ftoa(double n, char res[], int afterpoint);
int intToStr(int x, char str[], int d);
void reverse(char* str, int len);
int sumOfDigits(int x);

void odomCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // printf("Hi\n");
  char ser_msg[100];
  int checksum = sumOfDigits((int) (msg->linear.z*100000)) + sumOfDigits((int) (msg->angular.x*100000))
  + sumOfDigits((int) (msg->angular.y*100000)) + sumOfDigits((int) (msg->angular.z*100000));

  sprintf(ser_msg, "Hi,%.5f,%.5f,%.5f,%.5f,%d\n", msg->linear.z, msg->angular.x, msg->angular.y,
           msg->angular.z, checksum);
  printf("%s", ser_msg);

  write(serial_port, ser_msg, sizeof(ser_msg));
}

int main(int argc, char** argv) {

  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag |= PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag |= ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate
  cfsetispeed(&tty, B2000000);
  // cfsetospeed(&tty, B2000000);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }

//Create ros node and publish/subscribe
  ros::init(argc, argv, "serialcom");

  ros::NodeHandle n;

  ros::Publisher Fcon_pub = n.advertise<geometry_msgs::Twist>("/serialcom/attitude_and_rate", 1);
  ros::Subscriber sub = n.subscribe("/neo/odometry", 1, odomCallback);



  geometry_msgs::Twist attitude_and_rate;

  // Allocate memory for read buffer, set size according to your needs
  char data [256];
  // char msg[50];
  
  // exit(0);

  // Normally you wouldn't do this memset() call, but since we will just receive
  // ASCII data for this example, we'll set everything to 0 so we can
  // call printf() easily.
  // memset(&data, '\0', sizeof(data));

  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  uint8_t num_bytes, index, nb;
  int i,j;
  FILE *fp;

  double pub_time = 0.0, time_now = 0.0;

  time_t t = time(0);
  struct tm ltm = *localtime(&t);

  char filename[300], time_stamp[100]; unsigned long long int tm_stmp;

sprintf(filename , "/home/su/Dropbox/Quadrotor_flight_control/Arduino_playground/LOGS/%d_%d_%d_%d_%d_%d.csv",
   1900+ltm.tm_year, 1+ltm.tm_mon, ltm.tm_mday, ltm.tm_hour, ltm.tm_min, ltm.tm_sec);

  printf("Writing to file %s\n",filename);

  while(ros::ok())
  {

    fp = fopen(filename, "a");
    for (j = 0; j < 500; ++j)
    {
      // get a line
      num_bytes = read(serial_port, &data, sizeof(data));
      data[num_bytes] = '\0';//insert endline character to /n

      // get index of last ,
      for (i = 2; i < num_bytes; ++i)
      {
        if(data[num_bytes-i] == ',')
        {
          index = i;
          break;
        }
      }
      //get the number of bytes (should be after last ,)
      nb = 0;
      for (i = index-1; i >1 ; --i)
      {
        nb = nb*10 + data[num_bytes - i] - '0';
      }

      // printf("Hi\n");

      //check data validity
      if(data[0] == 'H' && data[1] == 'i' && data [2] == ',' && nb == num_bytes - index)
      {
        if(data[3] == 'p' && data[4] == 'r' & data[5] == 'm' && data[6] == ',')
        {
          parse(data, num_bytes - index - 1);
        }
        else
        {
          // puts(data);  
          tm_stmp = std::chrono::system_clock::now().time_since_epoch()/std::chrono::microseconds(1);
          sprintf(time_stamp, "%lf,", tm_stmp/1000000.0 );
          fputs(time_stamp, fp);
          fputs(data, fp);  
        }
        
      }
      else
      {
        puts(data);
      }


    }
    fclose(fp);

    tm_stmp = std::chrono::system_clock::now().time_since_epoch()/std::chrono::microseconds(1);
    time_now = tm_stmp / 1000000.0;

    if (time_now - pub_time >= 1.0/200.0)
    {
      attitude_and_rate.linear.x  = phi    ;
      attitude_and_rate.linear.y  = theta  ;
      attitude_and_rate.linear.z  = psi    ;
      attitude_and_rate.angular.x = p      ;
      attitude_and_rate.angular.y = q      ;
      attitude_and_rate.angular.z = r      ;

      Fcon_pub.publish(attitude_and_rate);
      ros::spinOnce();

      tm_stmp = std::chrono::system_clock::now().time_since_epoch()/std::chrono::microseconds(1);
      pub_time = tm_stmp / 1000000.0;

    }
    
        // printf("Hi\n");
  }

  close(serial_port);
  return 0; // success
}

void parse(char data[], uint8_t end)
{
  char value[20];
  memcpy(value, &data[11], end-10);
  value[end-10] = '\0';
  if(data[7] == 'p' && data[8] == 'h' && data[9] == 'i' && data[10] == ',')
  {
    phi = atof(value);
    // printf("%lf,%s",phi,data);
  }
  else if(data[7] == 't' && data[8] == 'h' && data[9] == 't' && data[10] == ',')
  {
    theta = atof(value); 
  }
  else if(data[7] == 'p' && data[8] == 's' && data[9] == 'i' && data[10] == ',')
  {
    psi = atof(value);
  }
  else if(data[7] == '_' && data[8] == 'p' && data[9] == '_' && data[10] == ',')
  {
    p = atof(value);
  }
  else if(data[7] == '_' && data[8] == 'q' && data[9] == '_' && data[10] == ',')
  {
    q = atof(value);
  }
  else if(data[7] == '_' && data[8] == 'r' && data[9] == '_' && data[10] == ',')
  {
    r = atof(value);
  }
  else
  {
    puts(data);
  }
}

void ftoa(double n, char res[], int afterpoint) 
{ 
    // Extract integer part 
    int ipart = (int)n; 
  
    // Extract floating part 
    double fpart = n - (double)ipart; 
  
    // convert integer part to string 
    int i = intToStr(ipart, res, 0); 
  
    // check for display option after point 
    if (afterpoint != 0) { 
        res[i] = '.'; // add dot 
  
        // Get the value of fraction part upto given no. 
        // of points after dot. The third parameter  
        // is needed to handle cases like 233.007 
        fpart = fpart * pow(10, afterpoint); 
  
        intToStr((int)fpart, res + i + 1, afterpoint); 
    }

}

// If d is more than the number of digits in x,  
// then 0s are added at the beginning. 
int intToStr(int x, char str[], int d) 
{ 
    int i = 0; 
    while (x) { 
        str[i++] = (x % 10) + '0'; 
        x = x / 10; 
    } 
  
    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d) 
        str[i++] = '0'; 
  
    reverse(str, i); 
    str[i] = '\0'; 
    return i; 
} 

// Reverses a string 'str' of length 'len' 
void reverse(char* str, int len) 
{ 
    int i = 0, j = len - 1, temp; 
    while (i < j) { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; 
        j--; 
    } 
}

int sumOfDigits(int x)
{
    int sum = 0;
    while (x != 0)
    {
        sum += x %10;
        x   = x /10;
    }
    return sum;
}