//RealSense
#include <librealsense2/rs.hpp>

// C library headers
#include <iostream>
using namespace std;
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <chrono>
#include <execinfo.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


uint8_t radio_on = 0, old_radio_on = 0;

// Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
int serial_port = open("/dev/ttyS4", O_RDWR);

void parse(char data[], uint8_t end);
void ftoa(double n, char res[], int afterpoint);
int intToStr(int x, char str[], int d);
void reverse(char* str, int len);
int sumOfDigits(int x);

double constrain(double x, double a, double b)
{ 
  if(x<a) x = a;
  if(x>b) x = b;
  return x;
}

/* Obtain a backtrace and print it to stdout. */
void print_trace (void)
{
  void *array[10];
  char **strings;
  int size, i;

  size = backtrace (array, 10);
  strings = backtrace_symbols (array, size);
  if (strings != NULL)
  {

    printf ("Obtained %d stack frames.\n", size);
    for (i = 0; i < size; i++)
      printf ("%s\n", strings[i]);
  }

  free (strings);
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

// Realsense part
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  // Create a configuration for configuring the pipeline with a non default profile
  rs2::config cfg;
  // Add pose stream
  cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
  // Start pipeline with chosen configuration
  pipe.start(cfg);

//Create ros node and publish/subscribe
  // ros::init(argc, argv, "serialcom");

  // ros::NodeHandle n;

  // ros::Publisher Fcon_pub = n.advertise<geometry_msgs::Twist>("/serialcom/attitude_and_rate", 1);
  // ros::Publisher Alt_vel_pub = n.advertise<geometry_msgs::Vector3>("/serialcom/alt_vel_des", 1);
  // ros::Publisher Rdo_pub = n.advertise<std_msgs::UInt8>("/serialcom/radio", 1);
  // ros::Subscriber sub = n.subscribe("/neo/odometry", 1, odomCallback);
  // ros::Rate loop_rate(200);


  // geometry_msgs::Twist attitude_and_rate;
  // geometry_msgs::Vector3 alt_vel_des;
  // std_msgs::UInt8 rad_on;

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

  float mass = 1.5, g = 9.81, Thrust_sf = 2.2/(mass*g);

  float Kp_x = 4.0, Kd_x = 3.0, Ki_x = 0.0035;
  float Kp_y = 4.0, Kd_y = 3.0, Ki_y = 0.0035;
  float Kp_z = 4.0, Kd_z = 1.8, Ki_z = 0.007;
  float Kp_yaw = 1.0;

  double err_sum_x = 0.0, err_sum_y = 0.0, err_sum_z = 0.0;

  float X_d = 0.0, Y_d = 0.0, Z_d = -1.0, yaw_d = 0.0;

  double X = 0.0, Y = 0.0, Z = 0.0, VX = 0.0, VY = 0.0, VZ = 0.0, yaw = 0.0, new_yaw = 0.0;

  float LP_X = 0.1, LP_Y = 0.1, LP_Z = 0.1;
  float LP_VX = 0.1, LP_VY = 0.1, LP_VZ = 0.1;
  float LP_yaw = 0.1;

  double phi_d_ac = 0.0, theta_d_ac = 0.0;

  // geometry_msgs::Twist msg;

  double T_d, phi_d, theta_d, r_d, xdd, ydd, zdd;

  float msg_l_x, msg_l_z, msg_a_x, msg_a_y, msg_a_z;

  double t2, t3, t4, psi, theta;

  

  while(true)
  {
    try
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

      //check data validity
      if(data[0] == 'H' && data[1] == 'i' && data [2] == ',' && nb == num_bytes - index)
      {
        if(data[3] == 'p' && data[4] == 'r' & data[5] == 'm' && data[6] == ',')
        {
          // puts(data);
          parse(data, num_bytes - index - 1);
        }
      }

      // Wait for the next set of frames from the camera
      auto frames = pipe.wait_for_frames();
      // Get a frame from the pose stream
      auto f = frames.first_or_default(RS2_STREAM_POSE);
      // Cast the frame to pose_frame and get its data
      auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

      

      // // Print the x, y, z values of the translation, relative to initial position
      // printf("\rDevice Position: %f, %f, %f, confidence:%d\n", pose_data.translation.x, pose_data.translation.y, pose_data.translation.z, pose_data.tracker_confidence);

      if(pose_data.tracker_confidence >= 1)
      {
          X = (1 - LP_X) * X + LP_X * (-pose_data.translation.z - 0.155*(1-cos(yaw)) -0.155*(1-cos(theta_d_ac)));
          Y = (1 - LP_Y) * Y + LP_Y * (pose_data.translation.x - 0.155*sin(yaw));
          Z = (1 - LP_Z) * Z + LP_Z * (-pose_data.translation.y - 0.155*sin(theta_d_ac));

          VX = (1 - LP_VX) * VX + LP_VX * -pose_data.velocity.z;
          VY = (1 - LP_VY) * VY + LP_VY * pose_data.velocity.x;
          VZ = (1 - LP_VZ) * VZ + LP_VZ * -pose_data.velocity.y;

          
          //yaw is pitch for real sense axes
          t2 = +2.0 * (pose_data.rotation.w * pose_data.rotation.y - pose_data.rotation.z * pose_data.rotation.x);
          if(t2 > 1.0)
          {
              t2 = 1.0;
          }
          if(t2 < -1.0)
          {
              t2 = -1.0;
          }
          theta = asin(t2);

          t3 = +2.0 * (pose_data.rotation.w * pose_data.rotation.z + pose_data.rotation.x * pose_data.rotation.y);
          t4 = +1.0 - 2.0 * (pose_data.rotation.y * pose_data.rotation.y + pose_data.rotation.z * pose_data.rotation.z);
          psi = atan2(t3, t4);

          if(fabs(psi) > M_PI/2.0 && theta > 0.0)
          {
              new_yaw = theta - M_PI;
          }
          else if(fabs(psi) > M_PI/2.0 && theta < 0.0)
          {
              new_yaw = theta + M_PI;
          }
          else
          {
              new_yaw = -theta;
          }

          yaw = (1 - LP_yaw) * yaw + LP_yaw * new_yaw;

          if(radio_on == 0)
          {
            err_sum_x = 0.0;
            err_sum_y = 0.0;
            err_sum_z = 0.0;
          }

          if(old_radio_on == 0 and radio_on ==1)
          {
            X_d = X;
            Y_d = Y;
          }
          

          xdd = Kp_x * (X_d - X) + Kd_x * (0.0 - VX) + Ki_x * err_sum_x;
          ydd = Kp_y * (Y_d - Y) + Kd_y * (0.0 - VY) + Ki_y * err_sum_y;
          zdd = Kp_z * (Z_d - Z) + Kd_z * (0.0 - VZ) + Ki_z * err_sum_z;
          r_d = Kp_yaw * (yaw_d - yaw);

          T_d   = mass * (g - zdd);
          phi_d   = ydd/g + 0.0*M_PI/180.0;
          theta_d = -xdd/g + 0.7*M_PI/180.0;

          theta_d_ac = -(xdd - Ki_x*err_sum_x)/g;
          phi_d_ac = (ydd - Ki_y*err_sum_y)/g   ;

          err_sum_x = constrain(err_sum_x + (X_d - X), -0.2/Ki_x, 0.2/Ki_x);
          err_sum_y = constrain(err_sum_y + (Y_d - Y), -1.0/Ki_y, 1.0/Ki_y);
          err_sum_z = constrain(err_sum_z + (Z_d - Z), -10.0/Ki_z, 10.0/Ki_z);

          // no integral without takeoff
          if(Z > -0.25)
          {
            err_sum_z = 0.0;
            err_sum_y = 0.0;
            err_sum_x = 0.0;
            phi_d     = 0.0*M_PI/180.0;
            theta_d   = 0.7*M_PI/180.0;
          }

          // # publish to a topic
          msg_l_x   = radio_on;
          // msg.linear.y   = 0.0;
          msg_l_z   = constrain(T_d * Thrust_sf, 0.0, 3.0);
          msg_a_x  = constrain(phi_d * 180.0/M_PI, -15.0, 15.0) ;
          msg_a_y  = constrain(theta_d * 180.0/M_PI, -15.0, 15.0);
          msg_a_z  = constrain(r_d * 180.0/M_PI, -60, 60);

          char ser_msg[100];
          int checksum = sumOfDigits(round(msg_l_z*100000)) + sumOfDigits(round(msg_a_x*100000))
          + sumOfDigits(round(msg_a_y*100000)) + sumOfDigits(round(msg_a_z*100000))
          + sumOfDigits(round(msg_l_x*100000));

          sprintf(ser_msg, "Hi,%.5f,%.5f,%.5f,%.5f,%d,%d\n", msg_l_z, msg_a_x, msg_a_y,
                   msg_a_z, (int)(msg_l_x), checksum);
          // printf("%s", ser_msg);

          write(serial_port, ser_msg, sizeof(ser_msg));
      }



    }
    catch (const rs2::error & e)
    {
      std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
      return EXIT_FAILURE;
    }
    catch (const std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      return EXIT_FAILURE;
    }
    catch(...)
    {
      printf("An exception occurred in Serial_port_read1.cpp .\n");
      print_trace();
    }
  }

  close(serial_port);
  return 0; // success
}

void parse(char data[], uint8_t end)
{
  char value[20];
  memcpy(value, &data[11], end-10);
  value[end-10] = '\0';
  if(data[7] == 'r' && data[8] == 'd' && data[9] == 'o' && data[10] == ',')
  {
    old_radio_on = radio_on;

    radio_on = int(atof(value));;
    // uint8_t data = int(atof(value));

    // radio_on = data;
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
    x = abs(x);
    // printf("%d, ",x);
    int sum = 0;
    while (x != 0)
    {
        sum += x %10;
        x   = x /10;
    }
    // printf("%d\n", sum);
    return sum;


}