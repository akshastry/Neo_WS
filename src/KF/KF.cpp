/ C++ headers
#include <cmath>
#include <stdio.h>
#include <stdlib.h>

#include <execinfo.h>

// ROS headers
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/UInt8.h"
#include "tf/tf.h"
// #include "std_msgs/Float64.h"

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

int main(int argc, char * argv[]) 
try
{
  // // Main loop
  while (ros::ok())
  {
    
    ros::spinOnce();
  }
      
  return EXIT_SUCCESS;
  }

catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch(...)
{
    printf("An exception occurred in pose.cpp .\n");
    print_trace();
}