#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <numeric>
#include <vector>

// Define a global client that can request services
ros::ServiceClient client;

std::pair<double, double> calculate_ball_centroid(const sensor_msgs::Image img)
{

  std::pair<double,double> ballCentroid;
  std::vector<int> xPix, yPix; 

  for(int pixIndex = 0 ; pixIndex < img.height*img.step; pixIndex++)
  {

    if(int(img.data[pixIndex]) == 255)
    {

      int y = pixIndex/img.height;
      int x = pixIndex % img.step;

      ROS_DEBUG("process_image(): Found white pixel at x = %d y = %d", x,y);
      xPix.push_back(x);
      yPix.push_back(y);
    }
  }

  
  ROS_WARN("process_image(): white cluster size %d", int(xPix.size()));
      
  if(((xPix.size() == 0) && (yPix.size() == 0)))
  {

    ballCentroid.first = -1;
    ballCentroid.second = -1;
    
  }
  else if((int(xPix.size()) > 17000)) //bug something went really wrong
  {
    ballCentroid.first = -1;
    ballCentroid.second = -1;
    

  }
  else
  {

    double xCentroid = static_cast<double>(std::accumulate(xPix.begin(), xPix.end(), 0))/static_cast<double>(xPix.size());;
    double yCentroid = static_cast<double>(std::accumulate(yPix.begin(), yPix.end(), 0))/static_cast<double>(yPix.size());;

    ballCentroid.first = xCentroid;
    ballCentroid.second = yCentroid;

  }


  return ballCentroid;


}


// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{

    ROS_INFO_STREAM("process_image(): Driving Bot to Location");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
    { 
      ROS_ERROR("process_image(): Failed to call service safe_move");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

  
    double leftMax = static_cast<double>(img.step)/3.0;
    double rightMin = 2*leftMax;
 
   
    std::pair<double,double> ballCentroid = calculate_ball_centroid(img);

    if((ballCentroid.first == -1)  &&  (ballCentroid.second == -1)) //stop because we can't find any ball
    {
      drive_robot(0,0);
    }
    else //ball in scene
    {
      if(ballCentroid.first < leftMax)
      {
        drive_robot(0.5,0.5);
      }
      else if(ballCentroid.first >= leftMax && ballCentroid.first < rightMin)
      {
        drive_robot(0.5,0.0);
      }
      else
      {
        drive_robot(0.5,-0.5);
      }
    }
    

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;



}
