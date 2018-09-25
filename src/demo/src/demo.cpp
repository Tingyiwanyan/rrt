/*
For class E599 Special Topics in Autonomous Robotics
ISE, Indiana University
Lantao Liu
9/21/2017
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <sstream>
#include<geometry_msgs/Pose.h>
#include<stdio.h>
#include<tf/tf.h>
#include<tf/transform_datatypes.h>
#include<iostream>
#include<ctime>
#include<cstdlib>
#include<math.h>
#include<vector>

struct Node
{
  double x;
  double y;
  double yaw;
  Node *parent;
  std::vector<Node*> child;
  Node(double x_,double y_, double yaw_)
  {
    x = x_;
    y = y_;
    yaw = yaw_;
  }
  Node(double x_,double y_, double yaw_, Node *parent_)
  {
    x = x_;
    y = y_;
    yaw = yaw_;
    parent = parent_;
  }
};

void insert_node(Node *parent, double pos_x,double pos_y,double yaw)
{
  std::cout<<"Im here in insert_node"<<std::endl;
  Node *node_new = new Node(pos_x,pos_y,yaw, parent);
  parent->child.push_back(node_new);
};

Node* check_nearset_node(Node *root,double pos_x,double pos_y)
{
  std::vector<Node*> queue;
  Node *nearest_node_link = root;
  double distance = std::sqrt(std::pow(root->x-pos_x,2)+std::pow(root->y-pos_y,2));
  queue.push_back(root);
  while(queue.size()!=0)
  {
    for(int i=0; i<queue[0]->child.size();i++)
    {
      queue.push_back(queue[0]->child[i]);
    }
    double distance_temp = std::sqrt(std::pow(queue[0]->x-pos_x,2)+std::pow(queue[0]->y-pos_y,2));
    if(distance_temp < distance)
    {
     distance = distance_temp;
     nearest_node_link = queue[0];
    }
    queue.erase(queue.begin());
  }
  return nearest_node_link;
  //std::cout<<"chile size is"<<root->child.size()<<std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_demo");

  //create a ros handle (pointer) so that you can call and use it
  ros::NodeHandle n;

  //in <>, it specified the type of the message to be published
  //in (), first param: topic name; second param: size of queued messages, at least 1 
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("some_chatter", 10);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  //each second, ros "spins" and draws 20 frames
  ros::Rate loop_rate(20);

  int frame_count = 0;
  float f = 0.0;
  Node* root = new Node(3, -3, 0,NULL);
  Node* temp_destination = root;
  static double step_size = 0.2;
  srand((unsigned)time(0));
  double dist_to_goal=100;
  int index_path = 0;
  std::vector<visualization_msgs::Marker*> obst2s;
  int id_index = 0;
  while (ros::ok())
  {
    if(dist_to_goal>0.2){
    //first create a string typed (std_msgs) message
    std_msgs::String msg;

    std::stringstream ss;
    ss << "Frame index: " << frame_count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str()); //printing on screen

    //publisher publishes messages, the msg type must be consistent with definition advertise<>(); 
    chatter_pub.publish(msg);

  /******************** From here, we are defining and drawing two obstacles in the workspace **************************/

    // define two obstacles
    static visualization_msgs::Marker obst1, obst2,obst3,obst4; 

    // Set obst1 and obst2 as a Cube and Cylinder, respectively
    obst1.type = visualization_msgs::Marker::CUBE;
    //obst2.type = visualization_msgs::Marker::CYLINDER;
    obst2.type = visualization_msgs::Marker::CUBE;
    obst3.type = visualization_msgs::Marker::CUBE;
    obst4.type = visualization_msgs::Marker::CUBE;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    obst1.header.frame_id = obst2.header.frame_id = obst3.header.frame_id=obst4.header.frame_id="map";  //NOTE: this should be "paired" to the frame_id entry in Rviz
    obst1.header.stamp = obst2.header.stamp = obst3.header.stamp = obst4.header.stamp = ros::Time::now();

    // Set the namespace and id 
    obst1.ns = obst2.ns = obst3.ns= obst4.ns = "obstacles";
    obst1.id = 0;
    obst2.id = 1;
    obst3.id = 2;
    obst4.id = 3;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    obst1.action = obst2.action = obst3.action = obst4.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker 
    obst1.scale.x = 6.0;
    obst1.scale.y = 0.5;
    obst1.scale.z = 0.0; //1x1x1 here means each side of the cube is 1m long
    obst3.scale.x = 6.0;
    obst3.scale.y = 0.5;
    obst3.scale.z = 0.0;
    obst4.scale.x = 6.0;
    obst4.scale.y = 0.5;
    obst4.scale.z = 0.0;
    obst2.scale.x = 0.3;
    obst2.scale.y = 0.1;
    obst2.scale.z = 0.1; //1x1x1 here means the cylinder as diameter 1m and height 1m

    // Set the pose of the marker. since a side of the obstacle obst1 is 1m as defined above, now we place the obst1 center at (1, 2, 0.5). z-axis is height
    obst1.pose.position.x = 2;
    obst1.pose.position.y = 2;
    obst1.pose.position.z = 0.0;
    obst3.pose.position.x = -2;
    obst3.pose.position.y = 0;
    obst3.pose.position.z = 0.0;
    obst4.pose.position.x = 2;
    obst4.pose.position.y = -2;
    obst4.pose.position.z = 0.0;
    
    
    tf::Quaternion q_orig, q_rot, q_new,q_vehicle;
    double r1=3.14159;
    double yaw = 0, yaw_vehicle = 0;
    q_rot = tf::createQuaternionFromRPY(0,0,yaw);
    q_vehicle = tf::createQuaternionFromRPY(0,0,yaw_vehicle);
    q_rot.normalize();
    q_vehicle.normalize();
  //tf::Quaternion q(2,3,4,1);
  //tf::Matrix3x3 m(q);
    tf::Matrix3x3 m1(q_rot);
    tf::Matrix3x3 m2(q_vehicle);
  //double roll,pitch,yaw,roll1,pitch1,yaw1;
  //m.getRPY(roll,pitch,yaw);
  //m1.getRPY(roll1,pitch1,yaw1);
  obst1.pose.orientation.x = q_rot.x();
  obst1.pose.orientation.y = q_rot.y();
  obst1.pose.orientation.z = q_rot.z();
  obst1.pose.orientation.w = q_rot.w();
  obst3.pose.orientation.x = q_rot.x();
  obst3.pose.orientation.y = q_rot.y();
  obst3.pose.orientation.z = q_rot.z();
  obst3.pose.orientation.w = q_rot.w();
  obst4.pose.orientation.x = q_rot.x();
  obst4.pose.orientation.y = q_rot.y();
  obst4.pose.orientation.z = q_rot.z();
  obst4.pose.orientation.w = q_rot.w();
  obst2.pose.orientation.x = q_vehicle.x();
  obst2.pose.orientation.y = q_vehicle.y();
  obst2.pose.orientation.z = q_vehicle.z();
  obst2.pose.orientation.w = q_vehicle.w(); 
    //obst1.pose.orientation.x = 0.0;
    //obst1.pose.orientation.y = 0.0;
    //obst1.pose.orientation.z = 0.0;
    //obst1.pose.orientation.w = 1.0;	//(x, y, z, w) is a quaternion, ignore it here (quaternion can be converted to angle and converted back, ros can do it)

    obst2.pose.position.x = 3;
    obst2.pose.position.y = -3;
    obst2.pose.position.z = 0.0;
    //obst2.pose.orientation = obst1.pose.orientation;

    // Set the color red, green, blue. if not set, by default the value is 0
    obst1.color.r = 0.0f; 
    obst1.color.g = 1.0f;
    obst1.color.b = 0.0f;
    obst1.color.a = 1.0;		//be sure to set alpha to something non-zero, otherwise it is transparent
    obst2.color.r = 1.0f;
    obst2.color.g = 1.0f;
    obst2.color.b = 0.0f;
    obst2.color.a = 1.0;
    obst3.color.r = 0.0f; 
    obst3.color.g = 1.0f;
    obst3.color.b = 0.0f;
    obst3.color.a = 1.0;
    obst4.color.r = 0.0f; 
    obst4.color.g = 1.0f;
    obst4.color.b = 0.0f;
    obst4.color.a = 1.0;

    obst1.lifetime = obst2.lifetime = obst3.lifetime = obst4.lifetime = ros::Duration();
    double diam_veh = std::sqrt(std::pow(obst2.scale.x/2.0,2)+std::pow(obst2.scale.y/2.0,2));

    // publish these messages to ROS system
    marker_pub.publish(obst1);
    marker_pub.publish(obst2);
    marker_pub.publish(obst3);
    marker_pub.publish(obst4);
    static visualization_msgs::Marker rob;
    static visualization_msgs::Marker path;
    rob.type = visualization_msgs::Marker::SPHERE;
    path.type = visualization_msgs::Marker::LINE_STRIP;

    rob.header.frame_id = path.header.frame_id = "map";  //NOTE: this should be "paired" to the frame_id entry in Rviz, the default setting in Rviz is "map"
    rob.header.stamp = path.header.stamp = ros::Time::now();
    rob.ns = path.ns = "rob";
    rob.id = 0;
    path.id = 1;
    rob.action = path.action = visualization_msgs::Marker::ADD;
    rob.lifetime = path.lifetime = ros::Duration();

    rob.scale.x = rob.scale.y = 0.2;
    rob.scale.z = 0.1; 

    rob.color.r = 1.0f;
    rob.color.g = 0.5f;
    rob.color.b = 0.5f;
    rob.color.a = 1.0; 

    // path line strip is blue
    path.color.b = 1.0;
    path.color.a = 1.0;

    path.scale.x = 0.02; 
    path.pose.orientation.w = 1.0;
   
    rob.pose.position.x = 4.0;
    rob.pose.position.y = 4.0;
    rob.pose.position.z = 0.0;

    int num_slice2 = 200;		// divide a circle into segments
    static int slice_index2 = 0;

  /************************* From here, we are using points, lines, to draw a tree structure *** ******************/

    //we use static here since we want to incrementally add contents in these mesgs, otherwise contents in these msgs will be cleaned in every ros spin.
   
    static visualization_msgs::Marker vertices;
    static visualization_msgs::Marker edges;
    

    vertices.type = visualization_msgs::Marker::POINTS;
    edges.type = visualization_msgs::Marker::LINE_LIST;

    vertices.header.frame_id = edges.header.frame_id = "map";
    vertices.header.stamp = edges.header.stamp = ros::Time::now();
    vertices.ns = edges.ns = "vertices_and_lines";
    vertices.action = edges.action = visualization_msgs::Marker::ADD;
    vertices.pose.orientation.w = edges.pose.orientation.w = 1.0;

    vertices.id = 0;
    edges.id = 1;

    // POINTS markers use x and y scale for width/height respectively
    vertices.scale.x = 0.05;
    vertices.scale.y = 0.05;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    edges.scale.x = 0.02; //tune it yourself

    // Points are green
    vertices.color.g = 1.0f;
    vertices.color.a = 1.0;

    // Line list is red
    edges.color.r = 1.0;
    edges.color.a = 1.0;

    geometry_msgs::Point p0;	// root vertex
    geometry_msgs::Point p1;
    //srand((unsigned)time(0));
    double random_integer_x;
    double random_integer_y;
    random_integer_x = ((rand()%100)+1.0-50.0)/10.0;
    random_integer_y = ((rand()%100)+1.0-50.0)/10.0;
    //p0.x = random_integer_x;
    //p0.y = random_integer_y;
    p0.z = 0;
    Node *temp = check_nearset_node(root,random_integer_x,random_integer_y);
    temp_destination = temp;
    p0.x = temp->x;
    p0.y = temp->y;
    double vector_x = random_integer_x - temp->x;
    double vector_y = random_integer_y - temp->y;
    double norm_ = std::sqrt(vector_x*vector_x+vector_y*vector_y);
    vector_x = (vector_x/norm_)*step_size;
    vector_y = (vector_y/norm_)*step_size;
    double new_pos_x = temp->x + vector_x;
    double new_pos_y = temp->y + vector_y;
    double dist_obj_veh_x_1 = std::abs(obst1.pose.position.x-new_pos_x);
    double dist_obj_veh_y_1 = std::abs(obst1.pose.position.y-new_pos_y);
    double dist_obj_veh_x_3 = std::abs(obst3.pose.position.x-new_pos_x);
    double dist_obj_veh_y_3 = std::abs(obst3.pose.position.y-new_pos_y);
    double dist_obj_veh_x_4 = std::abs(obst4.pose.position.x-new_pos_x);
    double dist_obj_veh_y_4 = std::abs(obst4.pose.position.y-new_pos_y);
    if((dist_obj_veh_x_1 > diam_veh+obst1.scale.x/2.0 || dist_obj_veh_y_1 > diam_veh+obst1.scale.y/2.0) &&(dist_obj_veh_x_3 > diam_veh+obst3.scale.x/2.0 || dist_obj_veh_y_3 > diam_veh+obst3.scale.y/2.0) &&(dist_obj_veh_x_4 > diam_veh+obst4.scale.x/2.0 || dist_obj_veh_y_4 > diam_veh+obst4.scale.y/2.0)  )
    {
      double norm_ = std::sqrt(vector_x*vector_x+vector_y*vector_y);
      double cos_ = vector_x/norm_;
      double yaw_new = acos(cos_);
    p1.x = new_pos_x;
    p1.y = new_pos_y;
    p1.z = 0;
    insert_node(temp, new_pos_x,new_pos_y,yaw_new);
    
    std::cout<<"random_x is"<<random_integer_x<<std::endl;
    std::cout<<"random_y is"<<random_integer_y<<std::endl;
    std::cout<<"node_x is"<<temp->x<<std::endl;
    std::cout<<"node_y is"<<temp->y<<std::endl;
    std::cout<<"new_x is"<<p1.x<<std::endl;
    std::cout<<"new_y is"<<p1.y<<std::endl;
    int num_slice = 20;		// e.g., to create 20 edges 
    float length = 1;		//length of each edge
    
    edges.points.push_back(p0);
    edges.points.push_back(p1);
    static int slice_index = 0;
    int herz = 10;		//every 10 ROS frames we draw an edge
    //publish msgs
    vertices.points.push_back(p1);
    marker_pub.publish(vertices);
    marker_pub.publish(edges);
    dist_to_goal = std::sqrt(std::pow(new_pos_x-rob.pose.position.x,2)+std::pow(new_pos_y-rob.pose.position.y,2));
    std::cout<<"dist_to_goal"<<dist_to_goal<<std::endl;
    }


  /******************** From here, we are defining and drawing a simple robot **************************/

    // a simple sphere represents a robot
    
/*
    if(frame_count % 2 == 0 && path.points.size() <= num_slice2)  //update every 2 ROS frames
    {
      geometry_msgs::Point p;

      float angle = slice_index2*2*M_PI/num_slice2;
      slice_index2 ++ ;
      p.x = 4 * cos(angle) - 0.5;  	//some random circular trajectory, with radius 4, and offset (-0.5, 1, .05)
      p.y = 4 * sin(angle) + 1.0;
      p.z = 0.05;
      
      rob.pose.position = p;
      path.points.push_back(p);		//for drawing path, which is line strip type
    }
*/
    //path.points.push_back(p1);

    marker_pub.publish(rob);
    //marker_pub.publish(path);


  /******************** To here, we finished displaying our components **************************/
 
    // check if there is a subscriber. Here our subscriber will be Rviz
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please run Rviz in another terminal.");
      sleep(1);
    }
    //++frame_count;

}
else
{
      std::vector<Node*> move_sequence;
      Node *trace_back = temp_destination->child[temp_destination->child.size()-1];
      move_sequence.push_back(trace_back);
      while(trace_back->parent != NULL)
      {
       trace_back = trace_back->parent;
       move_sequence.push_back(trace_back);
      }
      //index_path = move_sequence.size();
      std::cout<<"move_sequence_size"<<move_sequence.size()<<std::endl;
      //for(int j = 0;j<move_sequence.size();j++)
      //{
//	std::cout<<"move_sequnce "<<j<<std::endl;
//	std::cout<<"x "<<move_sequence[j]->x<<std::endl<<"y "<<move_sequence[j]->y<<std::endl<<"yaw "<<move_sequence[j]->yaw<<std::endl;
  //    }
     std::cout<<"index is "<<index_path<<std::endl;
      //return 0;

if(frame_count % 2 == 0 && index_path < move_sequence.size())  //update every 2 ROS frames
    {
/*
      static visualization_msgs::Marker path;
    path.type = visualization_msgs::Marker::LINE_STRIP;
     path.header.frame_id = "map";
     path.header.stamp = ros::Time::now();
     path.ns = "rob";
     path.id = 1;
     path.action = visualization_msgs::Marker::ADD;
     path.lifetime = ros::Duration();
     path.color.b = 1.0;
    path.color.a = 1.0;
    path.scale.x = 0.02; 
    path.pose.orientation.w = 1.0;
*/
      static visualization_msgs::Marker *obst2 = new visualization_msgs::Marker;
      obst2->type = visualization_msgs::Marker::CUBE;
      obst2->header.frame_id = "map";
      obst2->ns = "obstacles1";
      obst2->id = id_index;
      obst2->action = visualization_msgs::Marker::ADD;
      obst2->pose.position.x = move_sequence[move_sequence.size()-1-index_path]->x;
      obst2->pose.position.y = move_sequence[move_sequence.size()-1-index_path]->y;
      obst2->pose.position.z = 0.0;
      obst2->scale.x = 0.3;
    obst2->scale.y = 0.1;
    obst2->scale.z = 0.1;
      obst2->header.stamp = ros::Time::now();
      obst2->lifetime = ros::Duration();
      obst2->color.r = 1.0f;
    obst2->color.g = 1.0f;
    obst2->color.b = 0.0f;
    obst2->color.a = 1.0;
      tf::Quaternion q_vehicle_path;
      q_vehicle_path = tf::createQuaternionFromRPY(0,0,move_sequence[move_sequence.size()-1-index_path]->yaw);
      tf::Matrix3x3 m3(q_vehicle_path);
      obst2->pose.orientation.x = q_vehicle_path.x();
      obst2->pose.orientation.y = q_vehicle_path.y();
      obst2->pose.orientation.z = q_vehicle_path.z();
      obst2->pose.orientation.w = q_vehicle_path.w();
      index_path = index_path + 1;
      //marker_pub.publish(obst2);
      obst2s.push_back(obst2);
      id_index = id_index + 1;
      std::cout<<"obst2s size is "<<obst2s.size()<<std::endl;
      for(int k=0;k<obst2s.size();k++)
      {
	
        marker_pub.publish(*obst2s[k]);
      }
    } 
 
if(index_path == move_sequence.size()-1)
{
return 0;
}   
    
}
    //ros spins, force ROS frame to refresh/update once
    ros::spinOnce();

    loop_rate.sleep();
    ++frame_count;
  }

  return 0;
}


