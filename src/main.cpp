#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  //start in lane 1;
  int lane = 1;

  //Have a reference velocity to target
  double ref_vel = 0;//49.5/2.24; //mph converted to meters per second

  //set a boolean for lane change
  bool need_lane_change = false;

  //set up a counter for the lane changing duration
  int counter = 100;

  //set up reference vectors that store the s and d points of the previous vector
  vector<double> prev_s;
  vector<double> prev_d;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel,&need_lane_change,&counter,&prev_s,&prev_d](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	int prev_size = previous_path_x.size();

          	//number of points to push back
          	int points_to_push_back = 10;

          	//use the predictions for where my car will be during that same time
          	double future_car_s;
            double future_car_d;
            if(prev_size!=0){
                future_car_s = prev_s[50-prev_size+points_to_push_back-1];
                future_car_d = prev_d[50-prev_size+points_to_push_back-1];
            }
            else{
                future_car_s = car_s;
                future_car_d = car_d;
            }


          	// TODO: find the cars in front and decelerate if necessary
          	//create boolean signaling if no cars are within car's lane and position
            bool no_near_cars = true;
            //create booleans for left and right lanes free from traffic
            bool left_free = true;
            bool right_free = true;
            //set up current_lane to keep track when lane shift
            int current_lane = lane;
            //if the car is in the leftmost lane, left is not an option
            if(lane==0){
                left_free=false;
            }
            //if the car is in the rightmost lane, right is not an option
            if(lane==2){
                right_free=false;
            }
          	//for each car in the sensor fusion list
          	for(int i = 0;i<sensor_fusion.size(); i++){
                //observe if the car is in my lane
                double other_car_lane = sensor_fusion[i][6];
                double other_car_s = sensor_fusion[i][5];
                double other_car_vx = sensor_fusion[i][3];
                double other_car_vy = sensor_fusion[i][4];
                double other_car_v = sqrt(other_car_vx*other_car_vx+other_car_vy*other_car_vy);
                //now that the velocity is found, predict where the car will be in
                //points_to_push_back iterations
                other_car_s = other_car_s + other_car_v*.02*points_to_push_back;
                if(other_car_lane<(2+4*lane+2) && other_car_lane>(2+4*lane-2)){
                    //if the other car's s is greater than my vehicle's s and the distance
                    //between them is under 30m, begin slowing down and preparation
                    //for a lane change
                    if((other_car_s>future_car_s) && ((other_car_s-future_car_s)<25)){
                        //change the no_near_cars boolean
                        no_near_cars = false;
                        //signal need for lane change
                        need_lane_change = true;

                    }
                }
                //use the same sensor_fusion for loop to see if lane change is possible
                //to the left or right
                //if the distance between the two vehicle's s values less than 10,
                //and the other car's lane is to the left or right of the car's lane,
                //then do not switch lanes
                if(std::abs(other_car_s-future_car_s)<20){
                    //find which lane the other car is in
                    int other_car_lane_round = (int)other_car_lane/4;
                    if((other_car_lane_round-1)==lane){
                        //the lane is on the left. close the left lane change
                        left_free = false;
                    }
                    if((other_car_lane_round+1)==lane){
                        //the lane is on the right. close the right lane change
                        right_free = false;
                    }
                }
                //if the car in the other lane are going slower than the car in
                //the lane attempting to switch from, do not switch lanes
                if(((other_car_s-future_car_s)>=20) && ((other_car_s-future_car_s)<50)){
                    //find which lane the other car is in
                    int other_car_lane_round = (int)other_car_lane/4;
                    if(((other_car_lane_round-1)==lane)){// && (other_car_v<ref_vel)){
                        //the lane is on the left. close the left lane change
                        left_free = false;
                    }
                    if(((other_car_lane_round+1)==lane)){// && (other_car_v<ref_vel)){
                        //the lane is on the right. close the right lane change
                        right_free = false;
                    }
                }

          	}
          	std::cout<<"Other Car Lane: "<<other_car_lane<<std::endl;
          	//if the counter is less than 11, then the lane has not been constant
          	//for 10 iterations. do not change lanes if requested
          	if(counter<=40){
                need_lane_change = false;
          	}
          	if((need_lane_change) && (left_free)){
                //change lanes to the left. Set the reference velocity back to
                //its original and reset other variables
                lane = lane-1;
                no_near_cars = true;
                need_lane_change = false;
                counter = 0;
          	}
          	if((need_lane_change) && (right_free)){
                //change lanes to the right. Set the reference velocity back to
                //its original and reset other variables
                lane = lane+1;
                no_near_cars = true;
                need_lane_change = false;
                counter = 0;
          	}
          	//if there aren't any cars within your path and moving slower than 49.5mph
          	if((no_near_cars) && (ref_vel<(48.5/2.24))){
                //add onto the reference velocity
                ref_vel += .224/2.24; //mph converted to meters per second
          	}
          	//if there are cars within your path, decrease the velocity
          	else if(!no_near_cars){
                ref_vel -= .224/2.24;
          	}
          	counter += 1;
          	std::cout<<"Reference Velocity: "<<ref_vel<<endl;


          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
          	vector<double> next_s_vals;
          	vector<double> next_d_vals;
          	//create a list of widely spaced (s,d) waypoints, evenly spaced at 30m
          	//later we will interpolate these waypoints with a spline and fill it in with more points that control speed.
          	vector<double> ptss;
          	vector<double> ptsd;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	//push the last value into the vals if there was a previous path
          	//also set first_s and d values
          	double first_s;
          	double first_d;
          	if(prev_size>=2){
                //push back the last points_to_push_back points
                for(int i=0; i<points_to_push_back; i++){
                    next_x_vals.push_back(previous_path_x[i]);
                    next_y_vals.push_back(previous_path_y[i]);
                    next_s_vals.push_back(prev_s[50-prev_size+i]);
                    next_d_vals.push_back(prev_d[50-prev_size+i]);
                }
                double n_y = previous_path_y[0];
                double n_x = previous_path_x[0];
                //double theta = atan2(n_y-car_y,n_x-car_x);//deg2rad(car_yaw)
                vector<double> sd = getFrenet(n_x, n_y, deg2rad(car_yaw), map_waypoints_x, map_waypoints_y);
                first_s = prev_s[50-prev_size+points_to_push_back-1];
                first_d = prev_d[50-prev_size+points_to_push_back-1];
                ptss.push_back(first_s);
                ptsd.push_back(first_d);

          	}
          	else{
                //if it is the beginning, push back the first point using the ref_vel
                first_s = car_s + ref_vel*.02;
                first_d = car_d;
                vector<double> xy = getXY(first_s, first_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                next_x_vals.push_back(xy[0]);
                next_y_vals.push_back(xy[1]);
                ptss.push_back(first_s);
                ptsd.push_back(first_d);
          	}
          	//use s and d to find the new path values
          	//In Frenet, add evenly dist_inc spaced points ahead of the starting reference
          	double dist_inc = 20;
          	for(int i=0; i<3; i++){
                double next_s = first_s+(i+1)*dist_inc;
                double next_d = (2+4*lane);
                ptss.push_back(next_s);
                ptsd.push_back(next_d);
          	}
            //translate these points with respect to the first s and d
            for(int i=0; i<ptss.size(); i++){
                ptss[i] = ptss[i]-first_s;
                ptsd[i] = ptsd[i]-first_d;
            }
            //spline with respect to the points
            //create a spline
          	tk::spline s;
          	//set (x,y) points to the spline
          	s.set_points(ptss,ptsd);
          	//get size of points already in next vals
          	int next_vals_size = next_x_vals.size();
          	//generate the spline points
          	for(int i=0;i<(50-next_vals_size); i++){
                //obtain the spline s value
                double spline_s = (i+1)*.02*ref_vel;
                //obtain the spline y value
                double spline_d = s(spline_s);
                //un-transform the spline values
                spline_s += first_s;
                spline_d += first_d;
                next_s_vals.push_back(spline_s);
                next_d_vals.push_back(spline_d);
                //obtain the x and y values
                vector<double> xy = getXY(spline_s, spline_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                //add them to the next x and y values
                next_x_vals.push_back(xy[0]);
                next_y_vals.push_back(xy[1]);
          	}
          	//make the s and d of the current the previous for next time
          	prev_s = next_s_vals;
          	prev_d = next_d_vals;

          	/*
          	//create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          	//later we will interpolate these waypoints with a spline and fill it in with more points that control speed.
          	vector<double> ptsx;
          	vector<double> ptsy;

          	//reference x,y,yaw states
          	//either we will reference the starting point as where the car is or at the previous paths end point
          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);

          	//if the previous size is less than 2, use the car's current state
          	if(prev_size<2){
                //use two points that make the path tangent to the car
                double prev_car_x = car_x + (cos(car_yaw)+.001);
                double prev_car_y = car_y + (sin(car_yaw)+.001);

                //push the points into the ptsx vector
                ptsx.push_back(car_x);
                ptsx.push_back(prev_car_x);
                //push the points into the ptsy vector
                ptsy.push_back(car_y);
                ptsy.push_back(prev_car_y);
          	}
          	//else use the previous path's last two endpoints
          	else{
                //Redefine reference state as previous path end point
                //ref_x = previous_path_x[prev_size-1];
                //ref_y = previous_path_y[prev_size-1];
                //find the previous points before the reference point
                double ref_x_prev = previous_path_x[0];
                double ref_y_prev = previous_path_y[0];
                double ref_yaw_prev = atan2(ref_y_prev-ref_y,ref_x_prev-ref_x);

                double prev_car_x = ref_x_prev - cos(ref_yaw_prev);
                double prev_car_y = ref_y_prev - sin(ref_yaw_prev);

                //use two points that make the path tangent to the previous path's end point
                //push the points into the ptsx vector
                ptsx.push_back(ref_x);
                ptsx.push_back(ref_x_prev);
                //push the points into the ptsy vector
                ptsy.push_back(ref_y);
                ptsy.push_back(ref_y_prev);
          	}

          	//In Frenet, add evenly dist_inc spaced points ahead of the starting reference
          	double dist_inc = 20;
          	for(int i=0; i<3; i++){
                double next_s = car_s+(i+1)*dist_inc;
                double next_d = (2+4*lane);
                vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                std::cout<<" ["<<next_s<<","<<next_d<<","<<xy[0]<<","<<xy[1]<<"]"<<endl;

                ptsx.push_back(xy[0]);
                ptsy.push_back(xy[1]);
          	}

          	//transform the coordinates into the (ref_x,ref_y) space
          	for(int i=0;i<ptsx.size(); i++){
                //shift the points
                double shift_x = ptsx[i]-ref_x;
                double shift_y = ptsy[i]-ref_y;
                //change coordinates
                ptsx[i] = std::abs((shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw)));
                ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          	}

          	//create a spline
          	tk::spline s;

          	//output the vector ptsx
          	std::cout<<"Before spline: [";
          	for(int i=0;i<ptsx.size(); i++){
                std::cout<<ptsx[i]<<" , ";
          	}
          	std::cout<<" "<<endl;


          	//set (x,y) points to the spline
          	s.set_points(ptsx,ptsy);

          	//generate the spline points
          	for(int i=0;i<50; i++){
                //obtain the spline x value
                double spline_x = (i+1)*.02*ref_vel;
                //obtain the spline y value
                double spline_y = s(spline_x);
                //un-transform the spline values
                double x_point = spline_x;
                double y_point = spline_y;
                spline_x = (x_point*cos(ref_yaw-0)-y_point*sin(ref_yaw-0));
                spline_y = (x_point*sin(ref_yaw-0)+y_point*cos(ref_yaw-0));
                spline_x += ref_x;
                spline_y += ref_y;
                //add them to the next x and y values
                next_x_vals.push_back(spline_x);
                next_y_vals.push_back(spline_y);
          	}
          	*/
          	//output the vector next_x_vals
          	std::cout<<"[";
          	for(int i=0;i<next_x_vals.size(); i++){
                std::cout<<next_x_vals[i]<<" , ";
          	}
          	std::cout<<" "<<endl;
          	//output the vector next_y_vals
          	std::cout<<"[";
          	for(int i=0;i<next_y_vals.size(); i++){
                std::cout<<next_y_vals[i]<<" , ";
          	}
          	std::cout<<" "<<endl;


          	/*//std::cout<<ptsx<<endl;
          	//std::cout<<ptsy<<endl;
          	std::cout<<"Time "<<prev_size<<endl;
          	std::cout<<ptsx[0]<<endl;
          	std::cout<<ref_x<<endl;
          	std::cout<<ref_y<<endl;
          	*/


          	// END
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
