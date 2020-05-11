#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using Eigen::MatrixXd;


// start lane = 1
int lane = 1; // ||0|1|2

// have a reference speed
double ref_vel = 0; //mph vehicle speed
double v_max = 49.5; // mph maximum speed
int    iteration = 100; // wait 2 seconds before changing the lane 
int timer = 0; // count the iterations


// grid
vector<double> target_s = {-5,0,10,20,30,40};
vector<int> target_lane = {0,1,2};
int detect_range = 2; // get ready to change lane when the other car in my lane is 30 meter or less

double final_s = 30; // final distance in s
double final_lane = 1; // final lane

double car_prev_speed = 0;//mph used for speed PD controller

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
    
            // Sensor Fusion Data, a list of all other cars on the same side 
            // of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];
            
            int prev_size = previous_path_x.size();
            
            if(prev_size>0){
                car_s = end_path_s;
            }
            
            bool too_close = false;
            double tf_speed = 49.6; // the other car speed
            double tf_dist  = 30;
            double min_cost = 10000;
            double current_cost = 10000;
            MatrixXd A = MatrixXd(target_lane.size(),target_s.size());

            float car_lane = (car_d-2)/4.0; // lane of other cars
            car_lane = std::round(car_lane);
            // initial cost matrix
            for (int j = 0; j<target_lane.size(); ++j)
            {
                for (int k = 0; k<target_s.size(); ++k)
                {
                    double delta_d = sqrt(pow(j-car_lane,2));
                    A(j,k) = int(delta_d)+target_s[k]+10;
                }
            }
            
            //find ref_vel to use
            //std::cout<< "\nI am driving on lane " <<lane<<"\n";
            for (int i = 0; i < sensor_fusion.size(); ++i)
            {
                //std::cout<< "\ninput " <<sensor_fusion[i]<<"\n";
                double x  = sensor_fusion[i][1];
                double y  = sensor_fusion[i][2];
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                double check_car_s = sensor_fusion[i][5];
                double check_car_d = sensor_fusion[i][6];
                
                // prodect the car s location in next step
                check_car_s += ((double)prev_size*.02*check_speed);
                
                // check each lane and distance
                for (int j = 0; j<target_lane.size(); ++j)
                {
                    for (int k = 0; k<target_s.size(); ++k)
                    {
                        double current_d = 2 + 4*j;
                        double current_s = car_s + target_s[k]; // the distace between car and matrix cell
                        
                        vector<double> xy = getXY(current_s,current_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
                        double dist = distance(x,y,xy[0],xy[1]);

                        double dist_s = check_car_s - current_s; // the distance in s between sensor_fusion car vs matrix cell
                        double dist_d = sqrt(pow(check_car_d - current_d,2)); // abs of the distance in d between sensor_fusion car vs matrix cell
                        if (dist_s<6 && dist_s>-5 && dist_d < 2) // find vehicle in range of 9.8 meter before or after matirx cell
                        {
                            A(j,k) = 99; // set matrix cell to 99
                            if (j==lane && k<target_s.size()-detect_range ) // if see a car in 20 meter range than slow down
                            {
                                too_close = true;
                                // std::cout<< "debug  1 "<<j<< " too_close ------------- "<<too_close << std::endl;
                                tf_speed = check_speed;
                                tf_dist = target_s[k];
                            }
                        }
                        if (k>=1 && A(j,k-1) == 99) // check the car behind you
                        {
                            A(j,k) = 99;
                            
                        }
                        if (k>1 && A(j,k) == 99 && A(j,k-1) != 99) // 
                        {
                            A(j,k-1) += 3; // when following a car slowly this can make the care change lane
                        }
                    }
                }
            }
            int temp_lane = lane; // temp_lane is current lane
            double temp_s    = 30;
            int counter = 0;
            for (int j = 0; j<target_lane.size(); ++j)
            {
                double temp = target_s.size()-detect_range; // change lane when other car is in 30 meter range
                for (int k = temp; k<target_s.size(); ++k)
                {
                    current_cost = A(j,k);
                    if (current_cost<min_cost) // find the minimou cost 
                    {   
                        min_cost = current_cost;

                        int delta_lane = j-lane;
                        if ((delta_lane > 1.5 || delta_lane < -1.5)) // avoid double lane change
                        {
                            if ( A(1,2) != 99) // center lane doesn't have car in 10 meter ahead of me 
                            {
                                temp_lane += delta_lane/2; // 2,0 =>1  0,2 => 1
                            }
                            else
                            {
                                temp_lane = lane; // doesn't change lane;
                                too_close = true;
                                std::cout<< "debug  2 "<<j<< " too_close ------------- "<<too_close << std::endl;
                            }
                        }
                        else if (min_cost == 99)
                        {
                            temp_lane = lane; // don't change lane if there is no good lane
                        }
                        else
                        {
                            temp_lane = j;
                        }
                        temp_s = target_s[k]; 
                    }
                }
            }

            if (too_close) // recuce speed is too close
            {
                double delta_v = ref_vel-tf_speed; // delta proportional
                double delta_v_pre = delta_v-car_prev_speed; // delta derivative
                // PD controllers avoid 0 and proportional to 1/delta_s
                ref_vel = ref_vel - 0.0035 * 30/(tf_dist+1) * delta_v - 0.001 * delta_v_pre/((double)prev_size*.02); 
                car_prev_speed = ref_vel;
                if (ref_vel>v_max)
                {
                    ref_vel = v_max; // if speed over max speed, set to max
                }
            }
            else if (ref_vel<v_max && ref_vel >20)
            {
                //ref_vel += .224; // if speed too slow just incress the speed
                double delta_v = ref_vel-v_max; // delta proportional
                double delta_v_pre = delta_v-car_prev_speed; // delta derivative
                // PD controllers avoid 0 and proportional to 1/delta_s
                ref_vel = ref_vel - 0.0035 * 30/(tf_dist+1) * delta_v - 0.001 * delta_v_pre/((double)prev_size*.02); 
                car_prev_speed = ref_vel;
                if (ref_vel>v_max)
                {
                    ref_vel = v_max; // if speed over max speed, set to max
                }
            }
            else if (ref_vel <20)
            {
                ref_vel += .224;
            }
            
            else
            {
                ref_vel = v_max; // if speed over max speed, set to max
            }
            
            if (temp_lane != lane) // double-check before change the lane
            {
                timer += 1;
                if(timer == iteration) // 2 seconds
                {
                    int delta_lane = temp_lane-lane;
                    if (delta_lane > 1.5 || delta_lane < -1.5)
                    {
                        lane += temp_lane/2;
                        timer -= 1;
                    }
                    else
                    {
                        lane = temp_lane;
                    }
                    
                    std::cout<< "\n++++++++++++++\n" << "timer reach " <<timer<<std::endl;
                    std::cout<< "Change lane to " << lane<<std::endl;
                    timer = 0; // reset timer
                }
            }
            else
            {
                 timer = 0;
            }
            
            final_s = temp_s; // distance my car need to go  
            
            //std::cout<< "------------- "<<lane << " " <<tf_dist <<" " << too_close<<" " << timer << "\n" << A<<std::endl;
            
            // create a lis of widely spaced (x,y) waypoint, evenly ditributed over 30 meters
            // later use interoplate
            
            vector<double> ptsx;
            vector<double> ptsy;
            
            // reference x,y,yaw
            // next time step's starting point is current location
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            
            // if previous size is almost empty, use the care as starting ref_vel
            if (prev_size < 2)
            {
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);
                
                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);
                
                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            }
            else
            {
               ref_x = previous_path_x[prev_size-1];
               ref_y = previous_path_y[prev_size-1];
               
               double ref_x_prev = previous_path_x[prev_size-2];
               double ref_y_prev = previous_path_y[prev_size-2];
               ref_yaw = atan2(ref_y - ref_y_prev,ref_x - ref_x_prev);
               
               ptsx.push_back(ref_x_prev);
               ptsx.push_back(ref_x);
               
               ptsy.push_back(ref_y_prev);
               ptsy.push_back(ref_y);
            }
            
            
            // spline
            vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            
            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);
            
            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);
            
            for(int i=0; i < ptsx.size(); ++i)
            {
                //shift car ref system to 0 degree
                double shift_x = ptsx[i]-ref_x;
                double shift_y = ptsy[i]-ref_y;
                
                ptsx[i] = (shift_x *cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
                ptsy[i] = (shift_x *sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
            }
            
            // create a spline
            tk::spline s;
            
            // set (x,y);
            s.set_points(ptsx,ptsy);
            
            // define the splie;
            
    
            vector<double> next_x_vals;
            vector<double> next_y_vals;
    
            /**
            * TODO: define a path made up of (x,y) points that the car will visit
            *   sequentially every .02 seconds
            */
            
            // push the previous value first
            for (int  i =0; i<previous_path_x.size(); ++i)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
                
            }
            
            double target_x = final_s;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
            
            double x_add_on = 0; // start at the origin, because we used the shift
            
            double N = (target_dist/(.02*ref_vel/2.24)); // mph to m/s

            // add more points
            for (int i = 1; i <= 50-previous_path_x.size(); ++i)
            {
                double x_point = x_add_on+target_x/N;
                double y_point = s(x_point);
                
                x_add_on = x_point;
                
                double x_ref = x_point;
                double y_ref = y_point;
                
                // rotate back
                x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
                y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
                
                x_point += ref_x;
                y_point += ref_y;
                
                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
                
            }
    
            json msgJson;
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;
    
            auto msg = "42[\"control\","+ msgJson.dump()+"]";
    
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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