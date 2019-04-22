#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /**
           * TODO: Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */
          // Waypoint data is transformed from the global to the
          // local vehicle perspective
          for (unsigned int i = 0; i < ptsx.size(); i++) {
            // Shift car reference angle to 90 degrees
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;
            
            // Rotate points to make psi zero
            ptsx[i] = (shift_x * cos(0 - psi) - shift_y*sin(0 - psi));
            ptsy[i] = (shift_x * sin(0 - psi) + shift_y*cos(0 - psi));
          }
          
          // Set position and orientation state vector variables to zero
          px = 0.0;
          py = 0.0;
          psi = 0.0;
          
          double* ptrx = &ptsx[0];
          Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);
          
          double* ptry = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);
          
          // Polynomial fit to transformed waypoints
          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);
            
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);
          //double epsi = psi - atan(coeffs[1] + 2*px*coeffs[2] + 3*coeffs[3]*pow(px,2));
          
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          // Define variables for a 100ms (expressed in sec)
          const double L_dt = 0.1;
          const double Lf = 2.67;
          
          // Implement a 100ms latency delay to the state variables
          px = v * cos(psi) * L_dt;
          py = v * sin(psi) * L_dt;
          psi = v * psi / Lf * L_dt;
          v = v + throttle_value * L_dt;
          cte = cte + v * sin(epsi) * L_dt;
          epsi = epsi + v * psi / Lf * L_dt;          
          
          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;
          
          auto vars = mpc.Solve(state, coeffs);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          double increment = 2.5;
          int num_points = 25;
          for(int i = 1; i < num_points; i++){
            next_x_vals.push_back(increment * i);
            next_y_vals.push_back(polyeval(coeffs, increment*i));
          }
          
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the 
          //   steering value back. Otherwise the values will be in between 
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -1.0 * steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for ( int i = 2; i < vars.size(); i++ ) {
            if ( i % 2 == 0 ) {
              mpc_x_vals.push_back( vars[i] );
            } else {
              mpc_y_vals.push_back( vars[i] );
            }
          }

          /**
           * TODO: add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Green line
           */

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          
          // Display the waypoints/reference line
          //vector<double> next_x_vals;
          //vector<double> next_y_vals;
/*
          double poly_inc = 2.5;
          int num_points = 25;
          
          for (int i = 1; i < num_points; i++) {
            next_x_vals.push_back(poly_inc * i);
            next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
          }
  */        

          // Display the waypoints/reference line
          //vector<double> next_x_vals;
          //vector<double> next_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Yellow line
           */
          //double Lf = 2.67;
          msgJson["steering_angle"] = vars[0]/(deg2rad(25)*Lf);
          msgJson["throttle"] = vars[1];
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
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