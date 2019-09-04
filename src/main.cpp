#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

std::vector<double> dp = {0.01, 0.001, 0.1};
std::vector<double> p = {0.2, 0.0, 5};
std::vector<double> best_p = {0.0, 0.0, 0.0};

// dpp, dpi, dpd
int twiddle_dp = 0;

// kp=0, ki=1, kd=2
int twiddle_stage = 0;

// limit parameters
// sum < tol
double tol = 0.0001;

int num_iteration = 800;
double best_err = 1e9;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  // best pid 0.1995, 0.0009, 4.5125
  pid.Init(0.1995, 0.0009, 4.5125);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          // double speed = std::stod(j[1]["speed"].get<string>());
          // double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          // // twiddle
          // if (std::accumulate(dp.begin(),dp.end(),0.0) > tol){
          //   // for i in range(len(p)):
          //   if (twiddle_stage > 2){
          //     twiddle_stage = 0;
          //   }
          //   // 20ms fresh? 2.0s, after 400 steps err pow increase
          //   if (num_iteration < 800){
          //     pid.UpdateError(cte);
          //     num_iteration++;
          //   }
          //   else{
          //     double Errors = pid.totalError();
          //     std::cout << "P  " << p[0] << "\t" << p[1] << "\t" << p[2] << std::endl;
          //     std::cout << "DP  " << dp[0] << "\t" << dp[1] << "\t" << dp[2] << std::endl;
          //     switch (twiddle_stage)
          //     {
          //     case 0:
          //       p[twiddle_dp] += dp[twiddle_dp];
          //       // stay and let the simulator run 2s and get error
          //       // ready to move to next p
          //       twiddle_stage = 1;
          //       num_iteration = 0;
          //       break;
          //     case 1:
          //       if (Errors < best_err){
          //         best_err = Errors;
          //         // increase the dp[i]
          //         dp[twiddle_dp] *= 1.05;
          //         // return to the previous p
          //         twiddle_stage = 0;
          //         // check next dp
          //         twiddle_dp++;
          //         for (int i=0; i<3; i++){
          //           best_p[i] = p[i];
          //         }
          //       }
          //       else
          //       { 
          //         // else check downwards gain
          //         p[twiddle_dp] -= 2*dp[twiddle_dp];
          //         // go next stage
          //         twiddle_stage = 2;
          //         // simulate another 2 seconds
          //         num_iteration = 0;
          //       }
          //       break;

          //     case 2:
          //       if (Errors < best_err){
          //         best_err = Errors;
          //         dp[twiddle_dp] *= 1.05;
          //         for (int i=0; i<3; i++){
          //           best_p[i] = p[i];
          //         }
          //       }
          //       else
          //       {
          //         // if both faild return to original
          //         p[twiddle_dp] += dp[twiddle_dp];
          //         dp[twiddle_dp] *= 0.95;
          //       }
          //       twiddle_stage = 0;
          //       // next dp
          //       twiddle_dp++;
          //       break;
          //     }
          //     pid.Init(p[0], p[1], p[2]);
          //     std::cout << p[0] << "\t" << p[1] << "\t" << p[2] << std::endl;
          //     std::cout << "best pid" << "\t" << best_p[0] << "\t" << best_p[1] << "\t" << best_p[2] << std::endl;
          //   }
          // }
          // else {
          //   std::cout << "find best pid parameters!" << std::endl;
          //   std::cout << p[0] << "\t" << p[1] << "\t" << p[2] << std::endl;
          // }

          pid.UpdateError(cte);
          // restrict steer_value
          steer_value = pid.limitSteering(pid.steer_value);
          
          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
          //           << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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