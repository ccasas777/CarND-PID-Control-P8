#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include "json.hpp"
#include "PID.h"
#include "twiddle.h"


#define M_PI 3.14159
// for convenience
using nlohmann::json;
using std::string;
using namespace std;
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

  PID pid,pid_speed;
  Twiddle twiddle;

  ofstream fout;
  fout.open("twiddle.txt");
  if (fout.fail()) { std::cout << "faile to save data" << std::endl; }
  else {
      fout << "Here is the twiddle data\n";
      fout.close();
  }

  /**
   * TODO: Initialize the pid variable.
   */

  //p = {Kp, Ki, Kd}
  //vector<double> p = {0.25, 0.0022, 6.0}; 
  vector<double> p = { 0.08, 0.003, 3.0 };
  vector<double> p_2 = {0.18, 0.007, 0.0};
  pid.Init(p);
  pid_speed.Init(p_2); 
  twiddle.init(p,0.2);
  double prev_steer_value = 0;
  double total_cte = 0;
  double total_angle = 0;
  int count_step = 0;
  h.onMessage([&total_angle, &count_step ,&total_cte, &fout, &prev_steer_value,&pid_speed ,&p, &pid, &twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value = 0;
          double throttle = 0;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          
          /*
          twiddle.SaveCTE(cte);
          

          if (twiddle.updateParams()) {
              p = twiddle.GetParams();
              std::cout << "Kp: " << p[0] << ", Ki: " << p[1] << ", Kd: " << p[2] << std::endl;
              pid.UpdateParms(p);

              fout.open("twiddle.txt", ios::app);
              if (fout.fail()) {
                  std::cout << "fail to write data" << std::endl;
              }
              else {
                  
                  fout << p[0]<<", " << p[1] << ", " << p[2] << "\n";           
                  std::cout << "save the data!" << std::endl;
                  fout.close();
                  
              }

          }
          */
           
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          //std::cout << cte << std::endl;
          //std::cout << "###################" << std::endl;
          //std::cout << "cal_steer_value: " << steer_value << std::endl;
          double diff_steer = steer_value - prev_steer_value;
          //std::cout << "diff_steer : " << diff_steer << std::endl;
          double limit_of_steer = 0.8;
          double limit_of_steer_gradient = 0.1;
          /*
          if (prev_steer_value > 0 && diff_steer > limit_of_steer_gradient) {
              steer_value = prev_steer_value + limit_of_steer_gradient;
          }
          else if (prev_steer_value > 0 && diff_steer < -limit_of_steer_gradient) {
              steer_value = prev_steer_value - limit_of_steer_gradient;
          }
          else if (prev_steer_value < 0 && diff_steer > limit_of_steer_gradient) {
              steer_value = prev_steer_value + limit_of_steer_gradient;

          }
          else if (prev_steer_value < 0 && diff_steer < -limit_of_steer_gradient) {
              steer_value = prev_steer_value - limit_of_steer_gradient;
          }
          std::cout << "out_steer_value: " << steer_value << std::endl;
          */
          //limit maximum and minmum
          if (steer_value > limit_of_steer) {
              steer_value = limit_of_steer;
          }
          else if (steer_value < -limit_of_steer) {
              steer_value = -limit_of_steer;
          }

          prev_steer_value = steer_value;


          pid_speed.UpdateError(cte);
          throttle = 0.6 + pid_speed.TotalError_speed();
          
          if (count_step % 2500 > 1500) {
              p = { 0.13, 0.002, 3.0 };
              throttle = 0.3;
              pid.UpdateParms(p);
          }
         
         //Monitor the performance
         count_step++;
         total_cte += pow(cte,2);
         total_angle += pow(angle, 2);
         if (count_step % 500 == 0) {
             std::cout <<"total_cte: " <<total_cte << std::endl;
             std::cout << "total_angle: " <<total_angle << std::endl;
         }
        
         
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
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