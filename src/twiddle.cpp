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
   double P = 0.04, I = .0001, D = 1.5;
   double change_tolerance = .0001;
   pid = PID();
   pid.Init(P, I, D);

   double p[3] = {P, I, D};
   double best_p[3] = {P, I, D};
   double dp[3] = {.1, .0001, .1};
   int p_index = 0;
   int max_N = 600;
   bool outer_check = false, inner_check = false, check_flag = false;

   double error = 0.0;
   double best_error = 1000.;
   int n = 0;

  h.onMessage([&change_tolerance, &check_flag, &outer_check, &inner_check, &p_index, &dp, &best_p, &p, &best_error, &max_N, &error, &P, &I, &D, &n, &pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          double steer_value;

          error += pow(cte, 2);

          if (n == 0) {
            pid.Init(P, I, D);
          }

           pid.UpdateError(cte);
           steer_value = pid.TotalError();
          
          // DEBUG
//          std::cout << "CTE: " << cte << " Steering Value: " << steer_value
//                    << std::endl;

          n += 1;
          if (n > max_N) {
            if (!outer_check) {
              std::cout << "Intermediate p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << std::endl;
              p[p_index] = dp[p_index];
              outer_check = true;
            } else {
              double cycleError = error / n;
              if (cycleError < best_error && !inner_check) {
                best_error = cycleError;
                best_p[0] = p[0];
                best_p[1] = p[1];
                best_p[2] = p[2];
                dp[p_index] *= 1.1;
                check_flag = true;

                std::cout << "best_error: " << best_error << std::endl;
                std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << " " << best_p[1] << " " << best_p[2] << std::endl;
              } else {
                if (!inner_check) {
                  p[p_index] -= 2 * dp[p_index];
                  inner_check = true;
                } else {
                  if (cycleError < best_error) {
                    best_error = cycleError;
                    best_p[0] = p[0];
                    best_p[1] = p[1];
                    best_p[2] = p[2];
//                    dp[p_index] = p[p_index];
                  } else {
                    p[p_index] += dp[p_index];
                    dp[p_index] *= .9;
                  }
                  check_flag = true;

                  std::cout << "error: " << error << std::endl;
                  std::cout << "best_error: " << best_error << std::endl;
                  std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << " " << best_p[1] << " " << best_p[2] << std::endl;
                }
              }
            }

            if (check_flag) {
              p_index += 1;
              outer_check = false;
              inner_check = false;
              check_flag = false;
            }

            if (p_index == 3) p_index = 0;
            error = 0.;
            n = 0;

            double _change_sum = 0;
            for (double _dp : dp) _change_sum += _dp;

            if (_change_sum < change_tolerance) {
              std::cout << "Best p: {" << best_p[0] << ", " << best_p[1] << ", " << best_p[2] << "} " << std::endl;
            } else {
              std::string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            }
          } else {
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
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