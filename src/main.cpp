#include <uWS/uWS.h>
#include <iostream>
#include <math.h>
#include "json.hpp"
#include "PID.h"
#include "vehicle.h"
#include "twiddle.h"

// for convenience
using json = nlohmann::json;

// TWIDDLE
double best_err = std::numeric_limits<double>::max();

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }         
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// Restarts the simulator through the web socket protocol
bool skip = false;
void reset(uWS::WebSocket<uWS::SERVER>& ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
  skip = true;
}

double min_speed = 25.0, max_speed = 50.0;
int main() {
  uWS::Hub h;

  Vehicle vehicle;

/*
  double p[] = { 0.15, 0.0001, 2.0 };
  double dp[] = { 0.1, 0.0001, 1.0 };

  Twiddle twiddle(vehicle.steering_pid);
  twiddle.Init(p, dp);

  Twiddle twiddle(vehicle.throttle_pid);
  twiddle.Init(p, dp);
*/

  // h.onMessage([&vehicle, &twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
  h.onMessage([&vehicle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2' && !skip)
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double steering_angle = std::stod(j[1]["steering_angle"].get<std::string>());

          double set_speed = abs(abs(steering_angle / 25.0) - 1.0) * max_speed;
          if (set_speed < min_speed) { set_speed = min_speed; }
          vehicle.SetSpeed(set_speed);
          
          vehicle.Update(speed, steering_angle, cte);
          // bool did_reset = twiddle.Update(cte, std::bind(reset, ws));
          bool did_reset = false;

          if (!did_reset) {
            json msgJson;
            msgJson["steering_angle"] = vehicle.set_steering;
            msgJson["throttle"] = vehicle.throttle;
            std::string msg = "42[\"steer\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          else {
            vehicle.reset();
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    skip = false;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
