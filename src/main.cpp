#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "vehicle.h"
#include <limits>
#include <math.h>

// for convenience
using json = nlohmann::json;

// TWIDDLE
double best_err = std::numeric_limits<double>::max();

const int skip = 100;
const int n = 500;
int it = 0;
int cur_coef = 0;
double cur_err = 0;
double tol = 0.0001;
bool is_second_pass = false;

double p[3] = { 0.644128, 0.000729, 2.893 };
double dp[3] = { 0.1, 0.001, 1.0 };


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
void reset(uWS::WebSocket<uWS::SERVER>& ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

// variable used to set whether or not to optimize the pid.
bool should_tune = false;

void twiddle_init(PID& pid) {
  it = 0;
  cur_err = 0;
  is_second_pass = false;
  pid.Init(p[0], p[1], p[2]);

  std::cout << "PID Initialized: ( "
            << p[0] << ", "
            << p[1] << ", "
            << p[2] << " )\t"
            << cur_coef << "\n";
}

void twiddle(PID& pid, double cte, std::function<void()> reset) {
  if (++it >= n+skip) {
    double err = cur_err / n;

    // handle first pass
    if (best_err == std::numeric_limits<double>::max()) {
      best_err = err;
      p[cur_coef] += dp[cur_coef];
      twiddle_init(pid);
      std::cout << "FIRST\t" << err << "/" << best_err << "\n";
    }
    // found error is < best_err
    else if (err < best_err) {
      best_err = err;
      dp[cur_coef] *= 1.1;
      
      if (++cur_coef >= 3) {
        cur_coef = 0;
      }

      if (cur_coef == 0 && dp[0]+dp[1]+dp[2] < tol) {
        should_tune = false;
        std::cout << "PID Coefficients Found: ("
                  << p[0] << ","
                  << p[1] << ","
                  << p[2] << ")\n";
        return;
      }
      else {
        p[cur_coef] += dp[cur_coef];
        twiddle_init(pid);
        std::cout << "BEST\t" << err << "/" << best_err << "\n";
      }
    }
    // err > best_err and not second pass, set p and re-run
    else if (!is_second_pass) {
      p[cur_coef] -= 2 * dp[cur_coef];
      twiddle_init(pid);
      is_second_pass = true;
      std::cout << "FAIL -> FIRST PASS\t" << err << "/" << best_err <<  "\n";
    }
    else {
      p[cur_coef] += dp[cur_coef];
      dp[cur_coef] *= 0.9;
      std::cout << "FAIL -> SECOND PASS\t" << err << "/" << best_err << "\n";

      if (++cur_coef >= 3) {
        cur_coef = 0;
      }

      if (cur_coef == 0 && dp[0]+dp[1]+dp[2] < tol) {
        should_tune = false;
        std::cout << "PID Coefficients Found: ("
                  << p[0] << ","
                  << p[1] << ","
                  << p[2] << ")\n";
        return;
      }

      p[cur_coef] += dp[cur_coef];
      twiddle_init(pid); 
    }

    reset();
  }
  // found error is > best_err
  else {
    if (it >= skip) {
      cur_err += (cte*cte);
    }
  }
}

int main() {
  uWS::Hub h;

  Vehicle vehicle;

  h.onMessage([&vehicle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double steering_angle = std::stod(j[1]["steering_angle"].get<std::string>());
          
          vehicle.Update(speed, steering_angle, cte);

          json msgJson;
          msgJson["steering_angle"] = vehicle.set_steering;
          msgJson["throttle"] = vehicle.throttle;
          std::string msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          // std::cout << msg << std::endl;
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
