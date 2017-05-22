#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include "twiddle.h"

//#define TWIDDLE_ENABLE

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Twiddle from .2, .004, 2 = > .2108, .00679, 2.328 at .35 throttle
// Twiddle from .2108, .00679, 2.328 = > .179, .00779, 2.428 at .5 throttle
double Kp = .179;
double Ki = .00779;
double Kd = 2.428;

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

int main()
{
  uWS::Hub h;

  PID pid;
  
  twiddle tw;
  
  pid.Init(Kp,Ki,Kd);
  
  tw.set_PID_coeffs(Kp,Ki,Kd);

  h.onMessage([&pid,&tw](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          
          //limit steering values to +/- 1
          if(steer_value>1){steer_value=1;}
          if(steer_value<-1){steer_value=-1;}
          
          //slow down as max_CTE increases
          double max_CTE = 2.5;
          double max_throttle = .5;
          double throttle = max_throttle*(max_CTE-fabs(cte))/max_CTE;
          //Can't have negative throttle, also the car would not move if 0 throttle.
          if(throttle<0.1) throttle = 0.1;
          
#ifdef TWIDDLE_ENABLE
          tw.updateError(cte);          
          if(tw.get_update_count()>1200)
          {
            std::vector<double> newParams= tw.twiddle_PID_coeffs();
            pid.Init(newParams[0],newParams[1],newParams[2]);
            
            std::cout << "\n\n\n\n\nNEW PID PARAMS: P:"<<newParams[0]<<" I:"<<newParams[1]<<" D:"<<newParams[2]<<"\n\n\n\n";
          }
#endif
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
