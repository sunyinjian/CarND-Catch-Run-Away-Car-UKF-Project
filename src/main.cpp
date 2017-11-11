#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("]");
    if (found_null != std::string::npos) {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

/**@brief normalize angle between +pi and -pi;
 *
 * @param [OUT]: angle
 */
void NormalizeAngle(double& angle){
    while (angle > M_PI) angle-=2.*M_PI;
    while (angle <-M_PI) angle+=2.*M_PI;
}

int main()
{
    uWS::Hub h;

    // Create a UKF instance
    UKF ukf;

    double target_x = 0.0;
    double target_y = 0.0;
    double target_v = 0.0;

    h.onMessage([&ukf,&target_x,&target_y, &target_v](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
        auto s = hasData(std::string(data));
        if (s != "") {
            auto j = json::parse(s);
            std::string event = j[0].get<std::string>();

            if (event == "telemetry") {
                // j[1] is the data JSON object
                double hunter_x = std::stod(j[1]["hunter_x"].get<std::string>());
                double hunter_y = std::stod(j[1]["hunter_y"].get<std::string>());
                double hunter_heading = std::stod(j[1]["hunter_heading"].get<std::string>());

                string lidar_measurment = j[1]["lidar_measurement"];

                MeasurementPackage meas_package_L;
                istringstream iss_L(lidar_measurment);
                long long timestamp_L;

                // reads first element from the current line
                string sensor_type_L;
                iss_L >> sensor_type_L;

                // read measurements at this timestamp
                meas_package_L.sensor_type_ = MeasurementPackage::LASER;
                meas_package_L.raw_measurements_ = VectorXd(2);
                float px;
                float py;
                iss_L >> px;
                iss_L >> py;
                meas_package_L.raw_measurements_ << px, py;
                iss_L >> timestamp_L;
                meas_package_L.timestamp_ = timestamp_L;

                ukf.ProcessMeasurement(meas_package_L);

                string radar_measurment = j[1]["radar_measurement"];

                MeasurementPackage meas_package_R;
                istringstream iss_R(radar_measurment);
                long long timestamp_R;

                // reads first element from the current line
                string sensor_type_R;
                iss_R >> sensor_type_R;

                // read measurements at this timestamp
                meas_package_R.sensor_type_ = MeasurementPackage::RADAR;
                meas_package_R.raw_measurements_ = VectorXd(3);
                float ro;
                float theta;
                float ro_dot;
                iss_R >> ro;
                iss_R >> theta;
                iss_R >> ro_dot;
                meas_package_R.raw_measurements_ << ro,theta, ro_dot;
                iss_R >> timestamp_R;
                meas_package_R.timestamp_ = timestamp_R;

                ukf.ProcessMeasurement(meas_package_R);

                target_x = ukf.x_[0];
                target_y = ukf.x_[1];
                target_v = ukf.x_[2];

                double heading_to_target = atan2(target_y - hunter_y, target_x - hunter_x);
                NormalizeAngle(heading_to_target);
                //turn towards the target
                double heading_difference = heading_to_target - hunter_heading;
                NormalizeAngle(heading_difference);

                double dist_hunter_to_target = 0.0; // distance from current hunter to current target
                double dist_hunter_to_predTarget = 0.0; // distance from current hunter to predicted target in future;
                double dist_predHunter_to_predTarget = 0.0; // distance from predicted hunter to predicted target in future;

                for (double delta_t = 0.1; delta_t < 3.0; delta_t += 0.05){
                    //predict the target state in the future sequence
                    ukf.PredictStateInFuture(delta_t);
                    //caculate distance from current hunter to predicted target in future sequence;
                    dist_hunter_to_predTarget = sqrt((ukf.x_pred_[1] - hunter_y)*(ukf.x_pred_[1] - hunter_y) + (ukf.x_pred_[0] - hunter_x)*(ukf.x_pred_[0] - hunter_x));
                    //caculate distance from predicted hunter to predicted target in future sequence;
                    dist_predHunter_to_predTarget = dist_hunter_to_predTarget - delta_t * target_v;

                    if (dist_predHunter_to_predTarget < 0.0){
                        //calculate the heading to the predicted target;
                        heading_to_target = atan2(ukf.x_pred_[1] - hunter_y, ukf.x_pred_[0] - hunter_x);
                        NormalizeAngle(heading_to_target);
                        //turn towards the predicted target;
                        heading_difference = heading_to_target - hunter_heading;
                        NormalizeAngle(heading_difference);
                        break;
                    }
                }

                // smooth turning of hunter
                heading_difference *= 0.1;
                dist_hunter_to_target = sqrt((target_y - hunter_y)*(target_y - hunter_y) + (target_x - hunter_x)*(target_x - hunter_x));

                json msgJson;
                msgJson["turn"] = heading_difference;
                msgJson["dist"] = dist_hunter_to_target;
                auto msg = "42[\"move_hunter\"," + msgJson.dump() + "]";
                //std::cout << msg << std::endl;
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























































































