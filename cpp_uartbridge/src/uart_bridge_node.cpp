// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*

   Written by: Peer Poggendorf
  NODE DESCRIPTION:
  
  A Simple ROS2 Node that interfaces with a serial port device, publishing the read data to a ROS Topic. Writing to the serial Port is Done through a service.
  All relevant parameters are configurable ( PortName, Baudrate, TopicName, ServiceName)

  TOPIC DESCRIPTION:
  
  The Messages published to the topic are read from the serialport, cleaned up ( removal of carriage Return and Linebreaks) and then published to a topic of type "std_msgs/msg/string"

  SERVICE DESCRIPTION:

  The Service is used to write to the serial Port. The sent message is written to the bus, then an answer is expected from the Device. If no Answer is received within a time period, "OK" is returned to signal a successfull transmission.

  PARAMETER DESCRIPTION:
  ALL PARAMETERS NEED TO BE SET AT NODE CREATION!

  serialport - STRING - Name of the serial Port Device to be used ( eg. for Linux "/dev/ttyUSB0"), DEFAULT: "/dev/ttyUSB0"
  baudrate - int - Specifies the Baudrate of the Serial Port to be used (eg. 115200 ) DEFAULT: 115200
  topicName - STRING - Specifies the Name of the Topic the Received Messages will be published to. ( eg. "USBL_in"). DEFAULT: ${serialport}_in
  serviceName - STRING - Specifies the Name of the Service that is available to other ROS Nodes. ( eg. "USBL_service") DEFAUL: ${serialport}_service
  answerTimeout - int - Specifies the amount of time waited before the "OK" Timeout Signal is sent after calling the service. DEFAULT: 2000 ms

  TODO:
  [x] Implement Publisher
  [x] Implement Service
  [x] Implement Parameters
  [x] Lock Serial Device (handeld by library)
  [ ] Detect Lock to prevent doubling
  [x] Implement Serial Port deletion ( library does that)
  [ ] Implement Automatic Port Reopenng
  [x] Throw Readable Error, if the defined serial port is not there
  [x] Cleanup String
  [x] Timeout mit Parameter?

*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cstdio>
#include <serial/serial.h>
#include <chrono>
#include <thread>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "uartbridge_interfaces/srv/send_to_uart.hpp"

using namespace std::chrono_literals;
using namespace std;
using namespace std::this_thread; 
using namespace std::chrono;



class UartBridge : public rclcpp::Node
{
  public:
  UartBridge()
  : Node("UartBridge")
  {
    // enable Parameters
    this->declare_parameter("serialport", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("topicName", "/dev/ttyUSB0_in");
    this->declare_parameter("serviceName", "/dev/ttyUSB0_service");
    this->declare_parameter("answerTimeout", 2000);
    this->declare_parameter("pollRate", 1000);



    // read Parameters at startup
    port = this->get_parameter("serialport").get_parameter_value().get<string>();
    baud = this->get_parameter("baudrate").get_parameter_value().get<uint32_t>();
    topicName = this->get_parameter("topicName").get_parameter_value().get<string>();
    serviceName = this->get_parameter("serviceName").get_parameter_value().get<string>();
    answerTimeout = this->get_parameter("answerTimeout").get_parameter_value().get<int>();
    pollRate = this->get_parameter("pollRate").get_parameter_value().get<int>();


    if(topicName =="/dev/ttyUSB0_in" && port != "/dev/ttyUSB0")
    {
      topicName = port + "_in";
    }

    if(serviceName == "/dev/ttyUSB0_service" && port != "/dev/ttyUSB0")
    {
      serviceName = port + "_service";
    }
    int pollTimeout = 1000 / pollRate;
    // create the publisher, service and timer
    publisher_ = this->create_publisher<std_msgs::msg::String>(topicName, 10);
    timer_ = this->create_wall_timer(milliseconds(pollTimeout), std::bind(&UartBridge::timer_callback, this));

    service_ = this->create_service<uartbridge_interfaces::srv::SendToUART>(serviceName,bind(&UartBridge::service_callback,this, std::placeholders::_1,std::placeholders::_2));

    // initialize the serial Port Object
    openSerialPort();
 
  }//Constructor

  ~UartBridge()
  {
    
    delete serialPortObject;
  }//class destructor

  private:
  serial::Serial *serialPortObject; // creates a pointer to a object of the Serial class;
  string port;
  uint32_t baud;
  string topicName;
  string serviceName;
  int answerTimeout;
  int pollRate;

  void openSerialPort() // function that initializes the serial port
  {

    /*port = "/dev/ttyUSB0";
    baud = 9600;*/
    try {
    serialPortObject = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000)); // allocates memory on heap to a instance of serial and assign the memory adress to the pointer variable
    }
    catch (const serial::IOException& e)
    {
      cout << "No Serial Port of that Name exists!" <<endl;
      throw;
    }
    RCLCPP_INFO_STREAM(get_logger(), "Serial Port " << port << " opened with Baudrate " << baud); 
  }

  string cleanupMessage(string data)
  {
    data.erase(remove(data.begin(), data.end(), '\n'), data.cend());
    data.erase(remove(data.begin(), data.end(), '\r'), data.cend());
    return data;
  }
  string publishFromPort()
  {
    string data;
    // read Data
    serialPortObject->readline(data);
    cout << data << endl; // Debug
    // remove /n and /r
    data = cleanupMessage(data);
    // publish result
    auto message = std_msgs::msg::String();
    message.data = data;
    publisher_->publish(message);
    return data;
  }

  void timer_callback() // repeatetly called to read from the serial port
  {
    // testing only
    //string senddata = "test";
    //serialPortObject->write(senddata);
    if(! serialPortObject->isOpen())
    {
      RCLCPP_WARN(get_logger(), "Serial Port Closed");
      // reopen port
    }
    if(serialPortObject->available()>0)
    {
    // cout << "Bits " << to_string(serialPortObject->available()) << "available on Port " << port << endl; 
      // read message
      //string data = readFromPort();
      publishFromPort();
    }

  }//timer_callback()


  
  void service_callback(const shared_ptr<uartbridge_interfaces::srv::SendToUART::Request> request,
                        shared_ptr<uartbridge_interfaces::srv::SendToUART::Response> response)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Service called msg:" << request->message); 
    serialPortObject->write(request->message); // send into port
    string answer = "";
    auto begin = system_clock::now();
    while(system_clock::now() <= begin + milliseconds(answerTimeout))
    {
      if(serialPortObject->available()>0)
      {
        // read message
        //string data = readFromPort();
        string data = publishFromPort();
        answer = data;
        break;
      }
      else{
        answer = "";
        continue;
      }
    }

    if(answer != "")
    {
      response->response = answer;
    }    
    else {
      response->response = "OK";
    }
  }

  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<uartbridge_interfaces::srv::SendToUART>::SharedPtr service_;
}; //class UartBridge

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UartBridge>());
  rclcpp::shutdown();
  return 0;
}