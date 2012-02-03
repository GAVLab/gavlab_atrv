/*!
 * \file atrv/atrv.h
 * \author William Woodall <wjwwood@gmail.com>
 * \version 0.1
 *
 * \section LICENSE
 *
 * The BSD License
 *
 * Copyright (c) 2011 William Woodall
 *
 * Permission is hereby granted, free of charge, to any person obtaining a 
 * copy of this software and associated documentation files (the "Software"), 
 * to deal in the Software without restriction, including without limitation 
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides an interface to the ATRV in the gavlab.
 * 
 * This library depends on CMake-2.4.6 or later: http://www.cmake.org/
 * This library depends on MDC2250: https://github.com/GAVLab/mdc2250
 * 
 */

#ifndef ATRV_H
#define ATRV_H

// Standard Library Headers
#include <string>
#include <sstream>
#include <vector>
#include <map>

// Boost Headers
#include <boost/function.hpp>

// MDC2250 Headers
#include "mdc2250/mdc2250.h"
#include "mdc2250/decode.h"

namespace atrv {

/*!
 * Exception called when a connection to the atrv fails for some reason.
 */
class ConnectionFailedException : public std::exception {
  const std::string e_what_;
  int error_type_;
public:
  ConnectionFailedException(const std::string &e_what = "",
                            int error_type = -1)
  : e_what_(e_what), error_type_(error_type) {}
  ~ConnectionFailedException() throw() {}

  int error_type() {return error_type_;}

  virtual const char * what() const throw() {
    std::stringstream ss;
    ss << "Failed to connect: " << this->e_what_;
    return ss.str().c_str();
  }
};

/*!
 * Exception called when an exception occurs in the motor controller.
 */
class ATRVException : public std::exception {
  const std::string e_what_;
  size_t mc_index_;
  int error_type_;
public:
  ATRVException(const std::string &e_what = "",
                size_t mc_index = -1,
                int error_type = -1)
  : e_what_(e_what), mc_index_(mc_index), error_type_(error_type) {}
  ~ATRVException() throw() {}

  int error_type() {return error_type_;}

  virtual const char * what() const throw() {
    std::stringstream ss;
    ss << "Exception with the ";
    if (this->mc_index_ == 1) {
      ss << "front motor controller";
    } else if (this->mc_index_ == 2) {
      ss << "rear motor controller";
    } else {
      ss << "atrv";
    }
    ss << ": " << this->e_what_;
    return ss.str().c_str();
  }
};

/*!
 * This function type describes the prototype for the telemetry callback.
 * 
 * The function takes a motor controller index, 1 for front and 2 for rear is 
 * the convention but it matches to ATRV::connect parameters port1 and port2 
 * respectively, a mdc2250::queries::QueryType which describes the type of 
 * data in the telemetry (see mdc2250/decode.h), a std::vector of long's 
 * containing the telemetry data, and returns nothing.  It is 
 * called when new data from the telemetry system is received.  For more 
 * information about the query data refer to the mdc2250 user manual starting 
 * at page 99.
 * 
 * \see ATRV::setTelemetryCallback
 */
typedef boost::function<void(size_t mc_index,
                             const mdc2250::queries::QueryType &query_type,
                             std::vector<long> &telemetry)>
TelemetryCallback;

/*!
 * This function type describes the prototype for the logging callbacks.
 * 
 * The function takes a std::string reference and returns nothing.  It is 
 * called from the library when a logging message occurs.  This 
 * allows the library user to hook into this and integrate it with their own 
 * logging system.  It can be set with any of the set<log level>Handler 
 * functions.
 * 
 * \see ATRV::setInfoHandler
 */
typedef boost::function<void(const std::string&)> LoggingCallback;

/*!
 * This function type describes the prototype for the exception callback.
 * 
 * The function takes a std::exception reference and returns nothing.  It is 
 * called from the library when an exception occurs in a library thread.
 * This exposes these exceptions to the user so they can to error handling.
 * 
 * \see ATRV::setExceptionHandler
 */
typedef boost::function<void(const std::exception&)> ExceptionCallback;

/*!
 * Represents an MDC2250 Device and provides and interface to it.
 */
class ATRV {
public:
  /*!
   * Constructs the ATRV object.
   */
  ATRV();
  virtual ~ATRV();

  /*!
   * Connects to the ATRV motor controller given a serial port.
   * 
   * \param port1 Defines the port for the first motor controller.
   * Examples: Linux - "/dev/ttyS0" Windows - "COM1"
   * \param port2 Defines the port for the second motor controller.
   * Examples: Linux - "/dev/ttyS1" Windows - "COM2"
   * \params watchdog size_t time in milliseconds for the watchdog timeout 
   * period.  Defaults to 1000 ms.
   * \params echo bool true enables echoing on the motor controllers.  
   * Defaults to true
   * 
   * \throws ConnectionFailedException connection attempt failed.
   */
  void connect(std::string port1, std::string port2,
               size_t watchdog = 1000, bool echo = true);

  /*!
   * Disconnects from the ATRV.
   */
  void disconnect();

  /*!
   * Moves the ATRV.
   * 
   * \params linear_velocity ssize_t value for linear velocity in m/s
   * 
   * \params angular_velocity ssize_t value for angular velocity in rad/s
   */
  void move(ssize_t linear_velocity=0, ssize_t angular_velocity=0);

  /*!
   * Sets the function to be called when new telemetry is available.
   * 
   * \param telemetry_callback A function pointer to the callback to handle
   * new telemetry.
   * 
   * \see atrv::TelemetryCallback, ATRV::setInfoHandler
   */
  void
  setTelemetryCallback (TelemetryCallback telemetry_callback,
                        mdc2250::queries::QueryType query_type =
                        mdc2250::queries::any_query);

  /*!
   * Sets the function to be called when an info logging message occurs.
   * 
   * This allows you to hook into the message reporting of the library and use
   * your own logging facilities.
   * 
   * The provided function must follow this prototype:
   * <pre>
   *    void yourInfoCallback(const std::string &msg)
   * </pre>
   * Here is an example:
   * <pre>
   *    void yourInfoCallback(const std::string &msg) {
   *        std::cout << "MDC2250 Info: " << msg << std::endl;
   *    }
   * </pre>
   * And the resulting call to make it the callback:
   * <pre>
   *    Object my_object;
   *    my_object.setInfoCallback(yourInfoCallback);
   * </pre>
   * Alternatively you can use a class method as a callback using boost::bind:
   * <pre>
   *    #include <boost/bind.hpp>
   *    
   *    #include "object.h"
   *    
   *    class MyClass
   *    {
   *    public:
   *     MyClass () {
   *      my_object.setInfoHandler(
   *          boost::bind(&MyClass::handleInfo, this, _1));
   *     }
   *    
   *     void handleInfo(const std::string &msg) {
   *       std::cout << "MyClass Info: " << msg << std::endl;
   *     }
   *    
   *    private:
   *     Object object;
   *    };
   * </pre>
   * 
   * \param info_handler A function pointer to the callback to handle new 
   * Info messages.
   * 
   * \see serial::LoggingCallback
   */
  void setInfoHandler(LoggingCallback info_handler) {
    this->info = info_handler;
  }

  /*!
   * Sets the function to be called when an exception occurs internally.
   * 
   * This allows you to hook into the exceptions that occur in threads inside
   * the library.
   * 
   * \param exception_handler A function pointer to the callback to handle new 
   * interal exceptions.
   * 
   * \see mdc2250::ExceptionCallback, MDC2250::setInfoHandler
   */
  void
  setExceptionHandler (ExceptionCallback exception_handler) {
    this->handle_exc = exception_handler;
  }

  // Vehicle geometry
  double track_width_, wheel_radius_;
  size_t max_rpm_, encoder_ppr_;

private:
  // Exception callback handle
  ExceptionCallback handle_exc;
  LoggingCallback info;

  // Motor controllers
  mdc2250::MDC2250 front_mc_, rear_mc_;
  // Concurrent connecting variables
  std::string front_mc_error_, rear_mc_error_;
  void connect_(size_t mc_index, const std::string &port,
                size_t wd, bool echo);
  void disconnect_(size_t mc_index);

  // Telemetry variables
  void parse_telemetry_(size_t motor_index, const std::string &msg);
  std::map<mdc2250::queries::QueryType, TelemetryCallback> telemetry_cb_map_;

  // MDC2250 logging and error handling
  void info_cb_(const std::string &msg, size_t mc_index);
  void exc_cb_(const std::exception &error, size_t mc_index);

  // Motor controll
  ssize_t left_wheel_effort_, right_wheel_effort_;

  // Move thread safety
  boost::mutex move_mux;

};

}

#endif
