#pragma once
#include <iostream>
#include <exception>

class sensorexception: public std::exception
{
  virtual const char* what() const throw()
  {
    return "Exception: Sensor Not Connected";
  }
} sensorex;

class configexception: public std::exception
{
  virtual const char* what() const throw()
  {
    return "Exception: Configuration Failed";
  }
} configex;