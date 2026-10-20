#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace vica_guidance
{

class PyError : public std::runtime_error
{
public:
  PyError(std::string type, const std::string & message)
  : std::runtime_error(message), type_(std::move(type)) {}
  const std::string & type() const {return type_;}

private:
  std::string type_;
};

std::string py_float(double x);

std::string py_str_repr(const std::string & utf8);

std::string py_list_repr(const std::vector<std::string> & items);
std::string py_list_repr(const std::vector<double> & items);
std::string py_int_list_repr(const std::vector<std::string> & decimal_items);

std::string py_slice(const std::string & utf8, std::size_t n);

std::string py_strip(const std::string & utf8);

std::string py_lower_for_lookup(const std::string & utf8);

int64_t py_int_saturating(double x);

std::string py_round_decimal(double x);

int64_t sec_to_ns(double seconds);

}
