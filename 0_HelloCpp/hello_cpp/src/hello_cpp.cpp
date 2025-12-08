#include "hello_cpp.hpp"

#include <iostream>

HelloCpp::HelloCpp()
{
	// 생성자
    name_ = "GilDong";
}

HelloCpp::~HelloCpp()
{
	// 소멸자 
}

void HelloCpp::print_hello()
{
    std::cout << "Hello" << std::endl;
}

void HelloCpp::print_name()
{
	std::cout << "Name: " << name_ << std::endl;
}

std::string HelloCpp::get_name()
{
	return name_;
}

void HelloCpp::set_name(const std::string& name)
{
	name_ = name;
}