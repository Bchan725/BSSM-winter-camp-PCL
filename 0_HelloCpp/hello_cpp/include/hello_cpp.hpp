#ifndef HELLO_CPP_HPP
#define HELLO_CPP_HPP

#include <string>

class HelloCpp
{
public:
  	HelloCpp();
  	~HelloCpp();

public:
  	void print_hello();
	void print_name(); 

	std::string get_name();

	void set_name(const std::string& name);

private:
  	std::string name_;
};

#endif