#include "hello_cpp.hpp"

#include <iostream>

int main()
{
    HelloCpp hello;

    hello.print_hello();
    hello.print_name();

    std::string name = hello.get_name();
    std::cout << "Get Name: " << name << std::endl;

    hello.set_name("HongGildong");
    hello.print_name();

    return 0;
}