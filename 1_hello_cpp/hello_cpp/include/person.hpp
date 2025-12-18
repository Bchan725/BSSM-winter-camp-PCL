#pragma once

#include <string>
#include <vector>

class Person
{
public:
    Person(const std::string& name, int age);
    ~Person();

public:
    void print_info();
    void print_hobbies();

    void add_hobby(const std::string &hobby);

private:
    std::string name_;
    int age_;
    std::vector<std::string> hobbies_;
};
