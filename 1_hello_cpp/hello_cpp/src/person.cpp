#include "person.hpp"

#include <string>
#include <vector>
#include <iostream>

Person::Person(const std::string& name, int age)
    : name_(name)
    , age_(age)
{}

Person::~Person() {}

void Person::print_info() {
	std::cout << "이름: " << name_ << ", 나이: " << age_ << "세" << std::endl;
}

void Person::print_hobbies() {
        std::cout << name_ << "의 취미: ";
        for (size_t i = 0; i < hobbies_.size(); ++i) {
            std::cout << hobbies_[i];
            if (i < hobbies_.size() - 1) std::cout << ", ";
        }
        std::cout << std::endl;
    }

void Person::add_hobby(const std::string& hobby) {
	hobbies_.push_back(hobby);
}