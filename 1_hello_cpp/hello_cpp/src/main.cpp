#include "person.hpp"

#include <iostream>
#include <vector>

void space() {
    std::cout << std::endl;
}

int main()
{
    // 1. 객체 생성
    Person student1("홍길동", 20);

    // 2. 정보 출력
    student1.print_info();

    // 3. 취미 추가
    student1.add_hobby("축구");
    student1.add_hobby("독서");

    // 4. 취미 출력
    student1.print_hobbies();

    space();

    // 5. Vector로 여러 학생 관리
    std::vector<Person> students;

    students.push_back(Person("르끌레르", 28));
    students.push_back(Person("알론소", 44));

    std::cout << "\n전체 학생 수: " << students.size() << "명" << std::endl;

    for (auto& student : students) {
        student.print_info();
        student.print_hobbies();
    }

    return 0;
}