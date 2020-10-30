#include <iostream>
#include <string>
#include <array>

#include <Eigen/Core>

#include <franka_typing_aid/examples_common.h>

struct test_struct
{
    int a;
    std::string b;
    std::array<double, 6> c;
};

int main()
{
    test_struct str;
    str.a = 1;
    str.b = "fdsf";
    str.c = {{4, 7.1, 6, 9.9, 2, 34}};

    Eigen::Vector3d test_vec(str.c.data());

    std::cout << test_vec << std::endl;
    std::cout << test_vec.norm() << std::endl;

    return 0;
}
