//compile with g++ examples.cpp -o examples
#include <iostream>
#include "math.h"
#include "quaternion.hpp"




int main(){
  //SLERP пример
  Quaternion<double> q1(1, {0.6, 0.8, 0.0});
  Quaternion<double> q2(2, {-0.8, 0.0, -0.6});
  std::cout<<"SLERP for normalized " << q1 << " and " << q2 << "for t=0.1 is: " << SLERP(q1,q2, 0.1) <<std::endl;
  //пример поворота вектора на pi с помощью кватернионов
  float angle = M_PI;
  std::array<float, 3> u{1, 1, 1};
  std::array<float, 3> ax{1, 0, 0};
  std::array<float, 3> rot = rotate(u, ax, angle);
  std::cout <<"Rotating vector"<< u[0] <<"x "<< u[1] <<"y " << u[2] << "z"  << std::endl;
  std::cout <<"to " << rot[0] <<"x "<< rot[1] <<"y " << rot[2] << "z"  << std::endl;
  //Сложение, произведение кватернионов, нормализация и умножение на скаляр, обратный кватернион
  Quaternion<double> q3(1.1, {1.0, 2.9, 3.1});
  Quaternion<double> q4(1.1, {-2.9, 3.1, -1.0});
  std::cout << "q3 and q4 are " << q3 << " " << q4 << std::endl;
  std::cout << "addition of q3 and q4 is: " << q3 + q4 << std::endl;
  std::cout << "multiplication of q3 and q4 is: " << q3 * q4 << std::endl;
  std::cout << "normalized q3 is: " << q3.normalize() << std::endl;
  std::cout << "multiplication of q4 and scalar is: " << q4 * 2.2 << std::endl;
  std::cout << "inverse q3 is: " << q3.inverse() << std::endl;

  return 0;
}