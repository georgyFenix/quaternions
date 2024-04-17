#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <cmath>
#include <array>
#include <numeric>
#include <iostream>
#include <algorithm>

template<typename T>
// Вспомогательная функция для вычисления скалярного произведения векторов
inline T dotProduct(const std::array<T, 3>& v1, const std::array<T, 3>& v2) {
    return std::inner_product(v1.begin(), v1.end(), v2.begin(), T{0});
}


template<typename T>
class Quaternion {
public:
    // Конструктор по умолчанию инициализирует кватернион как единичный
    Quaternion() : w(1), vect({0, 0, 0}) {}
    // Конструктор инициализирующий кватернион заданным вектором
    Quaternion(const std::array<T, 3>& vect) : w(0), vect(vect) {}

    // Конструктор, инициализирующий кватернион заданными значениями
    Quaternion(T w, const std::array<T, 3>& vect) : w(w), vect(vect) {}


    //Сопряженный кватернион
    Quaternion<T> conjugate() const{
        return Quaternion<T>(w, {-this->vect[0],-this->vect[1],-this->vect[2]});
    }

    //Геттеры
    T get_w() const{return this->w;}
    std::array<T, 3> get_vect() const{return this->vect;}



    // Метод для вычисления модуля (длины) кватерниона
    T norm() const {
        T sum = w * w;
        sum += dotProduct(vect, vect);
        return std::sqrt(sum);
    }
    
    // Метод для нормализации кватерниона
    Quaternion<T> normalize() const{
        T n = norm();
        return Quaternion<T>(w/n, {vect[0]/n, vect[1]/n, vect[2]/n});
            
    }
    //Инверсия параметров кватерниона
    Quaternion<T> inverseParam() const{
        return Quaternion<T>(-w, {-vect[0],-vect[1],-vect[2]});
    }
    //Обратный кватернион
    Quaternion<T> inverse() const{
        const T zn = 1 / (this->norm() * this->norm()); // Знаменатель считаем отдельно, чтобы была лучше точность
        return this->conjugate() * zn;
    };

    //скалярное произведение для углов
    T dotProductQuat(const Quaternion<T>& q) const{
      return this->w * q.w + dotProduct(this->vect, q.get_vect());
    }

    // Перегрузка оператора умножения кватернионов(не используем циклы, так как на таком количестве операций быстрее будет все развернуть)
    Quaternion<T> operator*(const Quaternion<T>& q) const {
        Quaternion<T> result;
        result.w = w * q.w - dotProduct(vect, q.vect);
        result.vect[0] = w * q.vect[0] + q.w * vect[0] + vect[1] * q.vect[2] - vect[2] * q.vect[1];
        result.vect[1] = w * q.vect[1] + q.w * vect[1] + vect[2] * q.vect[0] - vect[0] * q.vect[2];
        result.vect[2] = w * q.vect[2] + q.w * vect[2] + vect[0] * q.vect[1] - vect[1] * q.vect[0];

        return result;
    }

    // Перегрузка оператора сложения кватернионов
    Quaternion<T> operator+(const Quaternion<T>& q) const {
        return Quaternion<T>(w + q.w, {vect[0] + q.vect[0], vect[1] + q.vect[1], vect[2] + q.vect[2]});
    }

    // Перегрузка оператора вычитания кватернионов
    Quaternion<T> operator-(const Quaternion<T>& q) const {
        return Quaternion<T>(w - q.w, {vect[0] - q.vect[0], vect[1] - q.vect[1], vect[2] - q.vect[2]});
    }

    // Перегрузка оператора умножения кватерниона на скаляр
    Quaternion<T> operator*(T scalar) const {
        return Quaternion<T>(w * scalar, {vect[0] * scalar, vect[1] * scalar, vect[2] * scalar});
    }


    //Пегергрузка оператора присваивания одного кватерниона другому
    void operator=(const Quaternion<T>& q){
      this->w = q.w;
      std::copy(std::begin(q.vect), std::end(q.vect), std::begin(this->vect));
    }



    // Перегрузка оператора << для вывода кватерниона. Да простят меня боги за описание френд функции внутри класса, но иначе оно вообще не хочет компилиться, я 2 дня думал над этим ужасом и не понял, почему линковщик не видит её:(
    friend std::ostream& operator<<(std::ostream& os, const Quaternion<T>& q){    
      os << "(" << q.w << ", " << q.vect[0] << "i, " << q.vect[1] << "j, " << q.vect[2] << "k)";
      return os;
    };

private:
    T w;
    std::array<T, 3> vect;
};

//Вращение кватернионом
template <typename T>
std::array<T, 3> rotate(const std::array<T, 3>& vect3d, const std::array<T, 3>& axis, T angle) {
    Quaternion<T> q(cos(angle/2), {sin(angle/2)*axis[0], sin(angle/2)*axis[1], sin(angle/2)*axis[2]});
    return (q * vect3d*q.inverse()).get_vect();
}


//Метод SLERP (параметр клиренса отвечает за случаи, когда скалярное произведение будет считаться очень близким к 1) 
//ненормализованные кватернионы нормализуются
template <typename T>
Quaternion<T> SLERP(const Quaternion<T>& q1, const Quaternion<T>& q2, T t, T clearance= 0.999){
  Quaternion<T> q1n = q1.normalize();
  Quaternion<T> q2n = q2.normalize();
  Quaternion<T> q3;
  T dot = q1n.dotProductQuat(q2n);
  //проблемы с делением на 0
  if(dot > clearance){
    q3 = q1n + (q2n - q1n)*t;
    return q3.normalize();
  }
  else if(dot < 0){
    q1n = q1n.inverseParam();
    dot = -dot;
  }
  T angle = acos(dot);
  q3 = q1n*(std::sin((1-t)*angle)/std::sin(angle)) + q2n*(std::sin(t*angle)/std::sin(angle));
  return q3.normalize();
}




#endif // QUATERNION_HPP