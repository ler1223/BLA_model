#include <iostream>
#include <cmath>
#include "raylib.h"


class MyVector {
    public:
    double i, j, k;
    MyVector(double i, double j, double k): i(i), j(j), k(k){};
    MyVector(Vector3 v): i(v.x), j(v.y), k(v.z){};

    MyVector operator+(const MyVector &v1){
        return MyVector(i + v1.i, j + v1.j, k + v1.k);
    }

    friend double MultiplyScalar(const MyVector &v1, const MyVector &v2){
        return v1.i*v2.i + v1.j*v2.j + v1.k*v2.k;
    }

    MyVector operator*(const double &number){
        return MyVector(i*number, j*number, k*number);
    }

    MyVector operator*(MyVector v){
        return MyVector(i*v.i, j*v.j, k*v.k);
    }

    friend std::ostream& operator<<(std::ostream &os, MyVector v){
        os << v.i << " " << v.j << " " << v.k;
        return os;
    }

    MyVector absolut(){
        return MyVector(abs(i), abs(j), abs(k));
    }

    double lenght2(){
        return i*i + j*j + k*k;
    }

    Vector3 toVect(){
        return Vector3{(float)i, (float)j, (float)k};
    }

    friend MyVector MultiplyVector(const MyVector &v1, const MyVector &v2){
        double ni = v1.j * v2.k - v1.k * v2.j;
        double nj = -(v1.i * v2.k - v1.k * v2.i);
        double nk = v1.i * v2.j - v1.j * v2.i;
        return MyVector(ni, nj, nk);
    }

    double lenght(){
        return sqrt(i*i + j*j + k*k);
    }

    MyVector invert(){
        return MyVector(-i, -j, -k);
    }
};

class MyQuaternion {
    public:
    double h;
    MyVector v;
    MyQuaternion(double h, MyVector v): h(h), v(v){};

    friend std::ostream& operator<<(std::ostream &os, MyQuaternion &q){
        os << q.h << " " << q.v << std::endl;
        return os;
    }

    MyQuaternion operator+(MyQuaternion &q1){
        return MyQuaternion(h + q1.h, v + q1.v);
    }

    MyQuaternion operator*(double &number){
        return MyQuaternion(h*number, v*number);
    }

    MyQuaternion operator*(MyQuaternion &q1){
        return MyQuaternion(h*q1.h - MultiplyScalar(v, q1.v), q1.v*h + v*q1.h + MultiplyVector(v, q1.v));
    }

    double length(){
        return sqrt(h*h + v.lenght2());
    }

    MyQuaternion normalize(){
        double n = 1 / this->length();
        return MyQuaternion(h, v)*n;
    }

    MyQuaternion invert(){
        return MyQuaternion(h, v.invert());
    }
};