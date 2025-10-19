#include <iostream>
#include <cmath>

class MyVector {
    public:
    double i, j, k;
    MyVector(double i, double j, double k): i(i), j(j), k(k){};

    MyVector operator+(const MyVector &v1){
        return MyVector(i + v1.i, j + v1.j, k + v1.k);
    }

    friend double MultiplyScalar(const MyVector &v1, const MyVector &v2){
        return v1.i*v2.i + v1.j*v2.j + v1.k*v2.k;
    }

    MyVector operator*(const double &number){
        return MyVector(i*number, j*number, k*number);
    }

    friend std::ostream& operator<<(std::ostream &os, MyVector v){
        os << v.i << " " << v.j << " " << v.k;
        return os;
    }

    double lenght2(){
        return i*i + j*j + k*k;
    }

    friend MyVector MultiplyVector(const MyVector &v1, const MyVector &v2){
        double ni = v1.j * v2.k - v1.k * v2.j;
        double nj = -(v1.i * v2.k - v1.k * v2.i);
        double nk = v1.i * v2.j - v1.j * v2.i;
        return MyVector(ni, nj, nk);
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
// int main(int argc, char const *argv[])
// {
//     MyVector v1(1, 0, 0);
//     MyQuaternion q1(0.5, MyVector(0.5, 0.5, 0.5));
//     MyQuaternion q1n = q1.normalize();
//     std::cout << q1n << std::endl;
//     MyQuaternion q1i = q1n.invert();
//     std::cout << q1i << std::endl;
//     MyQuaternion q3 = MyQuaternion(0, v1);
//     MyQuaternion q2 = q1n * q3 * q1i;
//     std::cout << q2 << std::endl;
//     return 0;
// }

 

void inversion(double **A, int N)
{
    double temp;
 
    double **E = new double *[N];
 
    for (int i = 0; i < N; i++)
        E[i] = new double [N];
 
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
        {
            E[i][j] = 0.0;
 
            if (i == j)
                E[i][j] = 1.0;
        }
 
    for (int k = 0; k < N; k++)
    {
        temp = A[k][k];
 
        for (int j = 0; j < N; j++)
        {
            A[k][j] /= temp;
            E[k][j] /= temp;
        }
 
        for (int i = k + 1; i < N; i++)
        {
            temp = A[i][k];
 
            for (int j = 0; j < N; j++)
            {
                A[i][j] -= A[k][j] * temp;
                E[i][j] -= E[k][j] * temp;
            }
        }
    }
 
    for (int k = N - 1; k > 0; k--)
    {
        for (int i = k - 1; i >= 0; i--)
        {
            temp = A[i][k];
 
            for (int j = 0; j < N; j++)
            {
                A[i][j] -= A[k][j] * temp;
                E[i][j] -= E[k][j] * temp;
            }
        }
    }
 
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            A[i][j] = E[i][j];
 
    for (int i = 0; i < N; i++)
        delete [] E[i];
 
    delete [] E;
}
 
int main()
{
    int N;
 
    std::cout << "Enter N: ";
    std::cin >> N;
 
    double **matrix = new double *[N];
 
    for (int i = 0; i < N; i++)
        matrix[i] = new double [N];
 
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
        {
            std::cout << "Enter matrix[" << i << "][" << j << "] = ";
            std::cin >> matrix[i][j];
        }
 
    inversion(matrix, N);
 
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
            std::cout << matrix[i][j] << "  ";
 
        std::cout << std::endl;
    }
 
    for (int i = 0; i < N; i++)
        delete [] matrix[i];
 
    delete [] matrix;
 
    std::cin.get();
    return 0;
}