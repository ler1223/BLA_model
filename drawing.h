#include <iostream>
#include "quaternionMath.h"
#include "raylib.h"

class MyParalepiped{
    public:
    Vector3 startPosition, position, R;
    Vector3* startVectArr = new Vector3[8];
    Vector3* vectArr = new Vector3[8];
    Color color;

    MyParalepiped(Vector3* vectArr, Color color): startVectArr(vectArr), vectArr(vectArr), color(color){}
    MyParalepiped(Vector3 position, double angle, float width, float height, float length, Color color): R(Vector3{0, 0, 0}), startPosition(position), position(position), color(color){
        MyVector vDir0 = MyVector(abs(round(cos(angle))),  0 ,abs(round(sin(angle))));
        MyVector vDir1 = MyVector(abs(round(cos(angle + 3.14/2))), 0, abs(round(sin(angle + 3.14/2))));
        MyVector vDir2 = MyVector(0, 1, 0);

        MyVector pos = MyVector(position.x, position.y, position.z);
        startVectArr[0] = ((vDir0 * (length / 2)) + (vDir1 * (width / 2)) + (vDir2 * (height / 2))).toVect();
        startVectArr[1] = ((vDir0 * (length / 2)) + (vDir1 * -(width / 2)) + (vDir2 * (height / 2))).toVect();
        startVectArr[2] = ((vDir0 * -(length / 2)) + (vDir1 * (width / 2)) + (vDir2 * (height / 2))).toVect();
        startVectArr[3] = ((vDir0 * -(length / 2)) + (vDir1 * -(width / 2)) + (vDir2 * (height / 2))).toVect();
        startVectArr[4] = ((vDir0 * (length / 2)) + (vDir1 * (width / 2)) + (vDir2 * -(height / 2))).toVect();
        startVectArr[5] = ((vDir0 * (length / 2)) + (vDir1 * -(width / 2)) + (vDir2 * -(height / 2))).toVect();
        startVectArr[6] = ((vDir0 * -(length / 2)) + (vDir1 * (width / 2)) + (vDir2 * -(height / 2))).toVect();
        startVectArr[7] = ((vDir0 * -(length / 2)) + (vDir1 * -(width / 2)) + (vDir2 * -(height / 2))).toVect();
        vectArr[0] = ((vDir0 * (length / 2)) + (vDir1 * (width / 2)) + (vDir2 * (height / 2))).toVect();
        vectArr[1] = ((vDir0 * (length / 2)) + (vDir1 * -(width / 2)) + (vDir2 * (height / 2))).toVect();
        vectArr[2] = ((vDir0 * -(length / 2)) + (vDir1 * (width / 2)) + (vDir2 * (height / 2))).toVect();
        vectArr[3] = ((vDir0 * -(length / 2)) + (vDir1 * -(width / 2)) + (vDir2 * (height / 2))).toVect();
        vectArr[4] = ((vDir0 * (length / 2)) + (vDir1 * (width / 2)) + (vDir2 * -(height / 2))).toVect();
        vectArr[5] = ((vDir0 * (length / 2)) + (vDir1 * -(width / 2)) + (vDir2 * -(height / 2))).toVect();
        vectArr[6] = ((vDir0 * -(length / 2)) + (vDir1 * (width / 2)) + (vDir2 * -(height / 2))).toVect();
        vectArr[7] = ((vDir0 * -(length / 2)) + (vDir1 * -(width / 2)) + (vDir2 * -(height / 2))).toVect();
        MyQuaternion q = MyQuaternion(1, MyVector(Vector3{0.f, 0.f, 0.f}));
        rotateQuaternion(q, MyVector(0, 0, 0));
    }

    void rotateQuaternion(MyQuaternion q, MyVector v){
        MyQuaternion qn = q.normalize();
        MyQuaternion qi = qn.invert();
        MyQuaternion pos = MyQuaternion(0, MyVector(startPosition) + v.invert());
        MyQuaternion resPos = qn * pos * qi;
        position = resPos.v.toVect();
        for (int i = 0; i < 8; i++)
        {
            MyQuaternion inputQ = MyQuaternion(0, MyVector(startVectArr[i].x, startVectArr[i].y, startVectArr[i].z) + v.invert());
            MyQuaternion resQ = qn * inputQ * qi;
            vectArr[i] = (resQ.v + position + R).toVect();
        }
    }

    void DrawParalepiped(){
        DrawTriangle3D(vectArr[1], vectArr[0], vectArr[3], color);
        DrawTriangle3D(vectArr[0], vectArr[2], vectArr[3], color);

        DrawTriangle3D(vectArr[5], vectArr[0], vectArr[1], color);
        DrawTriangle3D(vectArr[5], vectArr[4], vectArr[0], color);

        DrawTriangle3D(vectArr[4], vectArr[2], vectArr[0], color);
        DrawTriangle3D(vectArr[4], vectArr[6], vectArr[2], color);

        DrawTriangle3D(vectArr[3], vectArr[2], vectArr[7], color);
        DrawTriangle3D(vectArr[2], vectArr[6], vectArr[7], color);

        DrawTriangle3D(vectArr[1], vectArr[3], vectArr[5], color);
        DrawTriangle3D(vectArr[3], vectArr[7], vectArr[5], color);

        DrawTriangle3D(vectArr[7], vectArr[4], vectArr[5], color);
        DrawTriangle3D(vectArr[7], vectArr[6], vectArr[4], color);

        DrawLine3D(vectArr[0], vectArr[1], RED);
        DrawLine3D(vectArr[0], vectArr[2], RED);
        DrawLine3D(vectArr[2], vectArr[3], RED);
        DrawLine3D(vectArr[3], vectArr[1], RED);
        DrawLine3D(vectArr[6], vectArr[4], RED);
        DrawLine3D(vectArr[6], vectArr[7], RED);
        DrawLine3D(vectArr[7], vectArr[5], RED);
        DrawLine3D(vectArr[4], vectArr[5], RED);
        DrawLine3D(vectArr[6], vectArr[2], RED);
        DrawLine3D(vectArr[7], vectArr[3], RED);
        DrawLine3D(vectArr[5], vectArr[1], RED);
        DrawLine3D(vectArr[4], vectArr[0], RED);
    }
};

class MyRotor{
    public:
    MyParalepiped MyPar;
    MyVector startP;
    MyVector P;
    double p;
    double w;
    double b, d;

    MyRotor(Vector3 position, double b, double d, double w, Color color):MyPar(position, 0, 1.4f, 1.4f, 1.4f, color), P(0, 1, 0), startP(0, 1, 0), b(b), d(d), w(w), p(0){}
    
    void UpdateW(double dw){w = dw; P = returnForce();}

    void DrawRotor(){
        MyPar.DrawParalepiped();
        DrawLine3D((MyVector(MyPar.position) + MyVector(MyPar.R)).toVect(), (MyVector(MyPar.position) + MyVector(MyPar.R) + P).toVect(), BLACK);
    }

    void rotate(MyQuaternion q, MyVector v){
        MyPar.rotateQuaternion(q, v);

        MyQuaternion p = MyQuaternion(0, (startP));
        MyQuaternion qn = q.normalize();
        MyQuaternion qi = qn.invert();
        MyQuaternion resP = qn * p * qi;
        P = resP.v;
    }

    MyVector returnForce(){
        return P * b * w * w;
    }

    double returnMoment(){
        return  d * w * w;
    }
};

class MyGraf{
    public:
    double* posXdes;
    double* posYdes;
    double* posXmodel;
    double* posYmodel;
    Vector2 position;
    int mult = 10, dm = 1;
    Vector2 startPos;
    Vector2 size;

    MyGraf(double* posXdes, double* posYdes, double* posXmodel, double* posYmodel): startPos({0, 0}), size({500, 500}), position({0, 0}), posXdes(posXdes), posYdes(posYdes), posXmodel(posXmodel), posYmodel(posYmodel){};

    void draw(std::vector<Vector2> arr){
        DrawRectangle(startPos.x, startPos.y, size.x, size.y, BLACK);
        drawAxes();
        drawGrafic(arr);
    }

    void drawAxes(){
        int HWM = size.x / 2;
        Vector2 xAxisStart = {startPos.x, startPos.y + size.y / 2 + position.y};
        Vector2 xAxisEnd = {startPos.x + size.x, startPos.y + size.y / 2 + position.y};
        Vector2 yAxisStart = {startPos.x + size.x / 2 + position.x, startPos.y};
        Vector2 yAxisEnd = {startPos.x + size.x / 2 + position.x, startPos.y + size.y};

        for (int x = startPos.x + size.x / 2 + position.x; x < startPos.x + size.x; x+=1*mult){
            DrawLine(x,startPos.y + size.y / 2 - HWM, x, startPos.y + size.y / 2 + HWM, DARKGRAY);
        }

        for (int x = startPos.x + size.x / 2 + position.x; x > startPos.x; x-=1*mult){
            DrawLine(x,startPos.y + size.y / 2 - HWM, x, startPos.y + size.y / 2 + HWM, DARKGRAY);
        }

        for (int y = startPos.y + size.y / 2 + position.y; y < startPos.y + size.y; y+=1*mult){
            DrawLine(startPos.x + size.x / 2 - HWM, y, startPos.x + size.x / 2 + HWM, y, DARKGRAY);
        }

        for (int y = startPos.y + size.y / 2 + position.y; y > startPos.y; y-=1*mult){
            DrawLine(startPos.x + size.x / 2 - HWM, y, startPos.x + size.x / 2 + HWM, y, DARKGRAY);
        }
        DrawLineEx(xAxisStart, xAxisEnd, 2, GRAY);
        DrawLineEx(yAxisStart, yAxisEnd, 2, GRAY);
    }

    void drawGrafic(std::vector<Vector2> arr){
        for (Vector2 i : arr)
        {
            double x = i.x * mult + startPos.x + size.x / 2 + position.x, y = i.y * mult + startPos.y + size.y / 2 + position.y;
            if(x > startPos.x && x < startPos.x + size.x && y > startPos.y && y < startPos.y + size.y) DrawCircle(x, y, 2, RED);
        }
        drawPosDes();
    }

    void drawPosDes(){
        double xM = *posXdes * mult + startPos.x + size.x / 2 + position.x, yM = *posYdes * mult + startPos.y + size.y / 2 + position.y;
        if(xM > startPos.x && xM < startPos.x + size.x && yM > startPos.y && yM < startPos.y + size.y) DrawCircle(xM, yM, 3, GREEN);
        double xD = *posXmodel * mult + startPos.x + size.x / 2 + position.x, yD = *posYmodel * mult + startPos.y + size.y / 2 + position.y;
        if(xD > startPos.x && xD < startPos.x + size.x && yD > startPos.y && yD < startPos.y + size.y) DrawCircle(xD, yD, 3, MAGENTA);
    }

};