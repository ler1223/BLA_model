/*******************************************************************************************
*
*   raylib [core] example - Initialize 3d camera free
*
*   Example originally created with raylib 1.3, last time updated with raylib 1.3
*
*   Example licensed under an unmodified zlib/libpng license, which is an OSI-certified,
*   BSD-like license that allows static linking with closed source software
*
*   Copyright (c) 2015-2024 Ramon Santamaria (@raysan5)
*
********************************************************************************************/
// #include "quaternionMath.h"
#include "controlers.h"
#include "drawing.h"
#include "raylib.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "/Users/user/Downloads/eigen-master/Eigen/Dense"

class ModelBLA4{
    private:
    MyParalepiped Par1 = MyParalepiped(Vector3{0.f, 0.f, 0.f}, 0, 1.f, 1.f, 5.f, GREEN);
    MyParalepiped Par2 = MyParalepiped(Vector3{0.f, 0.f, 0.f}, 3.14/2, 1.f, 1.f, 5.f, GREEN);
    public:
    MyRotor rot1 = MyRotor(Vector3{0, 0, 2.5}, 0.000025, 0.0000000025, 1000, YELLOW);
    MyRotor rot2 = MyRotor(Vector3{2.5, 0, 0}, 0.000025, 0.0000000025, 1000, GRAY);
    MyRotor rot3 = MyRotor(Vector3{0, 0, -2.5}, 0.000025, 0.0000000025, 1000, ORANGE);
    MyRotor rot4 = MyRotor(Vector3{-2.5, 0, 0}, 0.000025, 0.0000000025, 1000, RED);
    double M = 10;
    MyVector startG = MyVector(0.f, -9.8f, 0.f);
    MyVector G = MyVector(0.f, -9.8f, 0.f);
    double Cd = 0.0001, Ir = 0.01;
    MyVector speed = MyVector(0.f, 0.f, 0.f);
    MyVector acceleration = MyVector(0.f, 0.f, 0.f);
    MyVector R = MyVector(0.f, 0.f, 0.f);
    MyVector accurAngle = MyVector(0.f, 0.f, 0.f);
    MyVector speedAngle = MyVector(0.f, 0.f, 0.f);
    MyVector angle = MyVector(0.f, 0.f, 0.f);

    void DrawModel(){
        Par1.DrawParalepiped();
        Par2.DrawParalepiped();
        rot1.DrawRotor();
        rot2.DrawRotor();
        rot3.DrawRotor();
        rot4.DrawRotor();
    }

    void rotate(MyQuaternion q, MyVector v){
        Par1.rotateQuaternion(q, v);
        Par2.rotateQuaternion(q, v);
        rot1.rotate(q, v);
        rot2.rotate(q, v);
        rot3.rotate(q, v);
        rot4.rotate(q, v);
        MyQuaternion g = MyQuaternion(0, (startG));
        MyQuaternion qn = q.normalize();
        MyQuaternion qi = qn.invert();
        MyQuaternion resG = qn * g * qi;
        G = resG.v;
    }

    void corectSpeedRot(double dw1, double dw2, double dw3, double dw4){
        rot1.w = dw1;
        rot2.w = dw2;
        rot3.w = dw3;
        rot4.w = dw4;
    }

    void move(MyVector v){
        R = R + v;
        Par1.R = (MyVector(Par1.R) + v).toVect();
        Par2.R = (MyVector(Par2.R) + v).toVect();
        rot1.MyPar.R = (MyVector(rot1.MyPar.R) + v).toVect();
        rot2.MyPar.R = (MyVector(rot2.MyPar.R) + v).toVect();
        rot3.MyPar.R = (MyVector(rot3.MyPar.R) + v).toVect();
        rot4.MyPar.R = (MyVector(rot4.MyPar.R) + v).toVect();
    }

    void updateForce(double t){

        MyVector normal = rot1.P * (1/rot1.P.lenght());
        MyVector A = rot1.P * (1/rot1.P.lenght());
        A.i = 25 * abs(normal.i) + 5 * abs(normal.k) + 5 * abs(normal.j);
        A.j = 25 * abs(normal.j) + 5 * abs(normal.i) + 5 * abs(normal.k);
        A.k = 25 * abs(normal.k) + 5 * abs(normal.j) + 5 * abs(normal.i);

        MyVector Fd = A * (-0.5) * Cd * 1.22 * speed * speed.absolut();
        
        acceleration = (rot1.returnForce() + rot2.returnForce() + rot3.returnForce() + rot4.returnForce() + Fd + startG * M) * (1 / M);
        speed = speed + acceleration * t;
        move(speed * t);
        updateMoment(t);
    }

    void updateMoment(double t){
        double sumW = rot1.w - rot2.w + rot3.w - rot4.w;
        double wj = speedAngle.j * sumW * Ir;
        double wk = speedAngle.k * sumW * Ir;
        
        accurAngle.i = rot1.returnMoment() - rot2.returnMoment() + rot3.returnMoment() - rot4.returnMoment();
        accurAngle.j = rot1.returnForce().lenght2() > rot3.returnForce().lenght2()?((rot1.returnForce() + rot3.returnForce().invert()).lenght() + wj) * 0.025:((rot1.returnForce() + rot3.returnForce().invert()).lenght() + wj)* -0.025;
        accurAngle.k = rot2.returnForce().lenght2() > rot4.returnForce().lenght2()?((rot2.returnForce() + rot4.returnForce().invert()).lenght() + wk) * -0.025:((rot2.returnForce() + rot4.returnForce().invert()).lenght() + wk)* 0.025;
        speedAngle = speedAngle + accurAngle * t;
        angle = angle + speedAngle * t;
    }
};

class Controler{
    public:
    ModelBLA4* model;
    MyVector R = MyVector(0,0,0);
    MyPIDControler contrPosX;
    MyPIDControler contrPosY;
    MyPIDControler contrPosZ;
    MyPIDControler contrSpeedX;
    MyPIDControler contrSpeedY;
    MyPIDControler contrAccurX;
    MyPIDControler contrAccurY;
    MyPIDControler contrAngleJ;
    MyPIDControler contrAngleK;
    MyPIDControler contrAngleSpeedJ;
    MyPIDControler contrAngleSpeedK;
    double bx0=0, bx1=0, bx2=0, bx3=0, bx4=0, bx5=0;
    double by0=0, by1=0, by2=0, by3=0, by4=0, by5=0;
    double bz0=0, bz1=0, bz2=0, bz3=0, bz4=0, bz5=0;
    double PosXdes, PosYdes, PosZdes, speedXdes, speedYdes, accurXdes, accurYdes, AngleJdes, AngleKdes, AngleSpeedJdes, AngleSpeedKdes, thrustcontr, time;

    Controler(ModelBLA4 *model): model(model), 
    contrPosX(&PosXdes, 0.15, 0.4, 0.0001, &model->R.i, -1.5, 1.5),
    contrPosY(&PosYdes, 0.15, 0.4, 0.0001, &model->R.k, -1.5, 1.5),
    contrPosZ(&PosZdes, 400., 500.0, 0.0001, &model->R.j, 0., 2000.),

    contrSpeedX(&speedXdes, 0.45, 0.4, 0.000001, &model->speed.i, -1.5, 1.5),
    contrSpeedY(&speedYdes, 0.45, 0.4, 0.000001, &model->speed.k, -1.5, 1.5),

    contrAccurX(&accurXdes, 0.005, 0.04, 0.000001, &model->acceleration.i, -1.5, 1.5),
    contrAccurY(&accurYdes, 0.005, 0.04, 0.000001, &model->acceleration.k, -1.5, 1.5),

    contrAngleJ(&AngleJdes, 20., 4.7, 0.00000001, &model->angle.j, -25.0, 25.0),
    contrAngleK(&AngleKdes, 20., 4.7, 0.00000001, &model->angle.k, -25.0, 25.0), 
    contrAngleSpeedJ(&AngleSpeedJdes, 3000., 10.0, 0.00000001, &model->speedAngle.j, -1600., 1600.), 
    contrAngleSpeedK(&AngleSpeedKdes, 3000., 10.0, 0.00000001, &model->speedAngle.k, -1600., 1600.),
    thrustcontr(1000.), PosXdes(0.), PosYdes(0.), PosZdes(4.), speedXdes(0.), speedYdes(0.), accurXdes(0.), accurYdes(0.), AngleJdes(0.), AngleKdes(0.), AngleSpeedJdes(0.), AngleSpeedKdes(0.), time(0.){}

    void up(){
        thrustcontr += 100;
    }

    void down(){
        thrustcontr -= 100;
    }

    void setbx(double b0, double b1, double b2, double b3, double b4, double b5){
        bx0 = b0;
        bx1 = b1;
        bx2 = b2;
        bx3 = b3;
        bx4 = b4;
        bx5 = b5;
    }

    void setby(double b0, double b1, double b2, double b3, double b4, double b5){
        by0 = b0;
        by1 = b1;
        by2 = b2;
        by3 = b3;
        by4 = b4;
        by5 = b5;
    }

    void setbz(double b0, double b1, double b2, double b3, double b4, double b5){
        bz0 = b0;
        bz1 = b1;
        bz2 = b2;
        bz3 = b3;
        bz4 = b4;
        bz5 = b5;
    }

    void test(double dt){
        time += dt;
        // PosXdes = 8 * cos(time / 4);
        // PosYdes = 10 * sin(time / 3);

        // speedXdes = -2. * sin(time / 4);
        // speedYdes = 10. / 3. * cos(time / 3);

        // PosZdes = time * 0.5;
        // PosXdes = 4 * cos(time / 2);
        // PosYdes = 4 * sin(time / 2);
        if(time < 15){
            PosXdes = pow(time, 5) * bx5 + pow(time, 4) * bx4 + pow(time, 3) * bx3 + pow(time, 2) * bx2 + time * bx1 + bx0;
            PosYdes = pow(time, 5) * by5 + pow(time, 4) * by4 + pow(time, 3) * by3 + pow(time, 2) * by2 + time * by1 + by0;
            PosZdes = pow(time, 5) * bz5 + pow(time, 4) * bz4 + pow(time, 3) * bz3 + pow(time, 2) * bz2 + time * bz1 + bz0;

            speedXdes = 5*pow(time, 4) * bx5 + 4*pow(time, 3) * bx4 + 3*pow(time, 2) * bx3 + 2*time * bx2 + bx1;
            speedYdes = 5*pow(time, 4) * by5 + 4*pow(time, 3) * by4 + 3*pow(time, 2) * by3 + 2*time * by2 + by1;

            accurXdes = 20*pow(time, 3) * bx5 + 12*pow(time, 2) * bx4 + 6*time * bx3 + 2 * bx2;
            accurYdes = 20*pow(time, 3) * by5 + 12*pow(time, 2) * by4 + 6*time * by3 + 2 * by2;
        } else{
            speedXdes = 0;
            speedYdes = 0;
            accurXdes = 0;
            accurYdes = 0;
        }
        // std::cout << PosXdes << " " << PosYdes << std::endl;
    }

    void defaultW(){
        double nw = (model->rot1.w + model->rot2.w + model->rot3.w + model->rot4.w) * 0.25;
        model->rot1.w = nw;
        model->rot2.w = nw;
        model->rot3.w = nw;
        model->rot4.w = nw;
    }

    void move(double dt){
        AngleJdes = -contrPosX.combing(dt) -contrSpeedX.combing(dt)/4;
        AngleKdes = contrPosY.combing(dt) +contrSpeedY.combing(dt)/4;

        // AngleJdes = -contrPosX.combing(dt)/3 -contrSpeedX.combing(dt)/3 -contrAccurX.combing(dt)/3;
        // AngleKdes = contrPosY.combing(dt)/3 +contrSpeedY.combing(dt)/3 +contrAccurY.combing(dt)/3;

        thrustcontr = contrPosZ.combing(dt);
        AngleSpeedJdes = contrAngleJ.combing(dt) -contrAccurX.combing(dt);
        AngleSpeedKdes = contrAngleK.combing(dt) +contrAccurY.combing(dt);

        double nw1 = thrustcontr, nw2 = thrustcontr, nw3 = thrustcontr, nw4 = thrustcontr;
        double pcj = contrAngleSpeedJ.combing(dt);
        double s1 = nw1 + nw3;
        nw1 = (s1 + pcj) / 2;
        nw3 = nw1 - pcj;

        double pck = contrAngleSpeedK.combing(dt);
        double s2 = nw2 + nw4;
        nw4 = (s2 + pck) / 2;
        nw2 = nw4 - pck;

        // std::cout << AngleJdes << " | " << AngleKdes << " | " << thrustcontr << " | " << AngleSpeedJdes << " | " << AngleSpeedKdes << " | " << pcj << " | " << pck << " " << model->R << std::endl;

        model->corectSpeedRot(nw1, nw2, nw3, nw4);
    }
};


MyQuaternion angleToQuaterion(double yaw, double pitch, double roll, ModelBLA4 model){
    MyQuaternion qYaw(cos((yaw + model.angle.j) / 2), MyVector(0.f, 0.f, sin((yaw + model.angle.j) / 2)));
    MyQuaternion qPitch(cos((pitch + model.angle.i) / 2), MyVector(0.f, sin((pitch + model.angle.i) / 2), 0.f));
    MyQuaternion qRoll(cos((roll + model.angle.k) / 2), MyVector(sin((roll + model.angle.k) / 2), 0.f, 0.f));
    MyQuaternion res = qYaw + qPitch + qRoll;
    return res;
}

enum class Mode {Quaterins, Angles, Graf};

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 1600;
    const int screenHeight = 1000;
    Mode mode = Mode::Graf;

    InitWindow(screenWidth, screenHeight, "raylib [core] example - 3d camera free");

    Image checked = GenImageChecked(2, 2, 1, 1, RED, GREEN);
    Texture2D texture = LoadTextureFromImage(checked);
    UnloadImage(checked);

    // Define the camera to look into our 3d world
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 50.0f, 50.0f, 50.0f }; // Camera position
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type

    // DisableCursor();                    // Limit cursor to relative movement inside the window
    int FPS = 60;

    SetTargetFPS(FPS);                   // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------

    float adH = 0, adI = 1, adJ = 0, adK = 0;
    float yaw = 0, pitch = 0, roll = 0;

    MyQuaternion ad = MyQuaternion(0, MyVector(adI, adJ, adK));

    bool flag = true, stab = true;
    std::srand(std::time(nullptr)); // use current time as seed for random generator

    ModelBLA4 model = ModelBLA4();
    Controler contr = Controler(&model);

    int index = 0;

    MyGraf graf = MyGraf(&contr.PosXdes, &contr.PosYdes, &model.R.i, &model.R.k);
    std::vector<Vector2> arrVector{};

    // Main game loop
    while (!WindowShouldClose())        // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        UpdateCamera(&camera, CAMERA_CUSTOM);

        if (IsKeyPressed('Z')) camera.target = model.R.toVect();;
        // if (IsKeyPressed('W')) contr.forward();
        // if (IsKeyPressed('S')) contr.back();
        // if (IsKeyPressed('A')) contr.left();
        if (IsKeyPressed('D')) {
            contr.time = 0;
            double T = 15;
            Eigen::MatrixXd A(6, 6);
            Eigen::MatrixXd A2(6, 6);
            Eigen::VectorXd x_coeff(6);
            Eigen::VectorXd y_coeff(6);
            Eigen::VectorXd z_coeff(6);
            Eigen::VectorXd bx(6);
            Eigen::VectorXd by(6);
            Eigen::VectorXd bz(6);

            // A << 0,             0,              0,              0,          0,  1,
            //  pow(T, 5),     pow(T, 4),      pow(T, 3),      pow(T, 2),  T,  1,
            //  0,             0,              0,              0,          1,  0,
            //  5*pow(T, 4),   4*pow(T, 3),    3*pow(T, 2),    2*T,        1,  0,
            //  0,             0,              0,              2,          0,  0,
            //  20*pow(T, 3),  12*pow(T, 2),   6*T,            2,          0,  0;

            A << 0,             0,              0,              0,          0,  1,
                pow(T/2, 5),   pow(T/2, 4),    pow(T/2, 3),    pow(T/2, 2), T/2, 1,
                pow(T, 5),     pow(T, 4),      pow(T, 3),      pow(T, 2),   T,   1,
                0,             0,              0,              0,           1,   0,
                5*pow(T/2, 4), 4*pow(T/2, 3),  3*pow(T/2, 2),  2*T/2,       1,   0,
                5*pow(T, 4),   4*pow(T, 3),    3*pow(T, 2),    2*T,         1,   0;

            // bx << 0, 8, 0, 0, 0, 0;
            // by << 0, 5, 0, 0, 0, 0;
            // bz << 0, 4, 0, 0, 0, 0;
            bx << 0, 4, 8, 0, 0, 0;
            by << 0, 1, 6, 0, 0, 0;
            bz << 0, 4, 7, 0, 0, 0;

            x_coeff = A.inverse() * bx;
            y_coeff = A.inverse() * by;
            z_coeff = A.inverse() * bz;
            contr.setbx(x_coeff[5], x_coeff[4], x_coeff[3], x_coeff[2], x_coeff[1], x_coeff[0]);
            contr.setby(y_coeff[5], y_coeff[4], y_coeff[3], y_coeff[2], y_coeff[1], y_coeff[0]);
            contr.setbz(z_coeff[5], z_coeff[4], z_coeff[3], z_coeff[2], z_coeff[1], z_coeff[0]);
        }
        if (IsKeyPressed('I')) flag==true?flag=false:flag=true;
        if (IsKeyPressed('T')) {
            int xRand = 20 + 1;
            while (xRand > 20) 
                xRand = 1 + std::rand() / ((RAND_MAX + 1u) / 20);
            int yRand = 20 + 1;
            while (yRand > 20) 
                yRand = 1 + std::rand() / ((RAND_MAX + 1u) / 20);
            contr.PosXdes = xRand - 10;
            contr.PosYdes = yRand - 10;
        }
        if (IsKeyPressed('R')) mode==Mode::Quaterins?mode=Mode::Angles:mode=Mode::Quaterins;
        if (IsKeyDown('Q')) {
            graf.mult += graf.dm;
            if(graf.mult > 100) graf.mult = 100;
        }
        if (IsKeyDown('E')) {
            graf.mult -= graf.dm;
            if(graf.mult < 10) graf.mult = 10;
        }
        if (IsKeyDown(KEY_UP)) graf.position.y += 0.1 * graf.mult;
        if (IsKeyDown(KEY_DOWN)) graf.position.y -= 0.1 * graf.mult;
        if (IsKeyDown(KEY_RIGHT)) graf.position.x -= 0.1 * graf.mult;
        if (IsKeyDown(KEY_LEFT)) graf.position.x += 0.1 * graf.mult;

        if (flag){
            ad = MyQuaternion(adH, MyVector(adI, adJ, adK));
            if (mode == Mode::Angles || mode == Mode::Graf) ad = angleToQuaterion(yaw, pitch, roll, model);
            contr.test(1.0/FPS);
            if(stab) contr.move(1.0/FPS);

            model.rotate(ad, MyVector(0,0,0));
            model.updateForce(1.0/FPS);
            camera.target = model.R.toVect();
            

            if(arrVector.size() >= 120){
                arrVector[index] = Vector2{(float)(model.R.i), (float)(model.R.k)};
                index = (index + 1) % 120;
            } else {
                arrVector.push_back(Vector2{(float)(model.R.i), (float)(model.R.k)});
            }
        }
        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();
            ClearBackground(RAYWHITE);

            BeginMode3D(camera);
                model.DrawModel();
                DrawGrid(1000, 1.0f);
            EndMode3D();

            if(mode == Mode::Quaterins){
                GuiSliderBar((Rectangle){ 600, 40, 120, 20}, "H", TextFormat("%.2f", adH), &adH, -1, 1);
                GuiSliderBar((Rectangle){ 600, 70, 120, 20}, "I", TextFormat("%.2f", adI), &adI, -1, 1);
                GuiSliderBar((Rectangle){ 600, 140, 120, 20}, "J", TextFormat("%.2f", adJ), &adJ, -1, 1);
                GuiSliderBar((Rectangle){ 600, 170, 120, 20}, "K", TextFormat("%.2f", adK), &adK, -1, 1);
            }
            if(mode == Mode::Angles){
                GuiSliderBar((Rectangle){ 600, 40, 120, 20}, "YAW", TextFormat("%.2f", yaw), &yaw, -1.57, 1.57);
                GuiSliderBar((Rectangle){ 600, 70, 120, 20}, "PITCH", TextFormat("%.2f", pitch), &pitch, -1.57, 1.57);
                GuiSliderBar((Rectangle){ 600, 140, 120, 20}, "ROLL", TextFormat("%.2f", roll), &roll, -1.57, 1.57);
            }
            if(mode == Mode::Graf){
                graf.draw(arrVector);
            }
        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}