
class MyMultiplier{
    private:
    double k;

    public:
    MyMultiplier(double k):k(k){};

    double combing(double x){
        return x * k;
    }
};

class MyDifferentiator{
    private:
    double xt1;
    double xt2;
    double t1;
    double t2;

    public:
    MyDifferentiator(double xt0):xt1(xt0), xt2(xt0), t1(0), t2(0){};

    double combing(double newX, double dt){
        xt1 = xt2;
        t1 = t2;
        xt2 = newX;
        t2 = t1 + dt;
        return (xt2 - xt1) / (dt);
    }
};

class MyIntegrator{
    private:
    double x;

    public:
    MyIntegrator(double xt0):x(xt0){};

    double combing(double newX,  double dt){
        x += newX * dt;
        return x;
    }
};

class MyLimiter{
    private:
    double minX;
    double maxX;

    public:
    MyLimiter(double minX, double maxX):minX(minX), maxX(maxX){};

    double combing(double x){
        if(x < minX){
            return minX;
        }
        if(x > maxX){
            return maxX;
        }
        return x;
    }
};

class MyPControler{
    private:
    double* xtcontr;
    double* xt;
    MyMultiplier k;
    public:
    MyPControler(double *xtcontr, double k, double* xModel):k(k), xtcontr(xtcontr), xt(xModel){}
    double combing(){
        return k.combing(*xtcontr - *xt);
    }
};

class MyIControler{
    private:
    double* xtcontr;
    double* xt;
    MyMultiplier k;
    MyIntegrator integrator;
    public:
    MyIControler(double *xtcontr, double k, double* xModel):k(k), xtcontr(xtcontr), xt(xModel), integrator(0.){}
    double combing(double dt){
        if(*xtcontr - *xt == 0){
            return 0.;
        }
        return 1. / integrator.combing(k.combing(*xtcontr - *xt), dt);
    }
};

class MyDControler{
    private:
    double* xtcontr;
    double* xt;
    MyMultiplier k;
    MyDifferentiator deff;
    public:
    MyDControler(double *xtcontr, double k, double* xModel):k(k), xtcontr(xtcontr), xt(xModel), deff(0.){}
    double combing(double dt){
        return deff.combing(k.combing(*xtcontr - *xt), dt);
    }
};

class MyPDControler{
    private:
    double* xtcontr;
    double* xt;
    MyMultiplier kp;
    MyMultiplier kd;
    MyDifferentiator deff;
    public:
    MyPDControler(double *xtcontr, double kp, double kd, double* xModel):kp(kp), kd(kd), xtcontr(xtcontr), xt(xModel), deff(0.){}
    double combing(double dt){
        return kp.combing(*xtcontr - *xt) + deff.combing(kd.combing(*xtcontr - *xt), dt);
    }
};

class MyPIDControler{
    private:
    double* xtcontr;
    double* xt;
    MyMultiplier kp;
    MyMultiplier kd;
    MyMultiplier ki;
    MyDifferentiator deff;
    MyIntegrator integrator;
    MyLimiter limiter;
    public:
    MyPIDControler(double *xtcontr, double kp, double kd, double ki, double* xModel, double minX, double maxX):kp(kp), kd(kd), ki(ki), xtcontr(xtcontr), xt(xModel), deff(0.), integrator(0.), limiter(minX, maxX){}
    double combing(double dt){
        return limiter.combing(kp.combing(*xtcontr - *xt) + deff.combing(kd.combing(*xtcontr - *xt), dt) + integrator.combing(ki.combing(*xtcontr - *xt), dt));
    }
};
