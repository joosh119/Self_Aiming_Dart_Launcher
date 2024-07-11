// Solves: 
// \theta_{1}=x
// \theta_{2}=y
// \sin\left(\theta_{1}+\theta_{2}\right)\left(\frac{d\cos\left(\theta_{1}\right)+\left(o_{x}-r\cos\left(\theta_{2}\right)\right)\cos\left(\theta_{1}\right)\ +\left(o_{y}+r\sin\left(\theta_{2}\right)\right)\sin\left(\theta_{1}\right)}{\cos\left(\theta_{1}+\theta_{2}\right)}\right)-\frac{g}{2}\left(\frac{d\cos\left(\theta_{1}\right)\ +\left(o_{x}-r\cos\left(\theta_{2}\right)\right)\cos\left(\theta_{1}\right)\ +\left(o_{y}+r\sin\left(\theta_{2}\right)\right)\sin\left(\theta_{1}\right)}{s\cos\left(\theta_{1}+\theta_{2}\right)}\right)^{2}=d\sin\left(\theta_{1}\right)+\left(o_{x}-r\cos\left(\theta_{2}\right)\right)\sin\left(\theta_{1}\right)\ -\left(o_{y}+r\sin\left(\theta_{2}\right)\right)\cos\left(\theta_{1}\right)

//g++ shot_solver.cpp -o solver.exe
//solver.exe

#include <iostream>
#include <chrono>
#include <math.h>
#include <iomanip>


const double BOUND = 1.22173;// 70 degrees
const double MIN_RETURN_ANGLE = 0;// 0 degrees
const double MAX_RETURN_ANGLE = 1.22173;// 70 degrees
const double EPSILON = 0.000001;


double func(const double theta1, const double d, const double g, const double s, const double ox, const double oy, const double r, const double alpha = 1){
    if(theta1 > BOUND || theta1 < -BOUND){
        return NAN;
    }

    const double dx = d*std::cos(theta1);
    const double dy = d*std::sin(theta1);
    const double cosTheta1 = std::cos(theta1);
    const double sinTheta1 = std::sin(theta1);
    
    double guess = 0;
    double err = 0;
    while(true){
        // Use the guess to find the error
        const double thetaT = theta1 + guess;

        const double vx = s*std::cos(thetaT);
        const double vy = s*std::sin(thetaT);
        
        const double rx = r*std::cos(guess);
        const double ry = r*std::sin(guess);
        const double cx = ox - rx;
        const double cy = oy + ry;

        const double Dx = dx + (cx*cosTheta1) + (cy*sinTheta1);
        const double Dy = dy + (cx*sinTheta1) - (cy*cosTheta1);

        const double DxOvervx = Dx / vx;

        err = vy*DxOvervx - (g/2)*DxOvervx*DxOvervx - Dy;

        // Check if the error is small enough
        if(std::abs(err) <= EPSILON){
            break;
        }

        // Update the guess
        // The closer the guess is to PI/2, the smaller the step
        //const double diff = M_PI/2 - std::abs(guess);
        const double step = alpha * (-err);// * diff;
        guess += step;
    }

    if(guess < MIN_RETURN_ANGLE){
        return MIN_RETURN_ANGLE;
    }
    else if(guess > MAX_RETURN_ANGLE){
        return MAX_RETURN_ANGLE;
    }

    return guess;
}


const double d = 1;
const double g = 9.81;
const double s = 7.5;
const double oX = 0.06;
const double oY = 0.03;
const double r = 0.08;
const double alpha = 1;

// Finds the angle that the angle servo should move to, in radians
double findShootAngle(const double theta1, const double _d, const double startGuess) {
  if (theta1 > BOUND || theta1 < -BOUND) {
    return 100000;
  }

  const double dx = d * std::cos(theta1);
  const double dy = d * std::sin(theta1);

  const double cosTheta1 = std::cos(theta1);
  const double sinTheta1 = std::sin(theta1);

  double guessTheta2 = startGuess;
  double err = 0;
  while (true) {
    // Use the guess to find the error
    const double thetaT = theta1 + guessTheta2;

    const double vx = s * std::cos(thetaT);
    const double vy = s * std::sin(thetaT);

    const double rx = r * std::cos(guessTheta2);
    const double ry = r * std::sin(guessTheta2);
    const double cx = oX - rx;
    const double cy = oY + ry;

    const double Dx = dx + (cx * cosTheta1) + (cy * sinTheta1);
    const double Dy = dy + (cx * sinTheta1) - (cy * cosTheta1);

    const double DxOvervx = Dx / vx;

    err = vy * DxOvervx - (g / 2) * DxOvervx * DxOvervx - Dy;

    // Check if the error is small enough
    if (err <= EPSILON && err >= -EPSILON) { // This uses less instruction memory than using abs()
      break;
    }

    // Update the guess
    //// The closer the guess is to PI/2, the smaller the step
    //const double diff = M_PI / 2 - guessTheta2;
    const double step = -alpha * err;// * diff;
    guessTheta2 += step;
  }

  return guessTheta2;
}



int main(int argc, char** argv){
    double _d = 1;
    double _g = 9.81;
    double _s = 10;
    double _ox = 0.06;
    double _oy = 0.03;
    double _r = 0.07;
    double _alpha = 0.1;
    if(argc < 7){
        std::cout << "Using Defaults:\n";
    }
    else{
        _d = std::stod(argv[1]);
        _g = std::stod(argv[2]);
        _s = std::stod(argv[3]);
        _ox = std::stod(argv[4]);
        _oy = std::stod(argv[5]);
        _r = std::stod(argv[6]);

        if(argc == 8){
            _alpha = std::stod(argv[7]);
        }
    }
    
    std::cout << "d: " << _d << "\n";
    std::cout << "g: " << _g << "\n";
    std::cout << "s: " << _s << "\n";
    std::cout << "ox: " << _ox << "\n";
    std::cout << "oy: " << _oy << "\n";
    std::cout << "r: " << _r << "\n";
    std::cout << "alpha: " << _alpha << "\n\n";




    std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();

    //test func
     std::cout << "theta1:    \ttheta2:\n";
    double prev = 0;
    for(int i = -71; i < 71; i++){
        double theta1 = i * 0.0174532925199;
        //double out = func(theta1, d, _g, _s, _ox, _oy, _r, _alpha);
        double out = findShootAngle(theta1, _d, prev);
        std::cout <<  theta1 << ": \t" << out << '\n';
        if(out < M_PI)
            prev = out;
    }

    std::chrono::time_point<std::chrono::steady_clock> end = std::chrono::steady_clock::now();
    std::chrono::microseconds diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "\nTime Elapsed: " << diff.count() << " microseconds \n";
    float timePerCalc = diff.count()/141;
    std::cout << "Microseconds Per Calculation: " << timePerCalc << " microseconds \n";
    std::cout << "Seconds Per Calculation: " << timePerCalc*1e-6 << " seconds \n";

    return 0;
}