//solves \tan x+\frac{\left(dg\cos x\right)}{2v^{2}\cos^{2}y}-\tan y=0
//g++ simple_shot_solver.cpp -o solver.exe
//solver.exe

#include <iostream>
#include <chrono>
#include <math.h>
#include <iomanip>


const double BOUND = 1.39626;//80 degrees
const double EPSILON = 0.0000001;


double func(const double theta1, const double d, const double g, const double v, const double alpha = 1){
    if(theta1 > BOUND || theta1 < -BOUND){
        return NAN;
    }

    const double tan1 = std::tan(theta1);
    const double dgcosx_over_2v2 = (d*g*std::cos(theta1)) / (2*v*v);
    
    double guess = 0;
    double err = tan1 + dgcosx_over_2v2;
    while(std::abs(err) > EPSILON){
        //the closer the guess is to pi/2, the smaller the step
        double diff = M_PI/2 - std::abs(guess);
        double step = alpha * err * diff;
        guess += step;

        double cosy = std::cos(guess);
        double tany = std::tan(guess);
        err = tan1 + dgcosx_over_2v2 / (cosy*cosy) - tany;

        if(guess > M_PI || guess < -M_PI){
            return INFINITY;
        }
    }

    return guess;
}





int main(int argc, char** argv){
    double d = 1;
    double g = 10;
    double v = 10;
    double alpha = 0.1;                
    if(argc < 4){
        std::cout << "Using Defaults:\n";
    }
    else{
        d = std::stod(argv[1]);
        g = std::stod(argv[2]);
        v = std::stod(argv[3]);

        if(argc == 5){
            alpha = std::stod(argv[4]);
        }
    }
    
    std::cout << "d: " << d << "\n";
    std::cout << "g: " << g << "\n";
    std::cout << "v: " << v << "\n";
    std::cout << "alpha: " << alpha << "\n";




    std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();

    //test func
    for(int i = -80; i < 81; i++){
        double theta1 = i * 0.0174532925199;
        double out = func(theta1, d, g, v, alpha);
        std::cout <<  theta1 << ": \t" << out << '\n';
    }

    std::chrono::time_point<std::chrono::steady_clock> end = std::chrono::steady_clock::now();
    std::chrono::microseconds diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "Time Elapsed: " << diff.count() << " microseconds \n";


    return 0;
}