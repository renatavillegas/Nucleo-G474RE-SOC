
// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>


#include "SystemModel.hpp"
//#include "OrientationMeasurementModel.hpp"
//#include "PositionMeasurementModel.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

#include <iostream>
#include <random>
#include <chrono>


using namespace KalmanExamples;

typedef float T;

// Some type shortcuts
typedef Soc::State<T> State;
typedef Soc::Control<T> Control;
typedef Soc::SystemModel<T> SystemModel;

typedef Soc::Current<T> CurrentMeasurement;
typedef Soc::CurrentMeasurementModel<T> CurrentModel;

int main(int argc, char** argv)
{
    // Simulated (true) system state
    State x;
    x.setZero();
    
    // Control input
    Control u;
    // System
    SystemModel sys;
    
    // Measurement model 
    CurrentModel im;

    // Random number generation (for noise simulation)
    std::default_random_engine generator;
    generator.seed( std::chrono::system_clock::now().time_since_epoch().count() );
    std::normal_distribution<T> noise(0, 1);
    
    // Some filters for estimation
    // Pure predictor without measurement updates
    Kalman::ExtendedKalmanFilter<State> predictor;
    // Extended Kalman Filter
    Kalman::ExtendedKalmanFilter<State> ekf;
    
    // Init filters with true system state
    predictor.init(x);
    ekf.init(x);
    
    // Standard-Deviation of noise added to all state vector 
    //components during state transition
    T systemNoise = 0.1;
    // Standard-Deviation of noise added to all measurement vector 
    // components in current measurements
    T currentNoise = 0.025;
    
    // Simulate for 100 steps
    const size_t N = 10;
    for(size_t i = 1; i <= N; i++)
    {
        // Constant current discharge 1.6A_
        u.i() = 1.6 
        
        // Simulate system
        x = sys.f(x, u);
        
        // Add noise: The system is affected by noise.
        x.soc() += systemNoise*noise(generator);
        x.vt() += systemNoise*noise(generator);
        
        
        // Predict state for current time-step using the filters
        auto x_pred = predictor.predict(sys, u);
        auto x_ekf = ekf.predict(sys, u);
        
        
        // Current measurement
        {
            CurrentMeasurement current = im.i(x);
 
            // Update EKF
            x_ekf = ekf.update(om, orientation);
        }
                
        // Print to stdout as csv format
        std::cout   << x.soc() << "," << x.vt() << ","
                    << x_pred.soc() << "," << x_pred.vt() << ","
                    << x_ekf.soc() << "," << x_ekf.vt() << ","
                    << std::endl;
    }
    
    return 0;
}
