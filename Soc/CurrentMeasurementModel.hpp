#ifndef KALMAN_EXAMPLES_ROBOT1_CURRENTMEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_ROBOT1_CURRENTMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>

namespace KalmanExamples
{
namespace Soc
{

/**
 * @brief Measurement vector measuring a current using INA219 sensor
 *
 * @param T Numeric scalar type
 */
template<typename T>
class CurrentMeasurement : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(CurrentMeasurement, T, 1)
    
    //! Current
    static constexpr size_t I = 0;
    T i()       const { return (*this)[ I ]; }
    
    T& i()      { return (*this)[ I ]; }
};

/**
 * @brief Measurement model for measuring orientation of a 3DOF robot
 *
 * This is the measurement model for measuring the orientation of our
 * planar robot. This could be realized by a compass / magnetometer-sensor.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class CurrentMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, CurrentMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef KalmanExamples::Soc::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  KalmanExamples::Soc::CurrentMeasurement<T> M;
    
    CurrentMeasurementModel()
    {
        // Setup jacobians. As these are static, we can define them once
        // and do not need to update them dynamically
        this->H.setIdentity();
        this->V.setIdentity();
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     * i = Soc*C_atual
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;
        // Measurement is given by the actual current
        measurement.i() = x.soc()*x.C_actual;
        
        return measurement;
    }
};

} // namespace Soc
} // namespace KalmanExamples

#endif