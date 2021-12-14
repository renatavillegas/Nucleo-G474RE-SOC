#ifndef KALMAN_EXAMPLES2_SOC_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES2_SOC_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>
#include <math.h>  

namespace KalmanExamples
{
namespace Soc
{

/**
 * @brief System state vector-type for a Soc
 *
 * 
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(State, T, 2)
    
    //! soc - State of charge
    static constexpr size_t SOC = 0;
    //! Vt - voltage in the RC branch 
    static constexpr size_t VP = 0;
    
    T soc()       const { return (*this)[ soc ]; }
    T vp()       const { return (*this)[ vp ]; }
    
    T& soc()      { return (*this)[ soc ]; }
    T& vp()      { return (*this)[ vp ]; }
    // model params 
    float t = 1;
    float Cp = 2.2; 
    float a = 0.8; 
    float b= 3.6; 
    float Rt = 0.4; 
    float Rp = -0.09; 
    //float Cp = 715.6;
    float C_actual; 
};

/**
 * @brief System control-input vector-type for a soc 
 *
 *  The current will be the only input 
 * 
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(Control, T, 1)
    
    //! current
    static constexpr size_t I = 0;
    
    T i()       const { return (*this)[ I ]; }
    
    T& i()      { return (*this)[ I ]; }
};

/**
 * @brief System model for a simple SOC
 *
 * This is the system model defining how a battery discharges.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
	typedef KalmanExamples::Soc::State<T> S;
    
    //! Control type shortcut definition
    typedef KalmanExamples::Soc::Control<T> C;
    
    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to 
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] x The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    S f(const S& x, const C& u) const
    {
        //! Predicted state vector after transition
        S x_;
        
        // New soc
        x_.soc() = x.soc + 1/(C_actual)*u.i;

        // New vt
        x_.vt() =  a*x.soc + x.vp + Rt*u.i;

        // New x-position given by old x-position plus change in x-direction
        // Change in x-direction is given by the cosine of the (new) orientation
        // times the velocity
        //x_.x() = x.x() + std::cos( newOrientation ) * u.v();
        //x_.y() = x.y() + std::sin( newOrientation ) * u.v();
        
        // Return transitioned state vector
        return x_;
    }
    
protected:
    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize
     * @param u The current system control input
     */
    void updateJacobians( const S& x, const C& u )
    {
        // F = df/dx (Jacobian of state transition w.r.t. the state)
        this->F.setZero();
        
        // partial derivative of x.soc() w.r.t. x.soc()
        this->F( S::SOC, S::SOC ) = 1;
        // partial derivative of x.soc() w.r.t. x.vp()
        this->F( S::SOC, S::VP ) = 0;
        
        // partial derivative of x.vp() w.r.t. x.vp()
        this->F( S::VP, S::SOC ) = 0;
        // partial derivative of x.vp() w.r.t. x.theta()
        this->F( S::VP, S::VP ) = exp(-t/(Rp*Cp));
                
        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        this->W.setIdentity();
        // TODO: more sophisticated noise modelling
        //       i.e. The noise affects the the direction in which we move as 
        //       well as the velocity (i.e. the distance we move)
    }
};

} // namespace Soc
} // namespace KalmanExamples

#endif