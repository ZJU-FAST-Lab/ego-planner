#include <bspline_opt/gradient_descent_optimizer.h>


GradientDescentOptimizer::RESULT 
GradientDescentOptimizer::optimize(Eigen::VectorXd &x_init_optimal, double &opt_f)
{
    void* f_data = f_data_;
    int iter = 0;
    Eigen::VectorXd x_k( x_init_optimal ), x_kp1( x_init_optimal.rows() );
    double cost_km1, cost_k, cost_kp1, cost_min;
    Eigen::VectorXd grad_k( x_init_optimal.rows() ), grad_kp1( x_init_optimal.rows() );

    cost_k = objfun_(x_k, grad_k, f_data);
    cost_min = cost_k;
    double max_grad = max( abs(grad_k.maxCoeff()), abs(grad_k.minCoeff()) );
    constexpr double MAX_MOVEMENT_AT_FIRST_ITERATION = 0.1; // meter
    double alpha0 = max_grad < MAX_MOVEMENT_AT_FIRST_ITERATION ?
        1.0 : (MAX_MOVEMENT_AT_FIRST_ITERATION/max_grad);
    x_kp1 = x_k - alpha0 * grad_k;
    cost_kp1 = objfun_(x_kp1, grad_kp1, f_data);
    if ( cost_min > cost_kp1 ) 
        cost_min = cost_kp1;

    /*** start iteration ***/
    while( ++iter <= iter_limit_ )
    {
        Eigen::VectorXd s = x_kp1 - x_k;
        Eigen::VectorXd y = grad_kp1 - grad_k;
        double alpha = s.dot(y) / y.dot(y);

        if (iter % 2)
        {
            x_k = x_kp1 - alpha * grad_kp1;
            cost_k = objfun_(x_k, grad_k, f_data);
        }
        else
        {
            x_kp1 = x_k - alpha * grad_k;
            cost_kp1 = objfun_(x_kp1, grad_kp1, f_data);
        }

        if ( cost_k > 1e20 || cost_kp1 > 1e20 )
        {
            return RETURN_BY_ORDER;
        }

        if ( min_grad_ > 1e-10 && abs( grad_k.maxCoeff() ) < min_grad_ && abs( grad_k.minCoeff() ) < min_grad_ )
        {
            return FIND_MIN;
        }
    }

    return REACH_MAX_ITERATION;
}


