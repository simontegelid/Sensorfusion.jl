module KalmanFilter

using Calculus

function predict_int!(statevec, xhat_next::Vector, F::Matrix, Q::Matrix)
    statevec.P = F*statevec.P*F' + Q
    statevec.xhat = xhat_next
end

function predict!(statevec, F::Matrix, Q::Matrix)
    predict_int!(statevec, F*statevec.xhat, F, Q)
end

function predict!(statevec, f::Function, fp::Function, Q::Matrix)
    predict_int!(statevec, f(statevec.xhat), fp(statevec.xhat), Q)
end

function predict!(statevec, f::Function, Q::Matrix)
    predict_int!(statevec, f(statevec.xhat), jacobian(f)(statevec.xhat), Q)
end

function update_int!(statevec, y::Vector, yhat::Vector, H::Matrix, R::Matrix)
    e = y - yhat
    S = H*statevec.P*H' + R
    K = statevec.P*H'/S
    statevec.P = statevec.P - K*H*statevec.P
    statevec.xhat = statevec.xhat + K*e
end

function update!(statevec, y::Vector, H::Matrix, R::Matrix)
    update_int!(statevec, y, H*statevec.xhat, H, R)
end

function update!(statevec, y::Vector, h::Function, hp::Function, R::Matrix)
    update_int!(statevec, y, h(statevec.xhat), hp(statevec.xhat), R)
end

function update!(statevec, y::Vector, h::Function, R::Matrix)
    update_int!(statevec, y, h(statevec.xhat), jacobian(h)(statevec.xhat), R)
end

end