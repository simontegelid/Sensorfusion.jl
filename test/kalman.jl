using LinearAlgebra

#
# KF
#

s = Statevector([0, 0, 0], Matrix(1.0I, 3, 3))
KalmanFilter.update!(s, [1, 1, 1], Matrix(1.0I, 3, 3), Matrix(1.0I, 3, 3))
@test s.xhat == [.5, .5, .5]
@test s.P == Matrix(0.5I, 3, 3)
KalmanFilter.predict!(s, [1 1 0;0 1 1;0 0 1], Matrix(0.5I, 3, 3))
@test s.xhat == [1, 1, .5]

#
# EKF
#

s = Statevector([0, 0, 0], Matrix(1.0I, 3, 3))
KalmanFilter.update!(s, [1, 1, 1], x -> [x[1], x[2], x[3]], x -> Matrix(1.0I, 3, 3), Matrix(1.0I, 3, 3))
@test s.xhat == [.5, .5, .5]
@test s.P == Matrix(0.5I, 3, 3)
KalmanFilter.predict!(s, x -> [1, 1, .5], x -> [1 1 0;0 1 1;0 0 1], Matrix(0.5I, 3, 3))
@test s.xhat == [1, 1, .5]

#
# EKF, numeric Jacobian
#

s = Statevector([0., 0., 0.], Matrix(1.0I, 3, 3))
KalmanFilter.update!(s, [1., 1., 1.], x -> [x[1], x[2], x[3]], Matrix(1.0I, 3, 3))
@test s.xhat == [.5, .5, .5]
@test s.P == Matrix(0.5I, 3, 3)
KalmanFilter.predict!(s, x -> [1, 1, .5], Matrix(0.5I, 3, 3))
@test s.xhat == [1, 1, .5]
