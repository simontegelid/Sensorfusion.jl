#
# KF
#

s = Statevector([0,0,0],eye(3))
KalmanFilter.update!(s, [1,1,1], eye(3), eye(3))
@test s.xhat == [.5,.5,.5]
@test s.P == .5*eye(3)
KalmanFilter.predict!(s, [1 1 0;0 1 1;0 0 1], .5*eye(3))
@test s.xhat == [1,1,.5]

#
# EKF
#

s = Statevector([0,0,0],eye(3))
KalmanFilter.update!(s, [1,1,1], x -> [x[1],x[2],x[3]], x -> eye(3), eye(3))
@test s.xhat == [.5,.5,.5]
@test s.P == .5*eye(3)
KalmanFilter.predict!(s, x -> [1,1,.5], x -> [1 1 0;0 1 1;0 0 1], .5*eye(3))
@test s.xhat == [1,1,.5]

#
# EKF, numeric Jacobian
#

s = Statevector([0.,0.,0.],eye(3))
KalmanFilter.update!(s, [1.,1.,1.], x -> [x[1],x[2],x[3]], eye(3))
@test s.xhat == [.5,.5,.5]
@test s.P == .5*eye(3)
KalmanFilter.predict!(s, x -> [1,1,.5], .5*eye(3))
@test s.xhat == [1,1,.5]

