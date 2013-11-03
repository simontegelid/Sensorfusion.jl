type Statevector
    xhat::Vector
    P::Matrix

    function Statevector(xhat::Vector, P::Matrix)
        m = size(xhat)[1]
        n,o = size(P)

        m==n==o ? new(xhat, P) : error("P must be NxN and xhat Nx1")

    end
end

Statevector(xhat::Matrix, P::Matrix) = Statevector(xhat[:], P)