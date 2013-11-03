using Sensorfusion
using Base.Test

tests = ["kalman"]

println("Running tests:")

for t in tests
    println(" * $(t)")
    include("test/$(t).jl")
end
