using Sensorfusion
using Test

tests = ["kalman"]

println("Running tests:")

for t in tests
    println(" * $(t)")
    include("test/$(t).jl")
end
