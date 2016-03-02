using MAEBOL
using Base.Test

# write your own tests here
@test 1 == 1

# Testing the coefficients
m = SearchDomain(9)
@assert hk(m, 0) == 9.0
@assert hk(m, 1) == 4.5
@assert hk(m, 2) == 4.5
@assert hk(m, 3) == 4.5
