using MAEBOL
using Base.Test

# write your own tests here
@test 1 == 1

# Testing the coefficients
m = SearchDomain(9)
@assert h_ij(0, 0, 9) == 9.0
@assert h_ij(1, 1, 9) == 4.5
@assert h_ij(2, 2, 9) == 4.5
@assert h_ij(3, 3, 9) == 4.5
