# Parameter name
string name

# Value of parameter
float64 value

# Min/max constraints during optimization
float64 min
float64 max

# Uncertainty of measurement in same units as value
# (if measurement uncertainty is +/- 5 inches, put 5 here)
float64 uncertainty

# Amount to scale value during optimization. Set this value
# to 1 if the units are original.
# This parameter is needed because the optimizer works best when
# all of the input parameters require adjustments that are approximately
# the same order of magnitude, but we need to know how to scale the
# parameter back to its default units when using it in the optimizer
float64 scaling
