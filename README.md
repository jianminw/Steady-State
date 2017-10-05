# Steady-State
A few scripts looking at steady state behaviour of the vehicle, which hopefully captures the upper bound of tire forces the car should expreience. 

# Files
## LoadShifts.m
Given maximum acceleration, braking, and cornering g forces, this code goes through the g-g diagram and finds the point of maximum load shift onto one tire. (front outer for braking/cornering and rear outer for acceleration/cornering)

## SteadyStateCornering.m
This script starts at 0 velocity, and slowly increases the velocity until the tires are no longer able to put out enough force to keep the car turning with the turn radius provides. 
This script is based off chapter 5 of Race Car Vehicle Dynamics

## StraightLineAcceleration.m
This script attempts to simulate the acceleration event at competition. The vehicle starts at rest at negative of the runup. The time measured is the time between the vehicle passing 0 and the vehicle passing the distance. 

## StraightLineBraking.m
The vehicle starts at a specified velocity, and brakes at the limit of the tires until it is at rest. The only data outputed are tire forces. 

# The TireOutputs Function
This function is found in all three of steady state simulations. This function is a lookup table. The Pacejka values are linearly interpolated (or exterpolated, depending on input values), and the MAXIMUM possible forces and moments are returned. 
