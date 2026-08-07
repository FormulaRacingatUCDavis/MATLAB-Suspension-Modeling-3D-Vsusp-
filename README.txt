Welcome to the GitHub repository for MATLAB Suspension Modeling for FRUCD! Add this folder and its subfolders to your path in MATLAB to read and use its contents. The folder is divided into four subfolders:

	Car Parameters contain data structures with front and rear hardpoints, along with other information like spring rates. These Car Parameter files 	provide initial static conditions for the vehicle and define its suspension links.

	Object Files define a class structure that stores and calculates information about the car. They use the basic hardpoint information from Car 	Parameters, and run additional calculations to tell us more about the car. They also enable movement of our simulated suspension links by defining 	relationships between our hardpoints.

	Analysis programs define test routines. They direct an object file to use its internal functions in response to a range of input. Often they 	sweep through the range, calculate a desired parameter, and display the relationship between the two.

	Results from analysis programs and useful data from actual track sessions will be stored in the data folder. Any data calculated internally will 	come from analysis programs. The data may given as numbers, or instances of object files that correspond to a specific input.

For further questions, please contact FRUCD suspension subteam co-lead Jonah Grancell (jsgrancell@ucdavis.edu)