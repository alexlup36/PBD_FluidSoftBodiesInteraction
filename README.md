# PBD_FluidSoftBodiesInteraction  
  
C++ implementation of a particle based framework for simulating real-time interactions between fluids and soft-bodies.
The simulation is based on position based dynamics and implements two constraints: density constraint to satisfy incompressibility
for the fluid simulation and shape matching to simulate deformations.  
  
SFML - graphics  
Boost - multithreading  
GLM - mathematics  
Visual Studio 2013  
  
Press F and then left click to add more particles  
Press S and then left click to add a soft body. To start updating the new soft body use right click  
Softbodies can be dragged around using the mouse  
The velocity damping and viscosity of the fluid can be modified using the mouse scroll (Use the arrow keys to select a value first)  
  
Build located in build.zip  
