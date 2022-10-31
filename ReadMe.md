# Scooter model simulation

Explains the (WIP) simulation base of the e-scooter.

## Contents

 [CHANGELOG 2022-10-31]

 - Simulation explained

- Startup simulation
- Install Toolboxes into MATLAB
- Import CAD files into MATLAB.
- Setup the e-scooter
- Construct a plane
- Combine plane with e-scooter

### Simulation Explained

[CHANGELOG 2022-10-31]

The simulation simulates the lean angle using a sensor on the scooter body which is inputed to the control system consisting of two PID controllers.
The outer PID is determined with oKp,oKi,oKd while the inner is determined with iKp,iKi,iKd(these values can be found in Scooterparam.m).
The Steering joint is controlled via an angle position. 
Choose between ***Deltastar or Deltadot*** to determine one or two PIDs use. 
Deltastar is only calculating the wanted stering angle and sets the joint to it. 
Deltadot is using two PIDs and outputs an velocity that is integrated(inside the PID block) to give an position to the joint.  
Both methods can balance the e-scooter and is limited to +/- 1 rad in stering angle.
Compare functionality on the desired and current angle graph inside the PID block.
A small push is applied to the scooter at second 2(can be removed/changed) which is connected in scooter block-> 6-DOF joint block.

### Startup Simulation

Files needed:

- Scooterparam.m
- scootermodel\_DataFile.m
- scootermodelsim.slx
- Body\_Default\_sldprt.STEP
- Handle\_Default\_sldprt.STEP
- wheel\_Default\_sldprt.STEP

 [CHANGELOG 2022-10-26]

- wheel\_Default\_sldprt\_round.STEP
- plane.slx
- Equation_Bicycle_Model.slx

[CHANGELOG 2022-10-26]

As an initial functionality the scooter has a free moving handle with an initialized velocity with 8km/h on the forward wheel. 
The wheels has two options one with rounded edges and one with a cylindrical form that can be changed in scooter block -> wheel_RIGID block.
In scooterparam.m the paramteres are measured from a borrowed E-scooter(not the final one).
Also in scooterparam.m there exist some variables that could be changed. 

- friction variables depending on material on ground.
- damping and spring coefficients
- plane variables to determine where the scooter can be.

[CHANGELOG 2022-10-31]

- PID coefficients for inner and outer loop  oKp,oKi,oKd,iKp,iKi,iKd.

Run scooterparam.m to start simulation.

### Install Toolboxes 

The simulation is done through [MATLAB](https://se.mathworks.com/products/new_products/latest_features.html) R2022b, [Simulink](https://se.mathworks.com/products/simulink.html), [Simscape](https://se.mathworks.com/products/simscape.html) together with [Simscape Multibody](https://se.mathworks.com/products/simscape-multibody.html) dynamics toolbox.

[CHANGELOG 2022-10-31]

 [Robotics System Toolbox](https://se.mathworks.com/products/robotics.html) and  [Control System Toolbox](https://se.mathworks.com/products/control.html) .


### Import CAD files into MATLAB.

(Only needed if the model needs to be updated)

This simulation is based upon designing a e-scooter in CAD which then can be visualised in the simulation. 
Solidworks 2022 is used with the Multibody toolbox which also is compatible with PTCCreo and Autodesk Inventor.

If a new model is inserted follow this [tutorial](https://se.mathworks.com/help/smlink/ug/installing-and-linking-simmechanics-link-software.html)
which creates a link between MATLAB and your CAD program.
Be sure to download both the multibody plugin and the install addon.

***Only PTCCreo, Autodesk Inventor and Solidworks is compatible***

After that [enable Simscape Multibody Link Plugin in SolidWorks](https://se.mathworks.com/help/smlink/ref/linking-and-unlinking-simmechanics-link-software-with-solidworks.html). By doing this you can export parts from solidworks directly into the workpath in matlab. 
- Type smlink_linksw in matlab command window to link to CAD program 
-  export the .xml file along with the individual parts in your workspace folder.
- calling the smimport('scootermodel');  in matlab command window creates a Datafile.m with a simulation CAD model. 

***Check if the joints are connected correctly in simulation and in datafile.***

The scooter model had a cylindrical joint originally that was changed to a revolute joint 'Revolute2' in Simulink. Also a planar joint constrained the front wheel to the body which was removed. This demanded changes in DataFile, removed planar joint and cylindrical joint and copied a new revolute joint. The transformation or rotations should not be needing any configuration. 

Revolute joint config:

smiData.RevoluteJoint(3).Rz.Pos = 0;

smiData.RevoluteJoint(3).ID = "[Handle-1:-:wheel-2]";

### Setup the e-scooter

The scooter has a individual block that takes frame and contact points as inputs and outputs the new frame. 

- Frame input - consist of the world frame which is rotated to allign to Z-axis which is done with 
rotationangle and rotationaxis in scooterparam.m file.
- contact points comes from the plane. With each scooter part having it's individual frame connected to the plane. 
without the contact points block it would be a scooter and a plane with no correspondance such the scooter would fall straight through the plane. Both wheels have an applied friction coefficient found in scooterparam.m. 
Friction\_stat/dyn\_back/front.

The steering angle is initialized with smiData.RevoluteJoint(2).Rz.Pos=0, thus the handle is straight. 

### Construct a plane

The plane comes from a solid block in simulink. 
It is defined with xPla, yPla, zPla stated in 
scooterparam.m 
The plane forwards it to contact points block to create conditions with the e-scooter.

###  Combine plane with e-scooter

The World is defined outside of this setting the gravity constant in z axis and adding the drop height translation. 
This is combined with the plane.

Thus the Simulation has a work flow like:

- World-> plane->contactpoints->scooter
- current steering angle and tilt angle->PID->scooter
- World-> scooter
- scooter->Sense tilt angle and velocity

Future work is to implement a MATLAB gui with a block and a control system block which could be implemented in same space as the plane and scooter blocks are.