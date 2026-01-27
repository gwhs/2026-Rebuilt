# Runbook for quick fixes on game day

To help the team adapt to game day problems, we create this file to help the pit team make changes quickly, in a tested way.

Each Runbook entry should include these elements


```
## (Title of entry)
Short description of the senario
### Diagnose
Explain when you need to follow the runbook, based on symptoms or events
### What to do
* do this
* then do this
* and check this
* and then test
* mention class and file names if possible
```


## Replacing the roborio
When you need to replace the roborio, you will need to change variuos things in the code

###  Diagnose 
When you replace the roborio on a robot, you need to update some code.
The roborio has a serial number that we use to configure the drivetrain and other things.  If you don't update the setup, incorrect drivetrain setting may be used.   
### What to do
* find the new serial numbers by running the robot, and seeing the output. There will be an alert `roborio unrecognized. here is the serial number:` with the roborio's number. If no alert, then the serial number will be one of the existing serial numbers.
* update the robot detection code to use the new serial number in `RobotContainer.getRobot()` 
* Test the robot gets created with all the correct setup.



## Disabling a subsystem quickly
 When a subsystem needs to be removed because it is non-functiona, the code needs to change.
### Diagnose
A subsystem is broken, and you don't want to rewrite RobotContainer.
A non functional subsystem cannot be commented out, as this will cause compile errors.  
### What to do
* identify the subsystem to remove.
* in the subsystem constructor, find the place the SubsystemIO class is created
* replace the constructor of `SystemIOReal` with `SystemIODisabled`
* if there is no Disabled version, copy the `SystemIOSim` class and remove all the simulation code and logic

