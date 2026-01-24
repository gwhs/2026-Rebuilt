# Runbook for quick fixes on game day

To help the team adapt to game day problems, we create this file to help the pit team make changes quickly, in a tested way.

Each Runbook entry should include these elements

Title
Short description of the senario
How to diagnose
What to do, inlcuding files 


## (Title)
 (description)
### Diagnose

### What to do

Example

## Replace the roborio
When you need to replace the roborio, you will need to change variuos things in the code

###  Diagnose 
The roborio has a serial number that we use to configure the drivetrain and other things.  If you don't update the setup, incorrect drivetrain setting may be used.   
### What to do
* find the new serial numbers by running the robot, and seeing the output. There will be an alert `roborio unrecognized. here is the serial number:` with the roborio's number. If no alert, then the serial number will be one of the existing serial numbers.
* update the robot detection code to use the new serial number in `RobotContainer.getRobot()` 
* Test the robot gets created with all the correct setup.


