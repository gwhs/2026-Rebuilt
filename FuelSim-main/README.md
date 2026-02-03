# FuelSim
This single-file mini-library is a lightweight fuel physics simulation. It handles fuel-fuel, fuel-field, fuel-net, and fuel-robot collisions as well as projectile motion and scoring in the hub. There is also included functionality for intakes. It's not a perfect analog of real life, as some of the parameters need to be tuned (namely the coefficients of restitution of various collisions), but it is close enough for simulation purposes. I built this using AdvantageKit's built-in logging, but it can easily be modified to use any logging method by swapping out a single line of code.
## Demo
https://github.com/user-attachments/assets/3b0a20c8-bec2-441b-a4a5-fdaee0f9cb8c

## Usage
### Installation
Simply copy over FuelSim.java into your project!
### Setup
All measurements are in meters. Logging is done automatically when updateSim is called.
In Robot.java:
```java
@Override
public void simulationPeriodic() {
    FuelSim.getInstance().updateSim();
}
```
In RobotContainer.java (or elsewhere):
```java
FuelSim.getInstance(); // gets singleton instance of FuelSim
FuelSim.getInstance().spawnStartingFuel(); // spawns fuel in the depots and neutral zone

// Register a robot for collision with fuel
FuelSim.getInstance().registerRobot(
        width, // from left to right
        length, // from front to back
        bumperHeight, // from floor to top of bumpers
        poseSupplier, // Supplier<Pose2d> of robot pose
        fieldSpeedsSupplier); // Supplier<ChassisSpeeds> of field-centric chassis speeds

// Register an intake to remove fuel from the field as a rectangular bounding box
FuelSim.getInstance().registerIntake(
        minX, maxX, minY, maxY, // robot-centric coordinates for bounding box
        shouldIntakeSupplier, // (optional) BooleanSupplier for whether the intake should be active at a given moment
        callback); // (optional) Runnable called whenever a fuel is intaked

FuelSim.getInstance().setSubticks(int subticks); // sets the number of physics iterations to perform per 20ms loop. Default = 5

FuelSim.getInstance().start(); // enables the simulation to run (updateSim must still be called periodically)
FuelSim.getInstance().stop(); // stops the simulation running (updateSim will do nothing until start is called again)
```
### While Running
```java
FuelSim.getInstance().stepSim(); // steps the simulation forward by 20ms, regardless of start/stop state
FuelSim.getInstance().spawnFuel(Translation3d pos, Translation3d vel); // spawns a fuel with a given position and velocity (both field centric, represented as vectors by Translation3d)
FuelSim.getInstance().clearFuel(); // clears all fuel from the field

FuelSim.Hub.BLUE_HUB.getScore(); // get number of fuel scored in blue hub
FuelSim.Hub.RED_HUB.getScore(); // get number of fuel scored in red hub
FuelSim.Hub.[BLUE/RED]_HUB.resetScore(); // resets the score of the blue/red hub

```
## Example Usage (as seen in demo)
In RobotContainer.java (to be called in constructor):
```java
private void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    instance.spawnStartingFuel();
    instance.registerRobot(
            Dimensions.FULL_WIDTH.in(Meters),
            Dimensions.FULL_LENGTH.in(Meters),
            Dimensions.BUMPER_HEIGHT.in(Meters),
            drive::getPose,
            drive::getFieldSpeeds);
    instance.registerIntake(
            -Dimensions.FULL_LENGTH.div(2).in(Meters),
            Dimensions.FULL_LENGTH.div(2).in(Meters),
            -Dimensions.FULL_WIDTH.div(2).plus(Inches.of(7)).in(Meters),
            -Dimensions.FULL_WIDTH.div(2).in(Meters),
            () -> intake.isRightDeployed() && turret.simAbleToIntake(),
            turret::simIntake);
    instance.registerIntake(
            -Dimensions.FULL_LENGTH.div(2).in(Meters),
            Dimensions.FULL_LENGTH.div(2).in(Meters),
            Dimensions.FULL_WIDTH.div(2).in(Meters),
            Dimensions.FULL_WIDTH.div(2).plus(Inches.of(7)).in(Meters),
            () -> intake.isLeftDeployed() && turret.simAbleToIntake(),
            turret::simIntake);

    instance.start();
    SmartDashboard.putData(Commands.runOnce(() -> {
                FuelSim.getInstance().clearFuel();
                FuelSim.getInstance().spawnStartingFuel();
            })
            .withName("Reset Fuel")
            .ignoringDisable(true));
}
```

In TurretVisualizer.java (called repeatedly):
```java
public void launchFuel(LinearVelocity vel, Angle angle) {
    if (fuelStored == 0) return;
    fuelStored--;
    Pose3d robot = poseSupplier.get();

    Translation3d initialPosition = robot.getTranslation();
    FuelSim.getInstance().spawnFuel(initialPosition, launchVel(vel, angle));
}
```
