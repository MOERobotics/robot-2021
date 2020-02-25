package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.GenericCommand;
import frc.robot.commands.LimelightAlign;
import frc.robot.genericrobot.GenericRobot;
import edu.wpi.first.wpilibj.controller.PIDController;

public class PlanE extends GenericAutonomous {

    //change speed depending on robot!! (CaMOElot = .4, TestBot = .3)
    double defaultSpeed = 0.2; //CHANGE WHEN DONE

    static double startingYaw = 0.0;
    static double startingDistance = 0.0;
    double correction;
    static double currentYaw = 0;
    double outerArcLength = 39;
    double innerArc = 35.45;
    double outerRadius = 32;
    double yawDifference = 0;
    double startingAngle = 20;
    long startingTime;
    double powerDecrement;
    double currentDistance;
    double escalatorPower;
    double indexerPower;
    boolean shooting = false;
    int ballCount = 0;
    double shooterUpperRPMNear = 2210;
    double shooterLowerRPMNear = 2210;
    double shooterUpperRPMFar = 2430; //PlanA: 2210
    double shooterLowerRPMFar = 2430; //PlanA: 2210
    GenericCommand activeCommand = new LimelightAlign(0.0, 0.9, 0.0165);
    CollectPowerCells getCells = new CollectPowerCells();

    @Override
    protected void printSmartDashboardInternal() {
        SmartDashboard.putNumber("Current Distance", currentDistance);
        SmartDashboard.putNumber("Starting Distance", startingDistance);
        SmartDashboard.putNumber("distanceDifference", currentDistance - startingDistance);
    }

    @Override
    public void autonomousInit(GenericRobot robot) {
        startingTime = System.currentTimeMillis();
        autonomousStep = -1;
        getCells.begin(robot);
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        PIDController PIDSteering = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());

        currentDistance = 0;
        double yawError;
        switch (autonomousStep) {
            case -1:
                ballCount = 0;
                shooting = false;
                robot.setShooterRPM(shooterUpperRPMNear, shooterLowerRPMNear);
                robot.resetAttitude();
                robot.resetEncoders();
                if (System.currentTimeMillis() >= startingTime + 100) {
                    autonomousStep += 1;
                }
                break;

            case 0: //turns on LEDs
                robot.limelight.table.getEntry("ledMode").setNumber(3);
                robot.limelight.table.getEntry("pipeline").setNumber(0);

                activeCommand.begin(robot);
                activeCommand.setEnabled(true);
                autonomousStep += 1;
                break;

            case 1: //auto aligns
            if (activeCommand.isEnabled()) {
                activeCommand.step(robot);

            } else {
                robot.limelight.table.getEntry("ledMode").setNumber(1);
                autonomousStep += 1;
            }
            break;

            case 2: // shoots
                if(robot.readyToShoot()){
                    escalatorPower = 0.5;
                    indexerPower = 1.0;
                } else {
                    escalatorPower = 0.0;
                    indexerPower = 0.0;
                }
                if(robot.getEscalatorSensorHigh() == true){
                    shooting = true;
                }
                if ((shooting) && (robot.getEscalatorSensorHigh() == false)) {
                    shooting = false;
                    ballCount++;
                }
                if (ballCount == 3) {
                    escalatorPower = 0;
                    indexerPower = 0;
                    autonomousStep += 1;
                }
                robot.escalatorUp(escalatorPower);
                robot.indexerLoad(indexerPower);
                break;

            case 3://skrts back to zero (account for 20 yaw offset)
                getCells.run(robot);
                currentYaw = robot.getYaw();
                robot.setMotorPowerPercentage(.2, -.2);
                if (currentYaw > startingAngle-7) {
                    robot.driveForward(0);
                    startingYaw = startingAngle;
                    autonomousStep += 1;

                }
                break;

            case 4: //PID reset for 1st (left) arc
                getCells.run(robot);
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();
                startingYaw = robot.getYaw();
                startingDistance = robot.getDistanceInchesRight();

                autonomousStep += 1;
                break;

            case 5: //1st (left) arc
                getCells.run(robot);
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                correction = PIDSteering.calculate((robot.getDistanceInchesRight() - startingDistance) + outerRadius * yawDifference);
                robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                currentDistance = robot.getDistanceInchesRight();

                if (currentDistance - startingDistance > outerArcLength) {
                    autonomousStep += 1;
                }
                break;

            case 6: //PID reset for 2nd (right) arc
                getCells.run(robot);
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();
                startingDistance = robot.getDistanceInchesLeft();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;


            case 7: //2nd (right) arc
                getCells.run(robot);
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) * Math.PI / 180.0);
                correction = PIDSteering.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage((defaultSpeed * 1.5) * (1 + correction), (defaultSpeed * .75) * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();

                if (currentDistance - startingDistance > outerArcLength) {
                    autonomousStep += 1;
                }
                break;

            case 8: //PID reset for straightaway
                getCells.run(robot);
                startingDistance = robot.getDistanceInchesLeft();
                PIDSteering.reset();
                PIDSteering.enableContinuousInput(-180, 180);
                currentYaw = startingAngle;
                autonomousStep += 1;
                break;

            case 9:
                //trench run (~200in)
                getCells.run(robot);
                correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage(1.5 * defaultSpeed * (1 + correction), 1.5 * defaultSpeed * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();

                if (currentDistance - startingDistance > 95) {
                    autonomousStep += 1;
                }
                break;

            case 10: // continues collection and starts timer
                getCells.run(robot);
                startingTime = System.currentTimeMillis();
                autonomousStep += 1;
                break;

            case 11: // continues collection fro 2 seconds
                getCells.run(robot);
                long currentTime = System.currentTimeMillis();
                if ((currentTime - startingTime) > 2000){
                    autonomousStep += 1;
                    break;
                }

            case 12: // stops collection and stop robot motion
                getCells.stop(robot);
                robot.driveForward(0);
                autonomousStep += 1;
                break;

            case 13: // align
                robot.limelight.table.getEntry("ledMode").setNumber(3);
                robot.limelight.table.getEntry("pipeline").setNumber(1);
                activeCommand.begin(robot);
                activeCommand.setEnabled(true);
                ballCount = 0;
                autonomousStep += 1;
                break;

            case 14: // shoot
                if(robot.readyToShoot()){
                    escalatorPower = 0.5;
                    indexerPower = 1.0;
                } else {
                    escalatorPower = 0.0;
                    indexerPower = 0.0;
                }
                if(robot.getEscalatorSensorHigh() == true){
                    shooting = true;
                }
                if ((shooting) && (robot.getEscalatorSensorHigh() == false)) {
                    shooting = false;
                    ballCount++;
                }
                if (ballCount == 3) {
                    escalatorPower = 0;
                    indexerPower = 0;
                    autonomousStep += 1;
                }
                robot.escalatorUp(escalatorPower);
                robot.indexerLoad(indexerPower);
                break;

            case 15: //cease your autonomous
                robot.setShooterPowerPercentage(0);
                if (activeCommand.isEnabled()) {
                    activeCommand.step(robot);

                } else {
                    robot.limelight.table.getEntry("ledMode").setNumber(1);
                    robot.driveForward(0);
                }
                break;
        }
    }
}


