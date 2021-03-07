package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.GenericCommand;
import frc.robot.commands.LimelightAlign;
import frc.robot.genericrobot.GenericRobot;
import edu.wpi.first.wpilibj.controller.PIDController;

public class PlanA extends GenericAutonomous {

    //change speed depending on robot!! (CaMOElot = .4, TestBot = .3)
    double defaultSpeed = 0.1;

    static double startingYaw = 0.0;
    static double startingDistance = 0.0;
    double correction;
    static double currentYaw = 0;
    double outerArcLength = 80; //89.2
    double innerArc = 35.45;
    double outerRadius = 50;
    double yawDifference = 0;
    long startingTime;
    double powerDecrement;
    int ballCount = 0;
    boolean shooting = false;
    double escalatorPower;
    double indexerPower;
    long alignWait = 1000;
    GenericCommand activeCommand = new LimelightAlign( -0.5, .8); //planA set setPoint to -2
    CollectPowerCells getCells = new CollectPowerCells();

    @Override
    public void autonomousInit(GenericRobot robot) {
        startingTime = System.currentTimeMillis();
        autonomousStep = -1;
        getCells.begin(robot);
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        PIDController PIDSteering = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        double currentDistance = 0;
        double yawError;
        switch (autonomousStep) {
            case -1: //resets and waits
                defaultSpeed = 0.1;
                ballCount = 0;
                shooting = false;
                robot.setShooterSpeedPresetName(GenericRobot.ShooterSpeedPresetName.SHORT_RANGE);
                robot.setShooterRPMFromSpeedConst();
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
                startingTime = System.currentTimeMillis();
                autonomousStep += 1;
                break;

            case 1: //auto aligns
                if (activeCommand.isEnabled() && ((System.currentTimeMillis() - startingTime) < alignWait)) {
                    activeCommand.step(robot);

                } else {
                    robot.limelight.table.getEntry("ledMode").setNumber(1);
                    autonomousStep += 1;
                }
                break;

            case 2: //you may fire at will
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

            case 3: //PID reset for 1st (left) arc
                getCells.run(robot);
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();
                startingYaw = robot.getYaw();
                startingDistance = robot.getDistanceInchesRight();
                robot.setShooterSpeedPresetName(GenericRobot.ShooterSpeedPresetName.MID_RANGE);
                robot.setShooterRPMFromSpeedConst();
                autonomousStep += 1;
                break;

            case 4: //1st (left) arc
                getCells.run(robot);
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                correction = PIDSteering.calculate((robot.getDistanceInchesRight() - startingDistance) + outerRadius * yawDifference);
                robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                currentDistance = robot.getDistanceInchesRight();
                if (currentDistance - startingDistance > outerArcLength) {
                    autonomousStep += 1;
                }
                break;

            case 5: //PID reset for 2nd (right) arc
                getCells.run(robot);
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();
                startingDistance = robot.getDistanceInchesLeft();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 6: //2nd (right) arc
                getCells.run(robot);
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) * Math.PI / 180.0);
                correction = PIDSteering.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage((defaultSpeed * 1.5) * (1 + correction), (defaultSpeed * .75) * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance > outerArcLength) {
                    autonomousStep += 1;
                }
                break;

            case 7: //PID reset for straightaway
                defaultSpeed = 0.09;
                getCells.run(robot);
                startingDistance = robot.getDistanceInchesLeft();
                PIDSteering.reset();
                PIDSteering.enableContinuousInput(-180, 180);
                currentYaw = 0;
                autonomousStep += 1;
                break;

            case 8: //straightaway, a little bit of oscillation, may need to turn P & D - PID coefficients
                getCells.run(robot);
                correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage(defaultSpeed * (1 + correction), defaultSpeed * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                //decrescendo power

                if (currentDistance - startingDistance > 65) { //start to decrement?
                    autonomousStep += 1;

                }
                break;


            case 9: //decrement power
                getCells.run(robot);
                currentDistance = robot.getDistanceInchesLeft();
                double slowToStop = (defaultSpeed - (defaultSpeed / 25) * ((currentDistance - startingDistance) - 65)) + .05; //?
                correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage(slowToStop * (1 + correction), slowToStop * (1 - correction)); // div by 2 to debug

                if (currentDistance - startingDistance > 90) {
                    autonomousStep += 1;

                }
                break;

            case 10:
                getCells.run(robot);
                startingTime = System.currentTimeMillis();
                autonomousStep += 1;
                break;

            case 11:
                getCells.run(robot);
                long currentTime = System.currentTimeMillis();
                if ((currentTime - startingTime) > 0){
                    autonomousStep += 1;
                    break;
                }


            case 12:
                getCells.stop(robot);
                robot.driveForward(0);
                ballCount = 0;
                autonomousStep += 1;
                break;

            case 13:
                robot.limelight.table.getEntry("ledMode").setNumber(3);
                robot.limelight.table.getEntry("pipeline").setNumber(1);
                activeCommand = new LimelightAlign(-3,.8);
                activeCommand.begin(robot);
                activeCommand.setEnabled(true);
                ballCount = 0;
                startingTime = System.currentTimeMillis();
                autonomousStep += 1;
                break;

            case 14:
                if (activeCommand.isEnabled() && ((System.currentTimeMillis() - startingTime) < alignWait)) {
                    activeCommand.step(robot);

                } else {
                    robot.limelight.table.getEntry("ledMode").setNumber(1);
                    autonomousStep += 1;
                }
                break;


            case 15:

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

            case 16: //cease your autonomous
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


