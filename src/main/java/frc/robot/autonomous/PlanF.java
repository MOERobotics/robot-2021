package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.commands.ElevationControl;
import frc.robot.commands.GenericCommand;
import frc.robot.commands.LimelightAlign;
import frc.robot.genericrobot.GenericRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PlanF extends GenericAutonomous {

    //change speed depending on robot!! (CaMOElot = .4, TestBot = .2)
    double defaultSpeed = 0.30;

    static double startingYaw = 0.0; //start at an angle, figure out later
    static double startingDistance = 0.0;
    double correction;
    static double currentYaw = 0;
    double outerArcLength = 50;
    double innerArc = 35.45;
    double innerRadius = 30;
    double outerRadius = 65;
    double yawDifference = 0;
    double prevStartingDistance = 0;
    long startingTime = System.currentTimeMillis();
    int ballCount = 0;
    boolean shooting = false;
    double escalatorPower;
    double indexerPower;
    long alignWait = 2000;
    GenericCommand activeCommand = new LimelightAlign( -0.5, .8); //planA set setPoint to -2
    CollectPowerCells getCells = new CollectPowerCells();

    ElevationControl shooterController = new ElevationControl();

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
                    robot.setMotorPowerPercentage(0,0);
                    autonomousStep += 1;
                }
                break;

            case 2: //you may fire when ready
                if (robot.readyToShoot()) {
                    escalatorPower = 0.5;
                    indexerPower = 1.0;
                } else {
                    escalatorPower = 0.0;
                    indexerPower = 0.0;
                }
                if (robot.getEscalatorSensorHigh() == true) {
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

            case 3: //PID reset for straightaway
                getCells.run(robot);
                startingDistance = robot.getDistanceInchesLeft();
                PIDSteering.reset();
                PIDSteering.enableContinuousInput(-180, 180);
                currentYaw = 0;
                autonomousStep += 1;
                break;

            case 4: //straightaway
                double speedScale = 1.2 - (0.2/104) * robot.getDistanceInchesLeft();
                speedScale = 2.5;
                getCells.run(robot);
                shooterController.begin(robot);
                shooterController.setEnabled(true);
                shooterController.setSetPoint(153);

                correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage(speedScale * defaultSpeed * (1 + correction),
                        speedScale * defaultSpeed * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance > 126) { //maybe change depending on how far we need to go
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 5: //reset for arc
                getCells.run(robot);

                startingDistance = robot.getDistanceInchesRight();
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();
                currentYaw = robot.getYaw();
                autonomousStep += 1;
                outerArcLength = 60;
                outerRadius = 56;

                break;

            case 6: // arc to pick up powercell
                yawDifference = continuousAngleDiff((robot.getYaw() - currentYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesRight();
                correction = PIDSteering.calculate((currentDistance - startingDistance) + outerRadius * yawDifference);
                robot.setMotorPowerPercentage ( (defaultSpeed * .75) + correction/3, (defaultSpeed * 1.5) - correction/3);

                if (currentDistance - startingDistance > outerArcLength) {
                    autonomousStep = +1;
                }
                break;
            case 7: //reset for arc
                getCells.run(robot);

                startingDistance = robot.getDistanceInchesRight();
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                //outerArcLength = 33;
                //outerRadius = 36.6;

                break;
            case 8: // backwards arc back to previous position
                yawDifference = -continuousAngleDiff((robot.getYaw() - currentYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesRight();
                correction = PIDSteering.calculate((-currentDistance + startingDistance) + outerRadius * yawDifference);
                robot.setMotorPowerPercentage ( (-defaultSpeed * .75) + correction/3, (-defaultSpeed * 1.5) - correction/3);

                if (currentDistance - startingDistance > outerArcLength) {
                    autonomousStep += 1;
                }
                break;


            case 9: //reset for backward straight-away
                getCells.run(robot);
                shooterController.begin(robot);
                shooterController.setEnabled(true);
                shooterController.setSetPoint(153);

                startingDistance = robot.getDistanceInchesLeft();
                PIDSteering.reset();
                PIDSteering.enableContinuousInput(-180, 180);
                currentYaw = 0;
                autonomousStep += 1;
                break;

            case 10: //backward straight-away
                getCells.run(robot);
                shooterController.begin(robot);
                shooterController.setEnabled(true);
                shooterController.setSetPoint(153);
                speedScale = 2.5;

                correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage(-1 * speedScale * defaultSpeed * (1 - correction),
                        -1 * defaultSpeed * speedScale * (1 + correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance < -126) { //maybe change depending on how far we need to go
                    robot.driveForward(0);
                    autonomousStep = 13;
                }
                break;

            case 13: // stop collecting
                getCells.stop(robot);
                robot.driveForward(0);
                ballCount = 0;
                autonomousStep += 1;
                break;

            case 14: // align
                robot.limelight.table.getEntry("ledMode").setNumber(3);
                robot.limelight.table.getEntry("pipeline").setNumber(0);
                activeCommand = new LimelightAlign(-3, .8); //fix dem bois
                activeCommand.begin(robot);
                activeCommand.setEnabled(true);
                ballCount = 0;
                startingTime = System.currentTimeMillis();
                autonomousStep += 1;
                break;

            case 15:
                if (activeCommand.isEnabled() && ((System.currentTimeMillis() - startingTime) < alignWait)) {
                    activeCommand.step(robot);

                } else {
                    robot.limelight.table.getEntry("ledMode").setNumber(1);
                    autonomousStep += 1;
                }
                break;

            case 16: // you may fire when ready
                if (robot.readyToShoot()) {
                    escalatorPower = 0.5;
                    indexerPower = 1.0;
                } else {
                    escalatorPower = 0.0;
                    indexerPower = 0.0;
                }
                if (robot.getEscalatorSensorHigh() == true) {
                    shooting = true;
                }
                if ((shooting) && (robot.getEscalatorSensorHigh() == false)) {
                    shooting = false;
                    ballCount++;
                }
                if (ballCount == 2) {
                    escalatorPower = 0;
                    indexerPower = 0;
                    autonomousStep += 1;
                }
                robot.escalatorUp(escalatorPower);
                robot.indexerLoad(indexerPower);
                break;

            case 17: //cease your autonomous
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
/*

      Position / Proportion  = How Far away we are
      Integral
      Derivative

      wheel to wheel: 23in

 */