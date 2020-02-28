/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.*;
import frc.robot.commands.*;
import frc.robot.genericrobot.*;

import static frc.robot.Util.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

public class Robot extends TimedRobot {

    //WheelOfFortune    colorWheel   = new WheelOfFortune();
    GenericAutonomous autoProgram = new PlanA(); //Auto routine to be used?
    GenericCommand activeCommand = new LimelightAlign( -0.5, .8, .0185);
    SmartClimb getOutaDodge = new SmartClimb();
    GenericRobot robot = new Falcon();
    Joystick leftJoystick = new Joystick(0);
    XboxController xboxJoystick = new XboxController(1);
    boolean shooterOn = false;

    double deadZone = 0.10;
    long timeStart;
    //boolean escalatorSpaceCounting =false;
    long escalatorSpacing = 0;
    int ballCount = 0;
    boolean ballCollectCounted = false;
    boolean ballShootCounted = false;

    @Override
    public void robotInit() {
        System.out.println("Klaatu barada nikto");
    }

    @Override
    public void robotPeriodic() {
        robot.updateMotorPowers();
        robot.printSmartDashboard();
        autoProgram.printSmartDashboard();
        activeCommand.printSmartDashboard();
        SmartDashboard.putBoolean("Is Shooter On", shooterOn);

        //SmartDashboard.putString("Instant Color", colorWheel.getAndStoreInstantColor().toString());
        //SmartDashboard.putString("Inferred Color",  colorWheel.getInferredColor().toString());

    }

    @Override
    public void disabledPeriodic() {
        if (leftJoystick.getTriggerPressed()) {
            System.out.println("AAAAAAAA");
            robot.resetAttitude();
            robot.resetEncoders();
            robot.resetClimberTicks();
        }

        robot.setShooterPowerPercentage(0);
        robot.setCollectorPower(0);
        robot.setEscalatorPower(0);
        robot.setIndexerPower(0);
        robot.setAngleAdjusterPower(0);
        robot.spinControlPanel(0);
        robot.setClimbVerticalPower(0);
        robot.setBalancePower(0);

        if (leftJoystick.getRawButtonPressed(5)) {
            autoProgram = new PlanA();
        }
        if (leftJoystick.getRawButtonPressed(6)) {
            autoProgram = new PlanC();
        }
        if (leftJoystick.getRawButtonPressed(7)) {
            autoProgram = new PlanD();
        }
        if (leftJoystick.getRawButtonPressed(8)) {
            autoProgram = new PlanE();
        }

        robot.limelight.table.getEntry("ledMode").setNumber(0);
    }

    @Override
    public void autonomousInit() {
        autoProgram.autonomousInit(robot);
    }

    @Override
    public void autonomousPeriodic() {
        autoProgram.autonomousPeriodic(robot);
    }

    @Override
    public void teleopInit() {
        LiveWindow.setEnabled(false);
        robot.limelight.table.getEntry("ledMode").setNumber(3);
        robot.limelight.table.getEntry("pipeline").setNumber(0);
        activeCommand.begin(robot);

    }

    @Override
    public void teleopPeriodic() {
        double escalatorPower = 0.0;
        double collectorPower = 0.0;

        if (leftJoystick.getRawButtonPressed(8)) { //INFORM 3 and 4 to jerk sideways
            activeCommand.setEnabled(false);
        }

        if (leftJoystick.getRawButtonReleased(2)){
            activeCommand.setEnabled(false);
        }


        if (activeCommand.isEnabled()) {
            activeCommand.step(robot);
            if (activeCommand.locksControls()) return;
        }

        double leftPower = -leftJoystick.getY() + leftJoystick.getX();
        double rightPower = -leftJoystick.getY() - leftJoystick.getX();


        double driverRestriction = 0.75;

        leftPower = driverRestriction * deadzoneValue(leftPower, deadZone);
        rightPower = driverRestriction * deadzoneValue(rightPower, deadZone);

        if (leftJoystick.getRawButton(3)) { //nudge left
            leftPower = -0.2;
            rightPower = 0.2;
        } else if (leftJoystick.getRawButtonReleased(3)) { //nudge left release
            leftPower = 0.0;
            rightPower = 0.0;
        }

        if (leftJoystick.getRawButton(4)) { //nudge right
            leftPower = 0.2;
            rightPower = -0.2;
        } else if (leftJoystick.getRawButtonReleased(4)) { //nudge right release
            leftPower = 0.0;
            rightPower = 0.0;
        }

        robot.setMotorPowerPercentage(leftPower, rightPower);

        if (leftJoystick.getRawButtonPressed(2)) {
            activeCommand.setEnabled(true);
        }

        //Collector
        if (xboxJoystick.getTriggerAxis(GenericHID.Hand.kRight) > 0) {
            escalatorPower = 0.0;
            if (robot.getEscalatorSensorMedium() && (ballCount<=3)) {
                timeStart = System.currentTimeMillis();
                escalatorPower = 0.5;
            } else {
                if ((System.currentTimeMillis() >= timeStart + escalatorSpacing)) {
                    escalatorPower = 0.0;
                } else {
                    escalatorPower = 0.5;
                }
            }
            if (ballCount<=4){collectorPower = 1.0;}
            if ((ballCount == 4) && !(robot.getEscalatorSensorLow())) { collectorPower = 0.0;}
            robot.collectorIn(collectorPower);
            robot.escalatorUp(escalatorPower);
        } else if (xboxJoystick.getTriggerAxis(GenericHID.Hand.kLeft) > 0) {
            robot.collectorOut(1.0);
        } else {
            robot.setCollectorPower(0);
        }

        //Escalator
        if (xboxJoystick.getXButton()) {
            robot.escalatorUp(.5);
        } else if (xboxJoystick.getAButton()) {
            robot.escalatorDown(.5);
        } else if (!(xboxJoystick.getTriggerAxis(GenericHID.Hand.kRight) > 0)) {
            robot.setEscalatorPower(0);
        }


        //Indexer
        if (xboxJoystick.getBumper(GenericHID.Hand.kRight)) {
            robot.indexerLoad(1.0);
        } else if (xboxJoystick.getBumper(GenericHID.Hand.kLeft)) {
            robot.indexerUnload(1.0);
        } else {
            robot.setIndexerPower(0);
        }


        POVDirection xboxDPadDirection = POVDirection.getDirection(xboxJoystick.getPOV());

        switch (xboxDPadDirection) {
            case NORTH: //high velocity (long range)
                robot.setShooterSpeedPresetName(GenericRobot.ShooterSpeedPresetName.LONG_RANGE);
                break;

            case SOUTH: //low velocity (short range)
                robot.setShooterSpeedPresetName(GenericRobot.ShooterSpeedPresetName.SHORT_RANGE);
                break;

            case EAST: //medium velocity (mid range)
                robot.setShooterSpeedPresetName(GenericRobot.ShooterSpeedPresetName.MID_RANGE);
                break;

            case WEST: //YEET
                robot.setShooterSpeedPresetName(GenericRobot.ShooterSpeedPresetName.YEET);
                break;

        }

        //Shooter
        if (xboxJoystick.getYButtonPressed()) {
            shooterOn = true;
        } else if (xboxJoystick.getBButtonPressed()) {
            shooterOn = false;
        }

        if (shooterOn) {
            robot.setShooterRPMFromSpeedConst();
        } else {
            robot.setShooterPowerPercentage(0);
        }

        //Climb vert
        if (getOutaDodge.getHolding()) {
            getOutaDodge.hold(robot);
        } else {
            robot.setClimbVerticalPower(0);
        }

        if (leftJoystick.getRawButtonPressed(7)) {
            getOutaDodge.begin(robot);
        }
        if (leftJoystick.getRawButtonReleased(7)) {
            getOutaDodge.setHolding(true);
        }
        if (leftJoystick.getRawButton(7)) {
            getOutaDodge.step(robot);
        }
        if (leftJoystick.getRawButton(13)) {
            robot.raiseClimberArms(.2);
            getOutaDodge.setHolding(false);
        }

        //Left individual control
        if (leftJoystick.getRawButton(5)) {
            getOutaDodge.setHolding(false);
            robot.setClimbVerticalPortPower(.6);
        } else if (leftJoystick.getRawButton(10)) {
            getOutaDodge.setHolding(false);
            robot.setClimbVerticalPortPower(-.2);
        }

        //Right individual control
        if (leftJoystick.getRawButton(11)) {
            getOutaDodge.setHolding(false);
            robot.setClimbVerticalStarboardPower(.6);
        } else if (leftJoystick.getRawButton(16)) {
            getOutaDodge.setHolding(false);
            robot.setClimbVerticalStarboardPower(-.2);
        }


        //below is code for keeping track of number of balls on robot
        if(leftJoystick.getRawButtonPressed(12)){ // resets ball count to zero
            ballCount = 0;
        }
        if(robot.getEscalatorSensorLow()){
            if (!ballCollectCounted) { //add ball
                ballCount++;
                ballCollectCounted = true;
            }
        }
        else {
            ballCollectCounted = false;
        }

        if(robot.getEscalatorSensorHigh()){
            ballShootCounted = true;
        }
        else {
            if (ballShootCounted){
                ballCount--;              // subtract ball
            }
            ballShootCounted = false;
        }
        SmartDashboard.putNumber("Ball Count", ballCount);
    }

    @Override
    public void testInit() {
        LiveWindow.setEnabled(false);
        getOutaDodge.setHolding(false);
    }


    @Override
    public void testPeriodic() {
        double escalatorPower;
        double collectorPower;

        LiveWindow.setEnabled(false);

        double leftPower = -leftJoystick.getY() + leftJoystick.getX();
        double rightPower = -leftJoystick.getY() - leftJoystick.getX();

        leftPower = deadzoneValue(leftPower, deadZone);
        rightPower = deadzoneValue(rightPower, deadZone);

        robot.setMotorPowerPercentage(leftPower, rightPower);

        //Collector
        if (leftJoystick.getRawButton(11)) {
            robot.collectorIn(1.0);
        } else if (leftJoystick.getRawButton(16)) {
            robot.collectorOut(1.0);
        } else {
            robot.setCollectorPower(0);
        }

        //Escalator
        if (leftJoystick.getRawButton(12)) {
            //do not do anything unless the medium sensor is tripped
            escalatorPower = 0.0;
            if (robot.getEscalatorSensorMedium()) {
                timeStart = System.currentTimeMillis();
                escalatorPower = 0.5;
            } else {
                if ((System.currentTimeMillis() >= timeStart + escalatorSpacing)) {
                    escalatorPower = 0.0;
                } else {
                    escalatorPower = 0.5;
                }
            }
            robot.escalatorUp(escalatorPower);
        } else if (leftJoystick.getRawButton(15)) {
            robot.escalatorDown(.5);
        } else {
            robot.setEscalatorPower(0);
        }

        //Shooter
        if (leftJoystick.getRawButton(13)) {
            robot.setShooterRPM(3500, 2500);
        } else if (leftJoystick.getRawButton(14)) {
            robot.setShooterPowerPercentage(0);
        }

        //Indexer
        if (leftJoystick.getRawButton(7) && robot.readyToShoot()) {
            robot.indexerLoad(1.0);
        } else if (leftJoystick.getRawButton(8)) {
            robot.indexerUnload(1.0);
        } else {
            robot.setIndexerPower(0);
        }

        //Vert Adjust
        if (leftJoystick.getRawButton(6)) {
            robot.aimUp(.4);
        } else if (leftJoystick.getRawButton(9)) {
            robot.aimDown(.4);
        } else {
            robot.setAngleAdjusterPower(0);
        }

        //CP
        if (leftJoystick.getRawButton(5)) {
            robot.spinControlPanel(-.2);
        } else if (leftJoystick.getRawButton(10)) {
            robot.spinControlPanel(.2);
        } else {
            robot.spinControlPanel(0);
        }

        //Climber
        if (leftJoystick.getRawButton(2)) {
            robot.setClimbVerticalPower(.6);
        } else if (leftJoystick.getRawButton(1)) {
            robot.setClimbVerticalPower(-.2);
        } else {
            robot.setClimbVerticalPower(0.0);
        }

        /* conflicting buttons
        //climb horiz
        if (leftJoystick.getRawButton( 3)) {
            robot.climberBalanceLeft(-.2);
        } else if (leftJoystick.getRawButton( 4)) {
            robot.climberBalanceRight(.2);
        } else {
            robot.setBalancePower(0);
        }
         */

        //Left individual control
        if (leftJoystick.getRawButton(3)) {
            robot.setClimbVerticalStarboardPower(0);
        }
        //Right individual control
        if (leftJoystick.getRawButton(4)) {
            robot.setClimbVerticalPortPower(0);
        }

    }

    public enum POVDirection {
        NORTH(0),
        NORTHEAST(45),
        EAST(90),
        SOUTHEAST(135),
        SOUTH(180),
        SOUTHWEST(225),
        WEST(270), //best
        NORTHWEST(315),
        NULL(-1);

        private final int angle;

        POVDirection(int angle) {
            this.angle = angle;
        }

        public int getAngle() {
            return angle;
        }

        //Kevin voodoo to turn ints into directions
        public static final Map<Integer, POVDirection> directionMap =
                Arrays.stream(POVDirection.values()).collect(
                        Collectors.toMap(
                                POVDirection::getAngle,
                                Function.identity()
                        )
                );

        public static POVDirection getDirection(int angle) {
            return directionMap.getOrDefault(angle, POVDirection.NULL);
        }
    }

}