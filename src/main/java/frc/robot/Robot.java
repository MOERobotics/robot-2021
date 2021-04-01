/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.autonomous.*;
import frc.robot.commands.*;
import frc.robot.genericrobot.*;

import static frc.robot.Util.*;
import static frc.robot.genericrobot.GenericRobot.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

public class Robot extends TimedRobot {

    //WheelOfFortune    colorWheel   = new WheelOfFortune();
    GenericAutonomous autoProgram       = new GalacticSearchDecision(); //Auto routine to be used?
    GenericCommand    activeCommand     = new LimelightAlign(-2,.8);
    SmartClimb        smartClimb        = new SmartClimb();
    GenericRobot      robot             = new SiMOElator(m_ds);
    Joystick          leftJoystick      = new Joystick(0);
    XboxController    xboxJoystick      = new XboxController(1);
    ElevationControl  shooterController = new ElevationControl();
    boolean shooterOn = false;

    int dataInd='a';
    OutputStream outputStream = null;
    JsonGenerator pixyWriter = null;

    double deadZone = 0.10;
    long timeEscalatorStarted = 0;
    //boolean escalatorSpaceCounting =false;
    int ballCount = 0;
    boolean ballCollectCounted = false;
    boolean ballShootCounted = false;
    boolean waitingForMediumHigh = false;
    boolean waitingForChange = false;

    public static final Map<String, GenericAutonomous> autonomousMap
        = new HashMap<String, GenericAutonomous> () {{
            put(PlanA.class.getName(), new PlanA());
            put(PlanC.class.getName(), new PlanC());
            put(PlanD.class.getName(), new PlanD());
            put(PlanE.class.getName(), new PlanE());
    }};

    public static final LimelightAlign
        limeLightAlignCloseDistance = new LimelightAlign(-2, 0.8),
        limelightAlignFarDistance   = new LimelightAlign(-3, 0.8);
    public static       LimelightAlign
        limelightAlignChosen        = limeLightAlignCloseDistance;

    @Override
    public void robotInit() {
        System.out.println("Klaatu barada nikto");

        try{
            outputStream = new FileOutputStream(
                System.getProperty("user.home") + File.separatorChar + "pixycamData.txt"
            );

            ObjectMapper mapper = new ObjectMapper();
            mapper.configure(JsonGenerator.Feature.AUTO_CLOSE_TARGET, false);
            pixyWriter = mapper
                .writerWithDefaultPrettyPrinter()
                .with(SerializationFeature.FLUSH_AFTER_WRITE_VALUE)
                .with(SerializationFeature.WRAP_ROOT_VALUE)
                .forType(PixyCam.Block[].class)
                .createGenerator(outputStream);
        } catch (Exception e){
            System.err.println(
                "Unable to open pixycamData.txt for writing."
            );
        }
    }

    @Override
    public void robotPeriodic() {
        robot.updateMotorPowers();
        robot.printSmartDashboard();
        autoProgram.printSmartDashboard();
        activeCommand.printSmartDashboard();

        //SmartDashboard.putString("Instant Color", colorWheel.getAndStoreInstantColor().toString());
        //SmartDashboard.putString("Inferred Color",  colorWheel.getInferredColor().toString());

    }

    @Override
    public void disabledInit(){}

    @Override
    public void disabledPeriodic() {
        if (leftJoystick.getTriggerPressed()) {
            robot.resetAttitude();
            robot.resetEncoders();
            robot.resetClimberTicks();
        }

        robot.setShooterPowerPercentage(0);
        robot.setCollectorPower(0);
        robot.setEscalatorPower(0);
        robot.setIndexerPower(0);
        robot.setAimAdjusterPower(0);
        robot.spinControlPanel(0);
        robot.setClimbVerticalPower(0);
        robot.setCameraTiltDegrees(0);
        robot.resetRobotSimulation();

        if (leftJoystick.getRawButtonPressed(5)) {
            autoProgram = autonomousMap.get(PlanA.class.getName());
        }
        if (leftJoystick.getRawButtonPressed(6)) {
            autoProgram = autonomousMap.get(PlanC.class.getName());
        }
        if (leftJoystick.getRawButtonPressed(7)) {
            autoProgram = autonomousMap.get(PlanD.class.getName());
        }
        if (leftJoystick.getRawButtonPressed(8)) {
            autoProgram = autonomousMap.get(PlanE.class.getName());
        }

        robot.limelight.table.getEntry("ledMode").setNumber(0);

        if (leftJoystick.getRawButton(5)){
            //coast mode
            robot.setClimberBrake(BrakeModeState.COAST);
        }

        if (leftJoystick.getRawButtonReleased(5)){
            //brake mode
            robot.setClimberBrake(BrakeModeState.BRAKE);
        }


        if (leftJoystick.getRawButtonPressed(2) && pixyWriter != null){
            Logger.log("PixyWriter", "Writing pixycam data to file");
            PixyCam.Block[] pixycamData = robot.getPixyCamBlocks();
            try {
                outputStream.write(dataInd++);
                pixyWriter.writeObject(pixycamData);
                outputStream.write('\n');
            }
            catch (Exception e) { e.printStackTrace(); }
        }
        if (leftJoystick.getRawButtonPressed(2) && pixyWriter != null){
            Logger.log("PixyWriter", "Writing pixycam data to file");
            PixyCam.Block[] pixycamData = robot.getPixyCamBlocks();
            try {
                pixyWriter.writeObject(pixycamData);
                outputStream.write('\n');
            }
            catch (Exception e) { e.printStackTrace(); }
        }

    }


    @Override
    public void autonomousInit() {
        robot.limelight.table.getEntry("ledMode").setNumber(3);
        autoProgram.autonomousInit(robot);
    }

    @Override
    public void autonomousPeriodic() {
        autoProgram.autonomousPeriodic(robot);
    }

    @Override
    public void teleopInit() {
        robot.limelight.table.getEntry("ledMode").setNumber(0);
        robot.limelight.table.getEntry("pipeline").setNumber(0);
        activeCommand.begin(robot);
        robot.setShooterSpeedPresetName(ShooterSpeedPresetName.SHORT_RANGE);
        robot.setCameraTiltDegrees(125);

    }

    @Override
    public void teleopPeriodic() {
        double escalatorPower = 0.0;
        double collectorPower = 0.0;

        if(shooterController.getEnabled()){
            shooterController.run(robot);
        }

        if (leftJoystick.getRawButtonPressed(8)) {
            activeCommand.setEnabled(false);
        }

        if (leftJoystick.getRawButtonReleased(2)){
            robot.limelight.table.getEntry("ledMode").setNumber(0);
            activeCommand.setEnabled(false);
        }

        if (leftJoystick.getRawButtonPressed(15)) {
            robot.limelight.table.getEntry("pipeline").setNumber(0);
        }

        if (activeCommand.isEnabled()) {
            activeCommand.step(robot);
            if (activeCommand.locksControls()) return;
        }

        double  leftPower = -leftJoystick.getY() + leftJoystick.getX();
        double rightPower = -leftJoystick.getY() - leftJoystick.getX();


        double driverRestriction = 0.75;

         leftPower = driverRestriction * deadzoneValue( leftPower, deadZone);
        rightPower = driverRestriction * deadzoneValue(rightPower, deadZone);

        if (leftJoystick.getRawButton(3)) { //nudge left
             leftPower = -0.2;
            rightPower =  0.2;
        }

        if (leftJoystick.getRawButton(4)) { //nudge right
             leftPower =  0.2;
            rightPower = -0.2;
        }

        robot.setMotorPowerPercentage(leftPower, rightPower);

        if (leftJoystick.getRawButtonPressed(2)) {
            activeCommand = limelightAlignChosen;
            activeCommand.begin(robot);
            activeCommand.setEnabled(true);
        }

        //Collector

        if (xboxJoystick.getTriggerAxis(GenericHID.Hand.kRight) > 0) {
            if (robot.getEscalatorSensorMedium()) { //&& (ballCount<=3)
                escalatorPower = 0.5;
            }
            //if (ballCount<=4){collectorPower = 1.0;}
            //if ((ballCount == 4) && !(robot.getEscalatorSensorLow())) { collectorPower = 0.0;}
            robot.collectorIn(1.0);
            robot.escalatorUp(escalatorPower);
        } else if (xboxJoystick.getTriggerAxis(GenericHID.Hand.kLeft) > 0) {
            robot.collectorOut(1.0);
        } else {
            robot.setCollectorPower(0);
        }

         /*
        if (xboxJoystick.getTriggerAxis(GenericHID.Hand.kRight) > 0) {
            escalatorPower = 0.0;
            collectorPower = 0.75;

            //two power cells
            if (robot.getEscalatorSensorLow() && robot.getEscalatorSensorMedium() && !robot.getEscalatorSensorMediumHigh()){
                waitingForMediumHigh = true;
            }

            //three and four power cells
            if (robot.getEscalatorSensorLow() && robot.getEscalatorSensorMedium() && robot.getEscalatorSensorMediumHigh() && !robot.getEscalatorSensorHigh()){
                waitingForChange = true;
                waitingForMediumHigh = false;
            }
            if (waitingForChange && !robot.getEscalatorSensorMediumHigh()){
                waitingForChange = false;
                waitingForMediumHigh = true;
            }
            if (waitingForMediumHigh || waitingForChange){
                escalatorPower = 0.7;
            }
            if (waitingForMediumHigh && robot.getEscalatorSensorMediumHigh()){
                waitingForMediumHigh = false;
            }
*/
            /*if(waitingForMediumHigh && !robot.getEscalatorSensorMediumHigh()){
                escalatorPower = 0.75;
                waitToChange = true;
            }
            if (waitToChange && robot.getEscalatorSensorMediumHigh()){
                escalatorPower = 0.0;
                waitingForMediumHigh = false;
                waitToChange = false;
            }

             */

/*
            if (robot.getEscalatorSensorLow() && robot.getEscalatorSensorMedium() && robot.getEscalatorSensorMediumHigh() && robot.getEscalatorSensorHigh()){
                collectorPower = 0.0;
                escalatorPower = 0.0;
            }
*/
            //if (ballCount<=4){collectorPower = 1.0;}
            //if ((ballCount == 4) && !(robot.getEscalatorSensorLow())) { collectorPower = 0.0;}
        /*
            robot.collectorIn(collectorPower);
            robot.escalatorUp(escalatorPower);
        } else if (xboxJoystick.getTriggerAxis(GenericHID.Hand.kLeft) > 0) {
            robot.collectorOut(1.0);
        } else {
            robot.setCollectorPower(0);
        }
*/
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

        //Elevation Adjuster
        if (xboxJoystick.getY(GenericHID.Hand.kLeft) < -0.5){
            robot.aimUp(0.6);
            shooterController.setEnabled(false);
        } else if (xboxJoystick.getY(GenericHID.Hand.kLeft) > 0.5){
            robot.aimDown(0.6);
            shooterController.setEnabled(false);
        } else if (!shooterController.getEnabled()){
            robot.aimUp(0);
        }


        POVDirection xboxDPadDirection = POVDirection.getDirection(xboxJoystick.getPOV());

        switch (xboxDPadDirection) {
            case SOUTH: //high velocity (long range)
                robot.setShooterSpeedPresetName(ShooterSpeedPresetName.YEET);
                robot.limelight.table.getEntry("pipeline").setNumber(1);
                limelightAlignChosen = limelightAlignFarDistance;
                shooterController.begin(robot);
                shooterController.setEnabled(true);
                shooterController.setSetPoint(131);
                break;

            case NORTH: //low velocity (short range)
                robot.setShooterSpeedPresetName(ShooterSpeedPresetName.SHORT_RANGE);
                robot.limelight.table.getEntry("pipeline").setNumber(0);
                limelightAlignChosen = limeLightAlignCloseDistance;
                shooterController.setEnabled(true);
                shooterController.setSetPoint(154);
                break;

            case EAST: //medium velocity (mid range)
                robot.setShooterSpeedPresetName(ShooterSpeedPresetName.YEET);
                robot.limelight.table.getEntry("pipeline").setNumber(1);
                limelightAlignChosen = limelightAlignFarDistance;
                shooterController.begin(robot);
                shooterController.setEnabled(true);
                shooterController.setSetPoint(137);

                break;

            case WEST: //YEET
                robot.setShooterSpeedPresetName(ShooterSpeedPresetName.YEET);
                robot.limelight.table.getEntry("pipeline").setNumber(1);
                limelightAlignChosen = limelightAlignFarDistance;
                shooterController.begin(robot);
                shooterController.setEnabled(true);
                shooterController.setSetPoint(127);
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
        if (smartClimb.getHolding()) {
            smartClimb.hold(robot);
        } else {
            robot.setClimbVerticalPower(0);
        }

        if (leftJoystick.getRawButtonPressed(7)) {
            smartClimb.begin(robot);
        }
        if (leftJoystick.getRawButtonReleased(7)) {
            smartClimb.setHolding(true);
        }
        if (leftJoystick.getRawButton(7)) {
            smartClimb.step(robot);
        }
        if (leftJoystick.getRawButton(13)) {
            robot.raiseClimberArms(.2);
            smartClimb.setHolding(false);
        }

        //Left individual control
        if (leftJoystick.getRawButton(5)) {
            smartClimb.setHolding(false);
            robot.setClimbVerticalPortPower(.6);
        } else if (leftJoystick.getRawButton(10)) {
            smartClimb.setHolding(false);
            robot.setClimbVerticalPortPower(-.2);
        }

        //Right individual control
        if (leftJoystick.getRawButton(11)) {
            smartClimb.setHolding(false);
            robot.setClimbVerticalStarboardPower(.6);
        } else if (leftJoystick.getRawButton(16)) {
            smartClimb.setHolding(false);
            robot.setClimbVerticalStarboardPower(-.2);
        }


        //below is code for keeping track of number of balls on robot
        if (leftJoystick.getRawButtonPressed(12)) { // resets ball count to zero
            ballCount = 0;
        }
        if (robot.getEscalatorSensorLow()){
            if (!ballCollectCounted) { //add ball
                ballCount++;
                ballCollectCounted = true;
            }
        } else {
            ballCollectCounted = false;
        }

        if (robot.getEscalatorSensorHigh()){
            ballShootCounted = true;
        } else {
            if (ballShootCounted){
                ballCount--;              // subtract ball
            }
            ballShootCounted = false;
        }


    }

    @Override
    public void testInit() {
        LiveWindow.setEnabled(false);
        smartClimb.setHolding(false);
    }


    @Override
    public void testPeriodic() {

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
            robot.escalatorUp(.5);
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
        if (leftJoystick.getRawButton(7)) {
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
            robot.setAimAdjusterPower(0);
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
        //if (leftJoystick.getRawButton(2)) {
        //    robot.setClimbVerticalPower(.6);
        //} else if (leftJoystick.getRawButton(1)) {
        //    robot.setClimbVerticalPower(-.2);
        //} else {
        //    robot.setClimbVerticalPower(0.0);
        //}

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