package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.GenericAutonomous;
import frc.robot.genericrobot.GenericRobot;
import java.lang.Math;

public class GalacticA extends GenericAutonomous {
    double inches_traveled = 0;
    double desired_distance = 72;
    boolean begin = true;
    boolean cell_check = false;
    boolean red = false;
    boolean blue = false;
    double default_speed = .2;
    double start_ticks = 0;
    int Turn = 0;
    int Dist = 0;


    @Override
    protected void printSmartDashboardInternal(){

    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        if (begin){
            start_ticks = robot.getDistanceTicksLeft();
            begin = false;
            cell_check = true;
        }
        if (cell_check){
            robot.setMotorPowerPercentage(default_speed,default_speed);
            inches_traveled = (robot.getDistanceInchesLeft() - start_ticks); // subtract start distance to get distance travelled, divide by 116 to get distance in inches

            if (inches_traveled >= desired_distance) {
                // check if power cell is there
                if (cellThere()){
                    red = true;
                    start_ticks = robot.getDistanceTicksLeft();
                    desired_distance = 5*Math.sqrt(5)/2*12;

                }
                else{
                    blue = true;
                    start_ticks = robot.getDistanceTicksLeft();
                    desired_distance = 5*Math.sqrt(13)/2*12;
                }
            }
        }
        if (red){
            switch (Turn){
                case 0:

                        Dist = 1;

                case 1:
                    Dist = 2;
                case 2:
                    Dist = 3;
            }

            switch (Dist){
                case 1:
                    inches_traveled = (robot.getDistanceInchesLeft() - start_ticks);
                    if (inches_traveled >= desired_distance) {
                        start_ticks = robot.getDistanceInchesLeft();
                        Dist = 0;
                        Turn = 1;
                        desired_distance = 5*Math.sqrt(10)/2*12;
                    }
                case 2:
                    inches_traveled = (robot.getDistanceInchesLeft() - start_ticks);
                    if (inches_traveled >= desired_distance) {
                        start_ticks = robot.getDistanceInchesLeft();
                        Dist = 0;
                        Turn = 2;
                        desired_distance = 25*6;
                    }
                case 3:
                    inches_traveled = (robot.getDistanceInchesLeft() - start_ticks);
                    if (inches_traveled >= desired_distance) {
                        start_ticks = robot.getDistanceInchesLeft();
                        Dist = 0;
                        Turn = 3;
                        break;
                    }
            }


        }
        if (blue){
            switch (Dist){
                case 0:
                    inches_traveled = (robot.getDistanceInchesLeft() - start_ticks);
                    if (inches_traveled >= desired_distance) {
                        start_ticks = robot.getDistanceInchesLeft();
                        Dist = 4;
                        Turn = 1;
                        desired_distance = 5 * Math.sqrt(10) / 2 * 12;
                    }
                case 1:
                    inches_traveled = (robot.getDistanceInchesLeft() - start_ticks);
                    if (inches_traveled >= desired_distance) {
                        start_ticks = robot.getDistanceInchesLeft();
                        Dist = 4;
                        Turn = 2;
                        desired_distance = 5 * Math.sqrt(5) / 2 * 12;
                    }
                case 2:
                    inches_traveled = (robot.getDistanceInchesLeft() - start_ticks);
                    if (inches_traveled >= desired_distance) {
                        start_ticks = robot.getDistanceInchesLeft();
                        Dist = 4;
                        Turn = 3;
                        desired_distance = 5 * 12;
                    }
                case 3:
                    inches_traveled = (robot.getDistanceInchesLeft() - start_ticks);
                    if (inches_traveled >= desired_distance) {
                        start_ticks = robot.getDistanceInchesLeft();
                        Dist = 4;
                        Turn = 4;
                        break;
                    }
            }

            switch (Turn){
                case 1:
                    Dist = 1;
                case 2:
                    Dist = 2;
                case 3:
                    Dist = 3;
            }
        }

    }
    public boolean cellThere(){
        return Math.random() < 0.5;
    }

    public void collectBall(){
        //do something
    }
}
