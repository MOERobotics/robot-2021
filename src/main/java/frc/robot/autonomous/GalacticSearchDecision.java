package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;
import frc.robot.genericrobot.PixyCam;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Queue;

public class GalacticSearchDecision extends GenericAutonomous{

    public Queue<Path> paths = new ArrayDeque<>(11);

    GenericAutonomous
        pathARed = new PathARed (),
        pathBRed = new PathBRed (),
        pathABlue = new PathABlue (),
        pathBBlue = new PathBBlue(),
        chosenPath = null;

    Path
            currentAnswer = Path.Unknown,
            currentPath = Path.Unknown;


    @Override
    protected void printSmartDashboardInternal() {
        SmartDashboard.putString("GSD current seen path", currentPath.toString());
        SmartDashboard.putString("GSD current chosen path", currentAnswer.toString());

        if (chosenPath != null) {
            SmartDashboard.putString("Autonomous Program", chosenPath.getClass().getName());
        }
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot){
        if (chosenPath == null) {
            switch(currentAnswer){
                default: case Unknown: return;
                case RedA: chosenPath = pathARed; break;
                case RedB: chosenPath = pathBRed; break;
                case BlueA: chosenPath = pathABlue; break;
                case BlueB: chosenPath = pathBBlue; break;
            }
        }
    }


    //Path newPath = parsePath(powerCells);



    public Path readAndChoosePath(Path newPath) {

        paths.add(newPath);
        if (paths.size() < 10) {
            return Path.Unknown;
        } else {
            int
                    bluACount = 0,
                    bluBCount = 0,
                    redACount = 0,
                    redBCount = 0,
                    unknownCount = 0;
            for (Path iterPath : paths) {
                switch (iterPath) {
                    case BlueA:
                        bluACount++;
                        break;
                    case BlueB:
                        bluBCount++;
                        break;
                    case RedA:
                        redACount++;
                        break;
                    case RedB:
                        redBCount++;
                        break;
                    case Unknown:
                        unknownCount++;
                        break;
                }
            }
            paths.remove();

            int maxcount = 0;

            maxcount = Math.max(maxcount, bluACount);
            maxcount = Math.max(maxcount, bluBCount);
            maxcount = Math.max(maxcount, redBCount);
            maxcount = Math.max(maxcount, redACount);
            maxcount = Math.max(maxcount, unknownCount);

            if (maxcount == bluACount) return Path.BlueA;
            if (maxcount == bluBCount) return Path.BlueB;
            if(maxcount == redACount) return  Path.RedA;
            if (maxcount == redBCount) return Path.RedB;
            if (maxcount == unknownCount) return Path.Unknown;
            return Path.Unknown;
        }

    }








    public static Path parsePath(PixyCam.Block[] powerCells){
        if (powerCells == null || powerCells.length != 3) return Path.Unknown;

        PixyCam.Block[] powerCellsSorted = Arrays.copyOf(powerCells, powerCells.length);
        Arrays.sort(powerCellsSorted, Comparator.comparingInt(PixyCam.Block::getCenterY));

        PixyCam.Block farCell = powerCellsSorted[0];
        PixyCam.Block midCell = powerCellsSorted[1];
        PixyCam.Block nearCell = powerCellsSorted[2];

        if (midCell.getCenterX() < nearCell.getCenterX()) {

            if (farCell.getCenterX() - midCell.getCenterX() < nearCell.getCenterX() - farCell.getCenterX())
                return Path.BlueA;
            else return Path.BlueB;
        } else {

            if (farCell.getCenterX() < nearCell.getCenterX()) return Path.RedA;
            else return Path.RedB;
        }




    }
    public enum Path{
        RedA,
        RedB,
        BlueA,
        BlueB,
        Unknown
    }

}
