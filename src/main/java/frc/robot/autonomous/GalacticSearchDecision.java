package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;
import frc.robot.genericrobot.PixyCam;

import java.util.*;
import java.util.function.Function;
import java.util.stream.Collectors;

import static frc.robot.autonomous.GalacticSearchDecision.Path.*;

public class GalacticSearchDecision extends GenericAutonomous {

	public Queue<Path> paths = new ArrayDeque<>(11);

	GenericAutonomous
		pathARed   = new PathARed (),
		pathBRed   = new PathBRed (),
		pathABlue  = new PathABlue(),
		pathBBlue  = new PathBBlue(),
		chosenPath = null;

	Path
		currentAnswer = Unknown,
		currentPath   = Unknown;

	@Override
	protected void printSmartDashboardInternal() {
		SmartDashboard.putString("GSD Current seen path"  , currentPath  .toString());
		SmartDashboard.putString("GSD Current chosen path", currentAnswer.toString());
		//hacky hacky
		if (chosenPath != null) {
			SmartDashboard.putString("Autonomous Program", chosenPath.getClass().getName());
		}
	}

	@Override
	public void autonomousPeriodic(GenericRobot robot) {
		if (chosenPath == null) {
			PixyCam.Block[] powerCells = robot.getPixyCamBlocks();
			currentPath   = parsePath(powerCells);
			currentAnswer = readAndChoosePath(currentPath);
			switch (currentAnswer) {
				default: case Unknown: return;
				case  RedA: chosenPath = pathARed ; break;
				case  RedB: chosenPath = pathBRed ; break;
				case BlueA: chosenPath = pathABlue; break;
				case BlueB: chosenPath = pathBBlue; break;
			}
		}
		if (chosenPath != null) {
			chosenPath.autonomousPeriodic(robot);
		}
	}


//Path newPath = parsePath(powerCells);

	public Path readAndChoosePath(Path newPath) {
		paths.add(newPath);
		if (paths.size() < 10) {
			return Unknown;
		} else {
			int    bluACount = 0,
				   bluBCount = 0,
				   redACount = 0,
				   redBCount = 0,
				unknownCount = 0;

			for (Path iterPath : paths) {
				switch (iterPath) {
					case BlueA  :    bluACount++; break;
					case BlueB  :    bluBCount++; break;
					case RedA   :    redACount++; break;
					case RedB   :    redBCount++; break;
					case Unknown: unknownCount++; break;
				}
			}
			paths.remove();

			int maxcount = 0;
				maxcount = Math.max(maxcount,    bluACount);
				maxcount = Math.max(maxcount,    bluBCount);
				maxcount = Math.max(maxcount,    redACount);
				maxcount = Math.max(maxcount,    redBCount);
				maxcount = Math.max(maxcount, unknownCount);

			if (maxcount ==    bluACount) return   BlueA;
			if (maxcount ==    bluBCount) return   BlueB;
			if (maxcount ==    redACount) return    RedA;
			if (maxcount ==    redBCount) return    RedB;
			if (maxcount == unknownCount) return Unknown;
			return Unknown;

		}
	}

			/*
			Path chosenPath = paths
				.stream()
				.collect(
					Collectors.groupingBy(
						Function.identity(),
						Collectors.counting()
					)
				).entrySet()
				.stream()
				.max(Comparator.comparingLong(Map.Entry::getValue))
				.map(Map.Entry::getKey)
				.orElse(Unknown);
			paths.remove();
			return chosenPath;
			*/

	public static Path parsePath(PixyCam.Block[] powerCells) {

		if (powerCells == null || powerCells.length != 3) return Unknown;

		PixyCam.Block[] powerCellsSorted = Arrays.copyOf(powerCells, powerCells.length);
		Arrays.sort(powerCellsSorted, Comparator.comparingInt(PixyCam.Block::getCenterY));

		PixyCam.Block  farCell = powerCellsSorted[0];
		PixyCam.Block  midCell = powerCellsSorted[1];
		PixyCam.Block nearCell = powerCellsSorted[2];

		//Second blue is always on the left,
		//Second red  is always on the right
		if (midCell.getCenterX() < nearCell.getCenterX()) {
			//For blue, the far cell is always in the middle.
			//If it's closer to mid, it's path A.
			//If it's closer to near, it's path B.
			if (
				farCell .getCenterX() - midCell.getCenterX() <
				nearCell.getCenterX() - farCell.getCenterX()
			) return BlueA;
			else return BlueB;
		} else {
			//For red, the far cell is always on the left.
			//If the near cell is to the left  of it, it's path B.
			//If the near cell is to the right of it, it's path A.
			if (
				farCell.getCenterX() < nearCell.getCenterX()
			) return RedA;
			else return RedB;
		}
	}

	public enum Path {
		RedA,
		RedB,
		BlueA,
		BlueB,
		Unknown
	}
}
