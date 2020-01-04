/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;

public class Robot extends TimedRobot {

  @Override public void robotInit() {
    var dummyPidController = new PIDController(0,0,0) {{
      setSetpoint(0);
      setTolerance(0.1);
      enableContinuousInput(-180,180);
    }};
    double error = 0.1;
    double correction = dummyPidController.calculate(error);
  }

  @Override public void robotPeriodic() {

  }

  @Override public void autonomousInit() {

  }

  @Override public void autonomousPeriodic() {

  }

  @Override public void teleopInit() {

  }

  @Override public void teleopPeriodic() {

  }

  @Override public void testInit() {

  }

  @Override public void testPeriodic() {

  }

}
