// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot  {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    initAdvantageKit();
    m_robotContainer = new RobotContainer();

    DriverStation.silenceJoystickConnectionWarning(true);

    if(RobotBase.isSimulation()) {
      // Obtains the default instance of the simulation world, which is a Crescendo Arena.
      SimulatedArena.getInstance();
      // Add a fuel
      SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(3, 3)));
      // Overrides the default simulation
      // SimulatedArena.overrideInstance(); 
    }
  }

  private void initAdvantageKit() {
    Logger.recordMetadata("projectName", "Noodle-5.0");
    Logger.addDataReceiver(new NT4Publisher());
    if (Constants.atCompetition) {
      Logger.addDataReceiver(new WPILOGWriter()); // <- log to USB stick
    }
    // new PowerDistribution();    // Apparently just constructing a PDH
    //                             // will allow it's values to be logged? 
    //                             // This is what the advantage kit docs imply at least.
    // TODO: We may not need this anymore as of 2026. The AdvantageKit docs indicate that this is
    //       automatically done for us now, provided we use the default CAN ID for our PDH.
    Logger.start();
  //   SignalLogger.setPath("/media/sdb1/ctre");
  //   SignalLogger.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if(RobotBase.isSimulation()) {
      simulationPeriod();
    }
  }

public void simulationPeriod() {
  // Get the positions of the fuel (both on the field and in the air)
  Pose3d[] fuelPoses = SimulatedArena.getInstance()
        .getGamePiecesArrayByType("Fuel");
  // Publish to telemetry using AdvantageKit
  Logger.recordOutput("FieldSimulation/FuelPositions", fuelPoses);
  SimulatedArena.getInstance().simulationPeriodic();
}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    m_robotContainer.drivetrain.allowTeleportsNextPoseUpdate();
    m_robotContainer.drivetrain.fullyTrustVisionNextPoseUpdate();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
