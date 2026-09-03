// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOMapleSim;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.SwerveModuleIOKraken;
import frc.robot.subsystems.drivetrain.SwerveModuleIOMapleSim;

public class RobotContainer {

    public final Drivetrain drivetrain;
    protected final HumanDriver duncan = new HumanDriver(0);
    final CommandXboxController duncanController;

    private SwerveDriveSimulation swerveDriveSimulation;
    
  public RobotContainer() {
    /**** INITIALIZE SUBSYSTEMS ****/
    if (RobotBase.isReal()) { //TODO put this back to not !
        // NOODLE OFFSETS: FL -0.184814453125, FR 0.044677734375, BL -0.3349609375, BR 0.088134765625 
        drivetrain = new Drivetrain( 
            new GyroIOPigeon(),
            new SwerveModuleIOKraken(1, 2, -0.260498, 1, "FL", true), 
            new SwerveModuleIOKraken(3, 4, 0.429199, 2, "FR", false),
            new SwerveModuleIOKraken(5, 6, -0.033203, 3, "BL", true),
            new SwerveModuleIOKraken(7, 8,  0.098389, 4, "BR", false) 
        );
    } else {
        // drivetrain = new Drivetrain(
        //     new GyroIOSim(){},
        //     new SwerveModuleIOSim(){},
        //     new SwerveModuleIOSim(){},
        //     new SwerveModuleIOSim(){},
        //     new SwerveModuleIOSim(){}
        // );
      DriveTrainSimulationConfig driveSimulationConfig = DriveTrainSimulationConfig.Default()
        // Specify gyro type (for realistic gyro drifting and error simulation)
        .withGyro(COTS.ofPigeon2())
        // Specify swerve module (for realistic swerve dynamics)
        .withSwerveModule(new SwerveModuleSimulationConfig(
                DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                DCMotor.getKrakenX44(1), // Steer motor is a Falcon 500
                1.0/Constants.SwerveModuleConstants.driveGearReduction, // Drive motor gear ratio.
                1.0/Constants.SwerveModuleConstants.steerGearReduction, // Steer motor gear ratio.
                Volts.of(0.25), // Drive friction voltage.
                Volts.of(0.2), // Steer friction voltage
                Inches.of(Constants.SwerveModuleConstants.wheelRadiusMeters), // Wheel radius
                KilogramSquareMeters.of(0.03), // Steer MOI
                1.2)) // Wheel COF
        // Configures the track length and track width (spacing between swerve modules)
        .withTrackLengthTrackWidth(
            Meters.of(DrivetrainConstants.wheelbaseMeters), 
            Meters.of(DrivetrainConstants.trackwidthMeters)
            )
        // Configures the bumper size (dimensions of the robot bumper)
        .withBumperSize(
            Meters.of(DrivetrainConstants.bumperWidthMeters),
            Meters.of(DrivetrainConstants.bumperWidthMeters)
        );

        /* Create a swerve drive simulation */
        swerveDriveSimulation = new SwerveDriveSimulation(
                // Specify Configuration
                driveSimulationConfig,
                // Specify starting pose
                new Pose2d(3, 3, new Rotation2d())
        );

        drivetrain = new Drivetrain(
            new GyroIOMapleSim(swerveDriveSimulation.getGyroSimulation()),
            new SwerveModuleIOMapleSim(swerveDriveSimulation.getModules()[0]),
            new SwerveModuleIOMapleSim(swerveDriveSimulation.getModules()[1]),
            new SwerveModuleIOMapleSim(swerveDriveSimulation.getModules()[2]),
            new SwerveModuleIOMapleSim(swerveDriveSimulation.getModules()[3])
        );

        drivetrain.setPoseMeters(new Pose2d(3, 3, new Rotation2d()));

        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
    }

    duncanController = duncan.getXboxController();
    configureBindings();
    setDefaultCommands();
  }

  private void configureBindings() {}

  public void setDefaultCommands() {
      drivetrain.setDefaultCommand(driverFullyControlDrivetrain().withName("driveDefualtCommand"));
      // canLedsCounter.setDefaultCommand(canLedsCounter.solidColorCommand(Color.fromHSV(canLedsCounter.getAllianceHue(), 255, 255)).ignoringDisable(true));
  }

      private Command driverFullyControlDrivetrain() { return drivetrain.run(() -> {
        drivetrain.fieldOrientedDrive(duncan.getRequestedFieldOrientedVelocity());
        Logger.recordOutput("drivetrain/runningDefaultCommand", true);
        }).finallyDo(() -> {
            Logger.recordOutput("drivetrain/runningDefaultCommand", false);
        }).withName("driverFullyControlDrivetrain");
    }

  /** Called by Robot.java, convenience function for logging. */
  public void periodic() {
    Logger.recordOutput("robotContainer/simulatedDrivetrainPoseMeters", swerveDriveSimulation.getSimulatedDriveTrainPose());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }


}
