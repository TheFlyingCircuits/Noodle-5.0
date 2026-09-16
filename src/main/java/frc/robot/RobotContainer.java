// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.PlayingField.FieldElement;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.gyro.GyroIOMapleSim;
import frc.robot.subsystems.drivetrain.gyro.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.swerve.SwerveModuleIOKraken;
import frc.robot.subsystems.drivetrain.swerve.SwerveModuleIOMapleSim;

public class RobotContainer {

    public final Drivetrain drivetrain;
    protected final HumanDriver duncan = new HumanDriver(0);
    final CommandXboxController duncanController;

    Timer pathTimer;
    Optional<Trajectory<SwerveSample>> trajectory2 = Choreo.loadTrajectory("Example_auto_p2");

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
                DCMotor.getNEO(1), // Steer motor is a Falcon 500
                1.0/Constants.SwerveModuleConstants.driveGearReduction, // Drive motor gear ratio.
                1.0/Constants.SwerveModuleConstants.steerGearReduction, // Steer motor gear ratio.
                Volts.of(0.25), // Drive friction voltage.
                Volts.of(0.2), // Steer friction voltage
                Inches.of(Units.metersToInches(Constants.SwerveModuleConstants.wheelRadiusMeters)), // Wheel radius
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

        drivetrain.odometry.setPoseMeters(new Pose2d(3, 3, new Rotation2d()));

        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
        pathTimer = new Timer();
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

  public boolean isOnRightSideField() {

    // gets linear distance from robot pose to both trech poses and checks what trench is closer
    double distFromLeftTrench = drivetrain.odometry.getPoseMeters().getTranslation().getDistance(FieldElement.TRENCH_LEFT.getLocation2d());
    double distFromRightTrench = drivetrain.odometry.getPoseMeters().getTranslation().getDistance(FieldElement.TRENCH_RIGHT.getLocation2d());

    return (distFromRightTrench < distFromLeftTrench);
  }
  
  public Command followChoreoTrajoectory(String trajName, double toleranceMeters, double velocityToleranceMPS) {
    // gets trajectory from choreo file and checks if it exists and if not returns
    Optional<Trajectory<SwerveSample>> optionalTrajectory = Choreo.loadTrajectory(trajName);
    if(optionalTrajectory.isEmpty()) return new InstantCommand();

    // gets alliance value and sees if mirrors path so robot follows correct path
    final boolean isRedAlliance = DriverStation.getAlliance().get() == Alliance.Red;

    boolean shouldMirror = isOnRightSideField();
    Trajectory<SwerveSample> trajectory = shouldMirror ? optionalTrajectory.get().mirrorY(): optionalTrajectory.get();
    
    // restarts path timer to 0 for path following
    // runs path until it reaches within position and tolerance of end of path
    return new SequentialCommandGroup(
      new InstantCommand(() -> pathTimer.restart()),
      drivetrain.run(() -> {drivetrain.followTrajectory(trajectory, pathTimer.get(), isRedAlliance);})
        .until(() -> drivetrain.isAtEndOfTrajectory(toleranceMeters, velocityToleranceMPS, trajectory.getFinalSample(isRedAlliance).get()))
    );
  }

  /** Called by Robot.java, convenience function for logging. */
  public void periodic() {
    // if in sim set fused pose to maple sim pose and log it
    if(RobotBase.isSimulation()) {
      drivetrain.odometry.setPoseMeters(swerveDriveSimulation.getSimulatedDriveTrainPose());
      Logger.recordOutput("robotContainer/simulatedDrivetrainPoseMeters", swerveDriveSimulation.getSimulatedDriveTrainPose());
    }
  }

  public Command getAutonomousCommand() {
    return trenchShallowAuto();
  }

  public Command trenchShallowAuto() {
    // need to replace Commands.wait with shooting and add intake and stuff
    return new SequentialCommandGroup(
      followChoreoTrajoectory("Example_auto_1", 0.1, 0.1),
      Commands.waitSeconds(1),
      followChoreoTrajoectory("Example_auto_p2", 0.1, 0.1),
      Commands.waitSeconds(1)
    );
  }


}
