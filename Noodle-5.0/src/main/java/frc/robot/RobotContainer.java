// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.SwerveModuleIOKraken;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;

public class RobotContainer {

    public final Drivetrain drivetrain;
    protected final HumanDriver duncan = new HumanDriver(0);
    final CommandXboxController duncanController;
    
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
        drivetrain = new Drivetrain(
            new GyroIOSim(){},
            new SwerveModuleIOSim(){},
            new SwerveModuleIOSim(){},
            new SwerveModuleIOSim(){},
            new SwerveModuleIOSim(){}
        );
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

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
