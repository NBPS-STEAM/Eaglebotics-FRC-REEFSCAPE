// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.oldordrivecommands.ScoreCommands.OpCommands;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.PipeIntakeSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  public final SensorSubsystem sensors=new SensorSubsystem();
  public final VisionSubsystem vision=new VisionSubsystem();
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  public final PipeIntakeSubsystem pipeIntake = new PipeIntakeSubsystem();
  public final IntakePositionSubsystem intakePositionSubsystem = new IntakePositionSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandPS5Controller driverGamepad = new CommandPS5Controller(0);
  final CommandPS5Controller coDriverGamepad = new CommandPS5Controller(1);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    drivebase.setDefaultCommand(OpCommands.getDriveCommand(drivebase, driverGamepad));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
   
    driverGamepad.circle().onTrue((Commands.runOnce(drivebase::zeroGyro)));

    

    coDriverGamepad.cross().onTrue(OpCommands.getBall1Command(intakePositionSubsystem, coDriverGamepad));
    coDriverGamepad.square().onTrue(OpCommands.getBall2Command(intakePositionSubsystem, coDriverGamepad));
    coDriverGamepad.triangle().onTrue(OpCommands.getBall3Command(intakePositionSubsystem, coDriverGamepad));
    coDriverGamepad.circle().onTrue(OpCommands.getBall4Command(intakePositionSubsystem, coDriverGamepad));

    coDriverGamepad.povDown().onTrue(OpCommands.getPipe1Command(intakePositionSubsystem, coDriverGamepad));
    coDriverGamepad.povRight().onTrue(OpCommands.getPipe2Command(intakePositionSubsystem, coDriverGamepad));
    coDriverGamepad.povUp().onTrue(OpCommands.getPipe3Command(intakePositionSubsystem, coDriverGamepad));
    coDriverGamepad.povLeft().onTrue(OpCommands.getPipe4Command(intakePositionSubsystem, coDriverGamepad));






    
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}