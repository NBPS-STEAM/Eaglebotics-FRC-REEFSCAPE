// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.oldordrivecommands.ScoreCommands.BallIntakeCommands;
import frc.robot.commands.oldordrivecommands.ScoreCommands.HangCommands;
import frc.robot.commands.oldordrivecommands.ScoreCommands.OpCommands;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.commands.oldordrivecommands.ScoreCommands.PipeIntakeCommands;
import frc.robot.subsystems.BallIntakeSubsystem;
import frc.robot.subsystems.HangSubsystem;
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
  public final IntakePositionSubsystem intakePosition = new IntakePositionSubsystem();

  public final BallIntakeSubsystem ballIntake = new BallIntakeSubsystem();
  public final HangSubsystem hangSubsystem = new HangSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandPS5Controller driverGamepad = new CommandPS5Controller(0);
  final CommandPS5Controller coDriverGamepad = new CommandPS5Controller(1);

  SendableChooser<Command> AutoChooser = new SendableChooser<>();


  PipeIntakeCommands pipeIntakeCommands = new PipeIntakeCommands(pipeIntake);
  BallIntakeCommands ballIntakeCommands = new BallIntakeCommands(ballIntake);
  HangCommands hangCommands = new HangCommands(hangSubsystem);
  OpCommands opCommands = new OpCommands(intakePosition);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Register commands for PathPlanner
    registerNamedCommands();

    // Configure the trigger bindings
    configureBindings();

    drivebase.setDefaultCommand(OpCommands.getDriveCommand(drivebase, driverGamepad));

    setAutoCommands();
    SmartDashboard.putData("Autos", AutoChooser);
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
    //Gets the slow version (half speed) of the drive command. That way our robot can go slow. We need the repeat because
    //while true does not repeat
    driverGamepad.L2().whileTrue(new RepeatCommand(OpCommands.getSlowDriveCommand(drivebase, coDriverGamepad)));
    driverGamepad.circle().onTrue(Commands.runOnce(drivebase::zeroGyro));
    

    //Co Driver:
    
    // Pipe Intake/Outtake/Stop Controls
    coDriverGamepad.L1().onTrue(pipeIntakeCommands.new Intake());
    coDriverGamepad.L2().onTrue(pipeIntakeCommands.new Outtake());
    coDriverGamepad.L3().onTrue(pipeIntakeCommands.new StopIntake());
    
    // Ball Intake/Outtake/Stop Controls
    coDriverGamepad.R1().onTrue(ballIntakeCommands.new Intake());
    coDriverGamepad.R2().onTrue(ballIntakeCommands.new Outtake());
    coDriverGamepad.R3().onTrue(ballIntakeCommands.new StopIntake());
  
    // Hang Control
    coDriverGamepad.options().onTrue(hangCommands.new Activate());
  
    // Ball Set Positions
    coDriverGamepad.cross().onTrue(opCommands.getBall1Command());
    coDriverGamepad.square().onTrue(opCommands.getBall2Command());
    coDriverGamepad.triangle().onTrue(opCommands.getBall3Command());
    coDriverGamepad.circle().onTrue(opCommands.getBall4Command());

    // Pipe Set Positions
    coDriverGamepad.povDown().onTrue(opCommands.getPipe1Command());
    coDriverGamepad.povRight().onTrue(opCommands.getPipe2Command());
    coDriverGamepad.povUp().onTrue(opCommands.getPipe3Command());
    coDriverGamepad.povLeft().onTrue(opCommands.getPipe4Command());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AutoChooser.getSelected(); 
  }

  public void setAutoCommands(){
    AutoChooser.addOption("One-Coral-Center", new PathPlannerAuto("One-Coral-Center"));
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("Pipe Outtake", pipeIntakeCommands.new Outtake());
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