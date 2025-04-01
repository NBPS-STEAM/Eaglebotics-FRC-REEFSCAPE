// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.oldordrivecommands.AutoCommands.WaitCommand;
import frc.robot.commands.oldordrivecommands.ScoreCommands.BallIntakeCommands;
import frc.robot.commands.oldordrivecommands.ScoreCommands.HangCommands;
import frc.robot.commands.oldordrivecommands.ScoreCommands.IntakePositionCommand;
import frc.robot.commands.oldordrivecommands.ScoreCommands.OpCommands;
import frc.robot.commands.oldordrivecommands.ScoreCommands.PipeIntakeCommands;
import frc.robot.commands.oldordrivecommands.ScoreCommands.StowCommand;
import frc.robot.commands.oldordrivecommands.ScoreCommands.TelePathingCommands;
import frc.robot.commands.oldordrivecommands.TestCommands.TestCommand;
import frc.robot.subsystems.BallIntakeSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.PipeIntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveModule;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{//test
  public double[] maxCurrentElevator={0,0,0};
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  public final VisionSubsystem vision = new VisionSubsystem(drivebase);
  public final PipeIntakeSubsystem pipeIntake = new PipeIntakeSubsystem();
  public final IntakePositionSubsystem intakePosition = new IntakePositionSubsystem();

  public final BallIntakeSubsystem ballIntake = new BallIntakeSubsystem();
  public final HangSubsystem hangSubsystem = new HangSubsystem();

  PipeIntakeCommands pipeIntakeCommands = new PipeIntakeCommands(pipeIntake);
  BallIntakeCommands ballIntakeCommands = new BallIntakeCommands(ballIntake);
  IntakePositionCommand intakePositionCommands = new IntakePositionCommand(intakePosition);
  HangCommands hangCommands = new HangCommands(hangSubsystem);
  OpCommands opCommands = new OpCommands(intakePosition);
  TelePathingCommands telePathingCommands = new TelePathingCommands(drivebase);
  public final TestCommand test=new TestCommand(drivebase, pipeIntake, intakePosition, ballIntake);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandPS5Controller driverGamepad = new CommandPS5Controller(0);
  final CommandPS5Controller coDriverGamepad = new CommandPS5Controller(1);
  final CommandGenericHID buttonPanel = new CommandGenericHID(2);
  /*
   * The button IDs on the button panel follow this layout:
   * 
   * Stick panel (LED pointing forward):
   * 1 2
   * 3 4
   * Left stick: axis 0 (horizontal), axis 1 (vertical)
   * Right stick: axis 4 (horizontal), axis 5 (vertical)
   * 
   * Button panel (3 LEDs pointing forward, 1 LED pointing right):
   * 13  9  5
   * 14 10  6
   * 15 11  7
   * 16 12  8
   */

  SendableChooser<Command> autoChooser = new SendableChooser<>();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Register commands for PathPlanner
    registerNamedCommands();

    // Configure the trigger bindings
    //configureBindings1(); // Set positions only (no command groups for IntakePosition set positions) (NO LONGER IMPLEMENTED, SEE GIT HISTORY)
    //configureBindings2(); // Sequential command groups for IntakePosition set positions
    //configureBindings3(); // Toggleable pipe/ball mode with sequential command groups for IntakePosition set positions
    //configureBindingsPanel1(); // Co-driver controls on the custom button panel with sequential command groups for IntakePosition set positions
    configureBindingsPanel2(); // Alternate co-driver controls on the custom button panel with sequential command groups for IntakePosition set positions

    setAutoCommands();
    
    SmartDashboard.putData("Autos", autoChooser);
  }

  










  //configureBindings2 is the bindings with sequential command groups and no coral/ball mode
  private void configureBindings2()
  {

    // DRIVER CONTROLS:

    //Joysticks (Default) - Drive the robot
    Command driveCommand = OpCommands.getDriveCommand(drivebase, driverGamepad);
    drivebase.setDefaultCommand(driveCommand);
    sticksInUseTrigger(driverGamepad).whileTrue(driveCommand); // to interrupt other commands when the sticks are in use

    //L2 - Gets the slow version (half speed) of the drive command. That way our robot can go slow.
    //driverGamepad.L2().whileTrue(OpCommands.getTemporarySlowSpeedCommand(drivebase));

    //Options - Zeros the robot
    driverGamepad.options().onTrue(Commands.runOnce(drivebase::zeroGyro));

    //Many Buttons - Auto Drive
    driverGamepad.povLeft().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveSide(-1)));
    driverGamepad.povDown().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveSide(0)));
    driverGamepad.povRight().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveSide(1)));

    driverGamepad.triangle().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(0)));
    driverGamepad.L1().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(1)));
    driverGamepad.square().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(2)));
    driverGamepad.cross().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(3)));
    driverGamepad.circle().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(4)));
    driverGamepad.R1().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(5)));

    driverGamepad.L2().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToCoralStation()));

    // Unlike all other commands, this "deferred" command is generated on command initialization, not instantiation.
    // In other words, this path-following command won't be generated until the command starts running.
    driverGamepad.R2().whileTrue(telePathingCommands.getAutoDriveDeferredCommand());
    




    //CODRIVER CONTROLS:

    //Options - Stow Position
    coDriverGamepad.options().onTrue(opCommands.getStowParallelCommand());



    //L1 - Pipe Intake
    coDriverGamepad.L1().onTrue(opCommands.getPipeIntakeFullCommand(pipeIntakeCommands));

    //L2 - Pipe Outtake
    coDriverGamepad.L2().onTrue(pipeIntakeCommands.getAwareOuttakeCommand(intakePosition, intakePositionCommands));

    //R2 - Ball Outtake
    coDriverGamepad.R2().onTrue(ballIntakeCommands.getAwareOuttakeCommand(intakePosition, intakePositionCommands));

    //DPad - Pipe Set Positions
    coDriverGamepad.povDown().onTrue(opCommands.pipeCommandGroup(1));
    coDriverGamepad.povLeft().onTrue(opCommands.pipeCommandGroup(2));
    coDriverGamepad.povRight().onTrue(opCommands.pipeCommandGroup(3));
    coDriverGamepad.povUp().onTrue(opCommands.pipeCommandGroup(4));



    //PS - Barge Shoot Position
    coDriverGamepad.PS().onTrue(opCommands.bargeShootCommandGroup());
  
    //Buttons - Ball Set Positions
    //X/Cross - Ground Ball
    coDriverGamepad.cross().onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(1),
      ballIntakeCommands. new Intake(),
      new StowCommand(intakePosition)
    ));
    //Circle - Processor Ball
    coDriverGamepad.circle().onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(2),
      ballIntakeCommands. new Intake()
    ));
    //Square - Low Reef Ball
    coDriverGamepad.square().onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(3),
      ballIntakeCommands. new Intake(),
      new StowCommand(intakePosition)
    ));
    //Triangle - High Reef Ball
    coDriverGamepad.triangle().onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(4),
      ballIntakeCommands. new Intake(),
      new StowCommand(intakePosition)
    ));



    //R1 - Zero Lift
    coDriverGamepad.R1().debounce(0.5).onTrue(Commands.runOnce(intakePosition::zeroLift));

    //Joysticks - Manual Lift/Pivot
    new Trigger(() -> Math.abs(coDriverGamepad.getLeftY()) > Constants.OIConstants.kDriveDeadband)
            .whileTrue(intakePositionCommands.new AdjustPivot(() -> -coDriverGamepad.getLeftY()));
    new Trigger(() -> Math.abs(coDriverGamepad.getRightY()) > Constants.OIConstants.kDriveDeadband)
            .whileTrue(intakePositionCommands.new AdjustLift(() -> -coDriverGamepad.getRightY()));

  }











  private boolean isBallMode = false;

  private void toggleBallMode() {
    isBallMode = !isBallMode;
  }

  /**
   * Version 3: Toggleable pipe/ball mode with sequential command groups for IntakePosition set positions
   * 
   * <p>Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.</p>
   */
  private void configureBindings3()
  {
    // Pipe/Ball Mode Control
    Trigger inBallMode = new Trigger(() -> isBallMode);
    Trigger inPipeMode = inBallMode.negate();
    Command toggleControlMode = Commands.runOnce(this::toggleBallMode);

    // Driver:

    Command driveCommand = OpCommands.getDriveCommand(drivebase, driverGamepad);
    drivebase.setDefaultCommand(driveCommand);
    sticksInUseTrigger(driverGamepad).whileTrue(driveCommand); // to interrupt other commands when the sticks are in use
    //Gets the slow version (half speed) of the drive command. That way our robot can go slow.
    //driverGamepad.L2().whileTrue(OpCommands.getTemporarySlowSpeedCommand(drivebase));
    driverGamepad.options().onTrue(Commands.runOnce(drivebase::zeroGyro));

    // Auto Drive
    driverGamepad.povLeft().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveSide(-1)));
    driverGamepad.povDown().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveSide(0)));
    driverGamepad.povRight().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveSide(1)));

    driverGamepad.triangle().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(0)));
    driverGamepad.L1().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(1)));
    driverGamepad.square().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(2)));
    driverGamepad.cross().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(3)));
    driverGamepad.circle().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(4)));
    driverGamepad.R1().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(5)));

    driverGamepad.L2().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToCoralStation()));

    // Unlike all other commands, this "deferred" command is generated on command initialization, not instantiation.
    // In other words, this path-following command won't be generated until the command starts running.
    driverGamepad.R2().whileTrue(telePathingCommands.getAutoDriveDeferredCommand());
    

    //Co Driver:

    // Manual lift and pivot controls
    coDriverGamepad.R1().debounce(0.5).onTrue(Commands.runOnce(intakePosition::zeroLift));
    new Trigger(() -> Math.abs(coDriverGamepad.getLeftY()) > Constants.OIConstants.kDriveDeadband)
            .whileTrue(intakePositionCommands.new AdjustPivot(() -> -coDriverGamepad.getLeftY()));
    new Trigger(() -> Math.abs(coDriverGamepad.getRightY()) > Constants.OIConstants.kDriveDeadband)
            .whileTrue(intakePositionCommands.new AdjustLift(() -> -coDriverGamepad.getRightY()));

    // Toggle Control Mode
    coDriverGamepad.povRight().onTrue(toggleControlMode);
    
    // Pipe Intake/Outtake/Stop Controls
    coDriverGamepad.L1().and(inPipeMode).onTrue(opCommands.getPipeIntakeFullCommand(pipeIntakeCommands));

    coDriverGamepad.L2().and(inPipeMode).onTrue(pipeIntakeCommands.getAwareOuttakeCommand(intakePosition, intakePositionCommands));
    //coDriverGamepad.L2().and(inPipeMode).onTrue(intakeNormal);
    
    
    // Ball Intake/Outtake/Stop Controls
    coDriverGamepad.L2().and(inBallMode).onTrue(ballIntakeCommands.getAwareOuttakeCommand(intakePosition, intakePositionCommands));

    // Stow Position
    coDriverGamepad.options().onTrue(opCommands.getStowParallelCommand());
  
    // Barge Shoot Position
    coDriverGamepad.PS().onTrue(opCommands.bargeShootCommandGroup());
  
    // Ball Set Positions
    coDriverGamepad.cross().and(inBallMode).onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(1),
      ballIntakeCommands. new Intake(),
      new StowCommand(intakePosition)
    ));

    coDriverGamepad.circle().and(inBallMode).onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(3),
      ballIntakeCommands. new Intake(),
      new StowCommand(intakePosition)
    ));

    coDriverGamepad.square().and(inBallMode).onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(4),
      ballIntakeCommands. new Intake(),
      new StowCommand(intakePosition)
    ));

    coDriverGamepad.triangle().and(inBallMode).onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(2),
      ballIntakeCommands. new Intake()
    ));

    // Pipe Set Positions
    coDriverGamepad.cross().and(inPipeMode).onTrue(opCommands.pipeCommandGroup(1));
    coDriverGamepad.circle().and(inPipeMode).onTrue(opCommands.pipeCommandGroup(2));
    coDriverGamepad.square().and(inPipeMode).onTrue(opCommands.pipeCommandGroup(3));
    coDriverGamepad.triangle().and(inPipeMode).onTrue(opCommands.pipeCommandGroup(4));

  }





  /**
   * Configure bindings for controls on the custom button panel.
   */
  private void configureBindingsPanel1()
  {

    // DRIVER CONTROLS:

    //Joysticks (Default) - Drive the robot
    Command driveCommand = OpCommands.getDriveCommand(drivebase, driverGamepad);
    drivebase.setDefaultCommand(driveCommand);
    sticksInUseTrigger(driverGamepad).whileTrue(driveCommand); // to interrupt other commands when the sticks are in use

    //L2 - Gets the slow version (half speed) of the drive command. That way our robot can go slow.
    //driverGamepad.L2().whileTrue(OpCommands.getTemporarySlowSpeedCommand(drivebase));

    //Options - Zeros the robot
    driverGamepad.options().onTrue(Commands.runOnce(drivebase::zeroGyro));

    //Many Buttons - Auto Drive
    driverGamepad.povLeft().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveSide(-1)));
    driverGamepad.povDown().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveSide(0)));
    driverGamepad.povRight().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveSide(1)));

    driverGamepad.triangle().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(0)));
    driverGamepad.L1().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(1)));
    driverGamepad.square().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(2)));
    driverGamepad.cross().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(3)));
    driverGamepad.circle().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(4)));
    driverGamepad.R1().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(5)));

    driverGamepad.L2().onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToCoralStation()));

    // Unlike all other commands, this "deferred" command is generated on command initialization, not instantiation.
    // In other words, this path-following command won't be generated until the command starts running.
    driverGamepad.R2().whileTrue(telePathingCommands.getAutoDriveDeferredCommand());
    




    //CODRIVER CONTROLS:

    //C3:R4 - Stow Position
    buttonPanel.button(8).onTrue(opCommands.getStowParallelCommand());



    //C3:R3 - Pipe Intake
    buttonPanel.button(7).onTrue(opCommands.getPipeIntakeFullCommand(pipeIntakeCommands));

    //C3:R2 - Pipe Outtake
    buttonPanel.button(6).onTrue(pipeIntakeCommands.getAwareOuttakeCommand(intakePosition, intakePositionCommands));

    //C3:R1 - Ball Outtake
    buttonPanel.button(5).onTrue(ballIntakeCommands.getAwareOuttakeCommand(intakePosition, intakePositionCommands));

    //C1:R1-4 - Pipe Set Positions
    buttonPanel.button(16).onTrue(opCommands.pipeCommandGroup(1));
    buttonPanel.button(15).onTrue(opCommands.pipeCommandGroup(2));
    buttonPanel.button(14).onTrue(opCommands.pipeCommandGroup(3));
    buttonPanel.button(13).onTrue(opCommands.pipeCommandGroup(4));



    // -- Ball Set Positions --
    //C2:R4 - Ground Ball
    buttonPanel.button(12).onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(1),
      ballIntakeCommands. new Intake(),
      new StowCommand(intakePosition)
    ));
    //C2:R3 - Low Reef Ball
    buttonPanel.button(11).onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(3),
      ballIntakeCommands. new Intake(),
      new StowCommand(intakePosition)
    ));
    //C2:R2 - High Reef Ball
    buttonPanel.button(10).onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(4),
      ballIntakeCommands. new Intake(),
      new StowCommand(intakePosition)
    ));
    //C2:R1 - Barge Shoot Position
    buttonPanel.button(9).onTrue(opCommands.bargeShootCommandGroup());

    //Gamepad:Dpad Up - Processor Ball
    coDriverGamepad.povUp().onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(2),
      ballIntakeCommands. new Intake()
    ));



    //Gamepad:PS+Options (hold for 0.1s) - Activate Then Move Hang
    //coDriverGamepad.PS().and(coDriverGamepad.options()).debounce(0.1).whileTrue(hangCommands.new Activate(HangConstants.kHangTwistPower, HangConstants.kHangUnlockPos));



    // -- Manual Control Overrides --
    //Joysticks:C1:R2 - Toggle Pipe Intake
    buttonPanel.button(3).toggleOnTrue(pipeIntakeCommands.new Intake());
    //Joysticks:C1:R1 - Toggle Pipe Outtake
    buttonPanel.button(1).toggleOnTrue(pipeIntakeCommands.new Outtake());
    //Joysticks:C2:R2 - Toggle Ball Intake
    buttonPanel.button(4).toggleOnTrue(ballIntakeCommands.new Intake());
    //Joysticks:C2:R1 - Toggle Ball Outtake
    buttonPanel.button(2).toggleOnTrue(ballIntakeCommands.new Outtake());

    //Joysticks:Left - Manual Lift
    buttonPanel.axisMagnitudeGreaterThan(1, Constants.OIConstants.kDriveLargeDeadband)
            .whileTrue(intakePositionCommands.new AdjustLift(() -> buttonPanel.getRawAxis(1)));
    //Joysticks:Right - Manual Pivot
    buttonPanel.axisMagnitudeGreaterThan(5, Constants.OIConstants.kDriveLargeDeadband)
            .whileTrue(intakePositionCommands.new AdjustPivot(() -> -buttonPanel.getRawAxis(5)));
    
    //Gamepad:R1 (hold for 0.25s) - Zero Lift
    coDriverGamepad.R1().debounce(0.25).onTrue(Commands.runOnce(intakePosition::zeroLift));

    //Gamepad:Circle - Unset Auto Drive
    coDriverGamepad.circle().onTrue(Commands.runOnce(telePathingCommands::setAutoDriveNone));

  }





  /**
   * Configure alternate bindings for controls on the custom button panel.
   * These bindings use a different layout of the panels than configureBindingsPanel1().
   * The panels may be laid out with the stick panel either directly above the button panel or against the upper left/right side of it.
   * The gamepad is intended to be layed out upside-down against the bottom side of either panel with the triggers pointing back.
   * Controls follow this layout on the button panel:
   *  _1__2__3_
   * 1|P4    BH
   * 2|P3    BL
   * 3|P2 AD BB
   * 4|AD ST AD
   * 5|AD DM AD
   * 6|DL AD DR
   * There are some additional inputs on the gamepad.
   *
   * ST - Go to stow position
   * P2,P3,P4 - Go to pipe scoring position level 2, 3, or 4
   * BL - Go to low ball scoring position
   * BH - Go to high ball scoring position
   * BB - Go to barge ball scoring position
   * AD - Set auto drive destination to corresponding reef side
   * DL,DM,DR - Set auto drive side (left, middle, right)
   */
  private void configureBindingsPanel2()
  {

    // DRIVER CONTROLS:

    //Joysticks (Default) - Drive the robot
    Command driveCommand = OpCommands.getDriveCommand(drivebase, driverGamepad);
    drivebase.setDefaultCommand(driveCommand);
    sticksInUseTrigger(driverGamepad).whileTrue(driveCommand); // to interrupt other commands when the sticks are in use

    //L2 (disabled) - Gets the slow version (half speed) of the drive command. That way our robot can go slow.
    //driverGamepad.L2().whileTrue(OpCommands.getTemporarySlowSpeedCommand(drivebase));

    //Options - Zeros the robot heading
    driverGamepad.options().onTrue(Commands.runOnce(drivebase::zeroGyro));

    //L2 - Activate Auto Drive (while held)
    // Unlike all other commands, this "deferred" command is generated on command initialization, not instantiation.
    // In other words, this path-following command won't be generated until the command starts running.
    driverGamepad.L2().whileTrue(telePathingCommands.getAutoDriveDeferredCommand());

    //R2 - Pipe Outtake
    driverGamepad.R2().onTrue(pipeIntakeCommands.getAwareOuttakeCommand(intakePosition, intakePositionCommands));

    //Circle - Ball Outtake
    driverGamepad.circle().onTrue(ballIntakeCommands.getAwareOuttakeCommand(intakePosition, intakePositionCommands));

    // -- Pipe Intake --
    //L1 - Pipe Intake, Left Coral Station (set position and auto drive destination)
    driverGamepad.L1().onTrue(new ParallelCommandGroup(
      Commands.runOnce(() -> telePathingCommands.setAutoDriveSide(-1)),
      Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToCoralStation()),
      opCommands.getPipeIntakeFullCommand(pipeIntakeCommands)
    ));

    //R1 - Pipe Intake, Right Coral Station (set position and auto drive destination)
    driverGamepad.R1().onTrue(new ParallelCommandGroup(
      Commands.runOnce(() -> telePathingCommands.setAutoDriveSide(1)),
      Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToCoralStation()),
      opCommands.getPipeIntakeFullCommand(pipeIntakeCommands)
    ));
    




    //CODRIVER CONTROLS:

    //C2:R4 - Stow Position
    buttonPanel.button(10).onTrue(opCommands.getStowParallelCommand());

    //C1:R1-3 - Pipe Set Positions 2-4
    buttonPanel.button(1).onTrue(opCommands.pipeCommandGroup(4));
    buttonPanel.button(3).onTrue(opCommands.pipeCommandGroup(3));
    buttonPanel.button(13).onTrue(opCommands.pipeCommandGroup(2));

    //Gamepad:Dpad Down - Pipe Set Position 1
    coDriverGamepad.povDown().onTrue(opCommands.pipeCommandGroup(1));



    // -- Ball Set Positions --
    //C3:R1 - High Reef Ball
    buttonPanel.button(2).onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(4),
      ballIntakeCommands. new Intake(),
      new StowCommand(intakePosition)
    ));
    //C3:R2 - Low Reef Ball
    buttonPanel.button(4).onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(3),
      ballIntakeCommands. new Intake(),
      new StowCommand(intakePosition)
    ));

    //C3:R3 - Barge Shoot Position
    buttonPanel.button(5).onTrue(opCommands.bargeShootCommandGroup());

    //Gamepad:Dpad Left - Processor Ball
    coDriverGamepad.povLeft().onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(2),
      ballIntakeCommands. new Intake()
    ));

    //Gamepad:Dpad Up - Ground Ball
    coDriverGamepad.povUp().onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(1),
      ballIntakeCommands. new Intake(),
      new StowCommand(intakePosition)
    ));



    // -- Auto Drive Destinations --
    buttonPanel.button(16).onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveSide(-1)));
    buttonPanel.button(11).onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveSide(0)));
    buttonPanel.button(8).onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveSide(1)));

    buttonPanel.button(9).onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(0)));
    buttonPanel.button(14).onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(1)));
    buttonPanel.button(15).onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(2)));
    buttonPanel.button(12).onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(3)));
    buttonPanel.button(7).onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(4)));
    buttonPanel.button(6).onTrue(Commands.runOnce(() -> telePathingCommands.setAutoDriveGoToReef(5)));



    //Gamepad:PS+Options (hold for 0.1s) - Activate Then Move Hang
    //coDriverGamepad.PS().and(coDriverGamepad.options()).debounce(0.1).whileTrue(hangCommands.new Activate(HangConstants.kHangTwistPower, HangConstants.kHangUnlockPos));



    // -- Manual Control Overrides --
    // Reminder: the controller is placed upside-down.
    //Gamepad:R1 - Toggle Pipe Intake
    coDriverGamepad.R1().toggleOnTrue(pipeIntakeCommands.new Intake());
    //Gamepad:R2 - Toggle Pipe Outtake
    coDriverGamepad.R2().toggleOnTrue(pipeIntakeCommands.new Outtake());
    //Gamepad:L1 - Toggle Ball Intake
    coDriverGamepad.L1().toggleOnTrue(ballIntakeCommands.new Intake());
    //Gamepad:L2 - Toggle Ball Outtake
    coDriverGamepad.L2().toggleOnTrue(ballIntakeCommands.new Outtake());

    //Joysticks:Left - Manual Lift
    buttonPanel.axisMagnitudeGreaterThan(1, Constants.OIConstants.kDriveLargeDeadband)
            .whileTrue(intakePositionCommands.new AdjustLift(() -> -buttonPanel.getRawAxis(1)));
    //Joysticks:Right - Manual Pivot
    buttonPanel.axisMagnitudeGreaterThan(5, Constants.OIConstants.kDriveLargeDeadband)
            .whileTrue(intakePositionCommands.new AdjustPivot(() -> buttonPanel.getRawAxis(5)));
    
    //Gamepad:Triangle+Square (hold for 0.1s) - Zero Lift
    coDriverGamepad.triangle().and(coDriverGamepad.square()).debounce(0.1).onTrue(Commands.runOnce(intakePosition::zeroLift));

    //Gamepad:Circle - Unset Auto Drive
    coDriverGamepad.circle().onTrue(Commands.runOnce(telePathingCommands::setAutoDriveNone));

  }







  public Trigger sticksInUseTrigger(CommandPS5Controller gamepad) {
    return new Trigger(() -> Math.abs(gamepad.getLeftX()) > Constants.OIConstants.kDriveDeadband
                          || Math.abs(gamepad.getLeftY()) > Constants.OIConstants.kDriveDeadband
                          || Math.abs(gamepad.getRightX()) > Constants.OIConstants.kDriveDeadband);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected(); 
  }

  private static double normalDegrees(double deg) {
    double mod = deg % 360.0;
    if (mod < 0) mod += 360;
    return mod;
  }
  
  public void setAutoCommands(){
    autoChooser = AutoBuilder.buildAutoChooser();

    PIDController turnController = new PIDController(DriveConstants.kTurningP, DriveConstants.kTurningI, DriveConstants.kTurningD);
    turnController.setIZone(DriveConstants.kTurningIZone);
    turnController.enableContinuousInput(0, 360);
    turnController.setSetpoint(0);

    Command pathplannerless = new SequentialCommandGroup(
      new InstantCommand(() -> drivebase.resetOdometry(new Pose2d(drivebase.getPose().getTranslation(), Rotation2d.k180deg))),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        drivebase.centerModulesCommand()
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(10),
        opCommands.getPipe1Command(),
        drivebase.driveCommand(() -> 0.1, () -> 0.0, () -> -turnController.calculate(normalDegrees(drivebase.getSwerveDrive().getYaw().getDegrees())))
      ),
      pipeIntakeCommands.new Outtake()
    );

    Command pathplannerless2 = new SequentialCommandGroup(
      new InstantCommand(() -> drivebase.resetOdometry(new Pose2d(drivebase.getPose().getTranslation(), Rotation2d.k180deg))),
      new ParallelDeadlineGroup(
        new WaitCommand(0.5),
        drivebase.centerModulesCommand()
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(5),
        opCommands.getPipe4Command(),
        drivebase.driveCommand(() -> 0.2, () -> 0.0, () -> -turnController.calculate(normalDegrees(drivebase.getSwerveDrive().getYaw().getDegrees())))
      ),
      drivebase.driveDistanceCommand(new Translation2d(0, -0.5), 0.1),
      drivebase.driveDistanceCommand(new Translation2d(-0.1, 0), 0.1),
      pipeIntakeCommands.new Outtake()
    );

    autoChooser.addOption("PPLESS DRIVE FORWARD", pathplannerless);
    autoChooser.addOption("PPLESS L4 SCORE", pathplannerless2);

    //AutoChooser.addOption("1Coral-RedSide", new ParallelCommandGroup(new PathPlannerAuto("1Coral-RedSide"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("1Coral-RedSide").getStartingPose()))));
    //AutoChooser.addOption("1Coral-Center", new ParallelCommandGroup(new PathPlannerAuto("1Coral-Center"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("1Coral-Center").getStartingPose()))));
    //AutoChooser.addOption("1Coral-BlueSide", new ParallelCommandGroup(new PathPlannerAuto("1Coral-BlueSide"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("1Coral-BlueSide").getStartingPose()))));
    //AutoChooser.addOption("2Coral-RedSide", new ParallelCommandGroup(new PathPlannerAuto("2Coral-RedSide"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("2Coral-RedSide").getStartingPose()))));
    //AutoChooser.addOption("2Coral-BlueSide", new ParallelCommandGroup(new PathPlannerAuto("2Coral-BlueSide"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("2Coral-BlueSide").getStartingPose()))));
    //AutoChooser.addOption("2Coral-BlueSide", new ParallelCommandGroup(new PathPlannerAuto("2Coral-Center"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("2Coral-Center").getStartingPose()))));
    //AutoChooser.addOption("EmptyTestAuto", new ParallelCommandGroup(new PathPlannerAuto("EmptyTestAuto"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("EmptyTestAuto").getStartingPose()))));
    //AutoChooser.addOption("3Coral-RedSide", new PathPlannerAuto("3Coral-RedSide"));
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("Pipe Outtake", pipeIntakeCommands.new Outtake());
    NamedCommands.registerCommand("Pipe Level 1", opCommands.getPipe1Command());
    NamedCommands.registerCommand("Pipe Level 4", opCommands.getPipe4Command());
    NamedCommands.registerCommand("Pipe Retrieve", opCommands.getPipeIntakeCommand());
    NamedCommands.registerCommand("Pipe Intake", pipeIntakeCommands.new Intake());
    NamedCommands.registerCommand("L2 Group", opCommands.pipeCommandGroup(2));
    NamedCommands.registerCommand("L4 Group", opCommands.pipeCommandGroup(4));
    NamedCommands.registerCommand("Ball Level 2", opCommands.getBall2Command());
    NamedCommands.registerCommand("Stow", new StowCommand(intakePosition));
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void stopIntakePosition() {
    intakePosition.stopIntakePosition();
  }


  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("Algae Sensor?", ballIntake.getHasBall());
    SmartDashboard.putBoolean("Coral Sensor?", pipeIntake.getHasPipe());

    SmartDashboard.putBoolean("Lift at Target?", intakePosition.liftAtTargetPos());
    SmartDashboard.putBoolean("Pivot at Target?", intakePosition.pivotAtTargetPos());

    SmartDashboard.putNumber("Lift Encoder Position", intakePosition.getLiftPosition());
    SmartDashboard.putNumber("Lift Motor Power", intakePosition.m_liftMotor2.get());
    SmartDashboard.putNumber("Lift Sum Current Draw", intakePosition.m_liftMotor1.getOutputCurrent() + intakePosition.m_liftMotor2.getOutputCurrent());
    SmartDashboard.putNumber("Pivot Encoder Position", intakePosition.getPivotPosition());

    SmartDashboard.putNumber("Pigeon Oritentation", drivebase.pigeon.getAccumGyroZ().getValueAsDouble() % 360.0);

    SmartDashboard.putNumber("Lift target", intakePosition.getLiftSetpoint());
    SmartDashboard.putNumber("Pivot target", intakePosition.getPivotSetpoint());

    SmartDashboard.putNumberArray("Lift Currents", intakePosition.getCurrent());

    Pose2d fieldPos = drivebase.getPose();
    SmartDashboard.putNumber("Field X Position", fieldPos.getX());
    SmartDashboard.putNumber("Field Y Position", fieldPos.getY());
    SmartDashboard.putNumber("Field Heading", fieldPos.getRotation().getDegrees());

    int i=1;
    for (SwerveModule module : drivebase.swerveDrive.swerveDriveConfiguration.modules) {
      SmartDashboard.putNumber("module absolute "+i, module.getAbsoluteEncoder().getAbsolutePosition());
      SmartDashboard.putNumber("module angle "+i, module.getState().angle.getDegrees());
      SmartDashboard.putNumber("module offset "+i,Math.abs(module.getAbsoluteEncoder().getAbsolutePosition()-module.getState().angle.getDegrees()));
      i++;
    }
    
    /* double goToStow = SmartDashboard.getNumber("Go to stow", 0);
    if (goToStow > 0.001) dashboardStowCommand.schedule();
    SmartDashboard.putNumber("Go to stow", 0);
    SmartDashboard.putBoolean("Will go stow?", dashboardStowCommand.isScheduled()); */

    SmartDashboard.updateValues();
  }

  //private Command dashboardStowCommand = opCommands.getStowParallelCommand();
}
