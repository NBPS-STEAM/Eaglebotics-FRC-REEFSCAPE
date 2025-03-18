// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePositionConstants;
import frc.robot.commands.oldordrivecommands.AutoCommands.WaitCommand;
import frc.robot.commands.oldordrivecommands.ScoreCommands.BallIntakeCommands;
import frc.robot.commands.oldordrivecommands.ScoreCommands.HangCommands;
import frc.robot.commands.oldordrivecommands.ScoreCommands.IntakePositionCommand;
import frc.robot.commands.oldordrivecommands.ScoreCommands.OpCommands;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.commands.oldordrivecommands.ScoreCommands.PipeIntakeCommands;
import frc.robot.commands.oldordrivecommands.ScoreCommands.StowCommand;
import frc.robot.subsystems.BallIntakeSubsystem;
import frc.robot.subsystems.HangSubsystem;
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
{
  // The robot's subsystems and commands are defined here...
  public final VisionSubsystem vision=new VisionSubsystem();
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  public final PipeIntakeSubsystem pipeIntake = new PipeIntakeSubsystem();
  public final IntakePositionSubsystem intakePosition = new IntakePositionSubsystem();

  public final BallIntakeSubsystem ballIntake = new BallIntakeSubsystem();
  public final HangSubsystem hangSubsystem = new HangSubsystem();

  PipeIntakeCommands pipeIntakeCommands = new PipeIntakeCommands(pipeIntake);
  BallIntakeCommands ballIntakeCommands = new BallIntakeCommands(ballIntake);
  IntakePositionCommand intakePositionCommands = new IntakePositionCommand(intakePosition);
  HangCommands hangCommands = new HangCommands(hangSubsystem);
  OpCommands opCommands = new OpCommands(intakePosition);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandPS5Controller driverGamepad = new CommandPS5Controller(0);
  final CommandPS5Controller coDriverGamepad = new CommandPS5Controller(1);

  SendableChooser<Command> AutoChooser = new SendableChooser<>();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Register commands for PathPlanner
    registerNamedCommands();

    // Configure the trigger bindings
    //configureBindings1(); // Set positions only (no command groups for IntakePosition set positions)
    configureBindings2(); // Sequential command groups for IntakePosition set positions

    setAutoCommands();
    
    SmartDashboard.putData("Autos", AutoChooser);
  }
  

  /**
   * Version 1: Set positions only (no command groups for IntakePosition set positions)
   * 
   * <p>Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.</p>
   */
  private void configureBindings1()
  {
    drivebase.setDefaultCommand(OpCommands.getDriveCommand(drivebase, driverGamepad));
    //Gets the slow version (half speed) of the drive command. That way our robot can go slow.
    driverGamepad.L2().whileTrue(OpCommands.getTemporarySlowSpeedCommand(drivebase));
    driverGamepad.circle().onTrue(Commands.runOnce(drivebase::zeroGyro));

    /* driverGamepad.L1().and(driverGamepad.R1()).onTrue(Commands.runOnce(intakePosition::zeroLift));
    driverGamepad.povUp().whileTrue(intakePositionCommands.new SetLiftVelocity(0.5));
    driverGamepad.povDown().whileTrue(intakePositionCommands.new SetLiftVelocity(-0.5));
    driverGamepad.povRight().whileTrue(Commands.run(() -> intakePosition.offsetPivotSetpoint(0.5)));
    driverGamepad.povLeft().whileTrue(Commands.run(() -> intakePosition.offsetPivotSetpoint(-0.5))); */
    

    //Co Driver:
    
    // Pipe Intake/Outtake/Stop Controls
    coDriverGamepad.L1().onTrue(pipeIntakeCommands.new Intake());
    //coDriverGamepad.L1().onTrue(opCommands.getPipeIntakeCommand());
    coDriverGamepad.L2().onTrue(pipeIntakeCommands.new Outtake());
    coDriverGamepad.L3().onTrue(pipeIntakeCommands.new StopIntake());
    
    // Ball Intake/Outtake/Stop Controls
    coDriverGamepad.R1().onTrue(ballIntakeCommands.new Intake());
    coDriverGamepad.R2().onTrue(ballIntakeCommands.new Outtake());
    coDriverGamepad.R3().onTrue(ballIntakeCommands.new StopIntake());
    
  
    // Ball Set Positions
    coDriverGamepad.cross().onTrue(opCommands.getBall1Command());
    coDriverGamepad.circle().onTrue(opCommands.getBall2Command());
    coDriverGamepad.square().onTrue(opCommands.getBall3Command());
    coDriverGamepad.triangle().onTrue(opCommands.getBall4Command());

    // Pipe Set Positions
    coDriverGamepad.povDown().onTrue(opCommands.getPipe1Command());
    coDriverGamepad.povLeft().onTrue(opCommands.getPipe2Command());
    coDriverGamepad.povRight().onTrue(opCommands.getPipe3Command());
    coDriverGamepad.povUp().onTrue(opCommands.getPipe4Command());


    coDriverGamepad.options().onTrue(opCommands.getStowParallelCommand());
    coDriverGamepad.PS().onTrue(opCommands.getPipeIntakeCommand());
  }


  /**
   * Version 2: Sequential command groups for IntakePosition set positions
   * 
   * <p>Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.</p>
   */
  private void configureBindings2()
  {
    drivebase.setDefaultCommand(OpCommands.getDriveCommand(drivebase, driverGamepad));
    //Gets the slow version (half speed) of the drive command. That way our robot can go slow.
    driverGamepad.L2().whileTrue(OpCommands.getTemporarySlowSpeedCommand(drivebase));
    driverGamepad.options().onTrue(Commands.runOnce(drivebase::zeroGyro));

    driverGamepad.cross().whileTrue(OpCommands.getAutoTurnDriveCommand(drivebase, driverGamepad, 0));
    driverGamepad.circle().whileTrue(OpCommands.getAutoTurnDriveCommand(drivebase, driverGamepad, 60));
    driverGamepad.R1().whileTrue(OpCommands.getAutoTurnDriveCommand(drivebase, driverGamepad, 120));
    driverGamepad.triangle().whileTrue(OpCommands.getAutoTurnDriveCommand(drivebase, driverGamepad, 180));
    driverGamepad.L1().whileTrue(OpCommands.getAutoTurnDriveCommand(drivebase, driverGamepad, 240));
    driverGamepad.square().whileTrue(OpCommands.getAutoTurnDriveCommand(drivebase, driverGamepad, 300));
    

    //Co Driver:

    // Manual lift and pivot controls
    coDriverGamepad.R1().debounce(0.5).onTrue(Commands.runOnce(intakePosition::zeroLift));
    new Trigger(() -> Math.abs(coDriverGamepad.getLeftY()) > Constants.OIConstants.kDriveDeadband)
            .whileTrue(intakePositionCommands.new AdjustPivot(() -> -coDriverGamepad.getLeftY(), null));
    new Trigger(() -> Math.abs(coDriverGamepad.getRightY()) > Constants.OIConstants.kDriveDeadband)
            .whileTrue(intakePositionCommands.new AdjustLift(() -> -coDriverGamepad.getRightY(), null));
    
    // Pipe Intake/Outtake/Stop Controls
    coDriverGamepad.L1().onTrue(new SequentialCommandGroup(
      opCommands.getPipeIntakeCommand(),
      pipeIntakeCommands. new Intake(),
      new StowCommand(intakePosition)
    ));

    coDriverGamepad.L2().onTrue(pipeIntakeCommands.getAwareOuttakeCommand(intakePosition, intakePositionCommands));
    //coDriverGamepad.L2().onTrue(intakeNormal);
    
    
    // Ball Intake/Outtake/Stop Controls
    Command normalBallOuttake = new SequentialCommandGroup(
      ballIntakeCommands.new Outtake(),
      new StowCommand(intakePosition)
    );

    Command bargeOuttake = new ParallelCommandGroup(
      ballIntakeCommands.new Outtake(IntakeConstants.kBallOuttakeBargeSpeed),
      new InstantCommand(() -> intakePosition.setPivotSetpoint(IntakePositionConstants.bargePivotShove, 5))
    );

    coDriverGamepad.R2().onTrue(new ConditionalCommand(
      bargeOuttake,
      normalBallOuttake,
      () -> intakePosition.getPositionLevel() == 5
    ));

    // Stow Position
    coDriverGamepad.options().onTrue(opCommands.getStowParallelCommand());
  
    // Barge Shoot Position
    coDriverGamepad.PS().onTrue(opCommands.bargeShootCommandGroup());
  
    // Hang Control
    //coDriverGamepad.PS().whileTrue(new HangCommands.Activate(hangSubsystem, 1.0, 1.0));
  
    // Ball Set Positions
    coDriverGamepad.cross().onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(1),
      ballIntakeCommands. new Intake(),
      new StowCommand(intakePosition)
    ));
    coDriverGamepad.circle().onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(2),
      ballIntakeCommands. new Intake(),
      new StowCommand(intakePosition)
    ));
    coDriverGamepad.square().onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(3),
      ballIntakeCommands. new Intake(),
      new StowCommand(intakePosition)
    ));
    coDriverGamepad.triangle().onTrue(new SequentialCommandGroup(
      opCommands.ballCommandGroup(4),
      ballIntakeCommands. new Intake(),
      new StowCommand(intakePosition)
    ));

    // Pipe Set Positions
    coDriverGamepad.povDown().onTrue(opCommands.pipeCommandGroup(1));
    coDriverGamepad.povLeft().onTrue(opCommands.pipeCommandGroup(2));
    coDriverGamepad.povRight().onTrue(opCommands.pipeCommandGroup(3));
    coDriverGamepad.povUp().onTrue(opCommands.pipeCommandGroup(4));

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AutoChooser.getSelected(); 
  }

  private static double normalDegrees(double deg) {
    double mod = deg % 360.0;
    if (mod < 0) mod += 360;
    return mod;
}
  
  public void setAutoCommands(){
    PIDController turnController = new PIDController(DriveConstants.kTurningP, DriveConstants.kTurningI, DriveConstants.kTurningD);
    turnController.setIZone(DriveConstants.kTurningIZone);
    turnController.enableContinuousInput(0, 360);
    turnController.setSetpoint(0);

    Command pathplannerless = new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        drivebase.centerModulesCommand()
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(10),
        opCommands.getPipe1Command(),
        drivebase.driveCommand(() -> -0.1, () -> 0.0, () -> -turnController.calculate(normalDegrees(drivebase.getSwerveDrive().getYaw().getDegrees())))
      ),
      new InstantCommand(() -> drivebase.resetOdometry(new Pose2d(0, 0, Rotation2d.k180deg))),
      pipeIntakeCommands.new Outtake()
    );

    AutoChooser.addOption("PATHPLANNERLESS DRIVE FORWARD", pathplannerless);

    AutoChooser.addOption("1Coral-RedSide", new ParallelCommandGroup(new PathPlannerAuto("1Coral-RedSide"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("1Coral-RedSide").getStartingPose()))));
    AutoChooser.addOption("1Coral-Center", new ParallelCommandGroup(new PathPlannerAuto("1Coral-Center"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("1Coral-Center").getStartingPose()))));
    AutoChooser.addOption("1Coral-BlueSide", new ParallelCommandGroup(new PathPlannerAuto("1Coral-BlueSide"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("1Coral-BlueSide").getStartingPose()))));
    AutoChooser.addOption("2Coral-RedSide", new ParallelCommandGroup(new PathPlannerAuto("2Coral-RedSide"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("2Coral-RedSide").getStartingPose()))));
    AutoChooser.addOption("2Coral-BlueSide", new ParallelCommandGroup(new PathPlannerAuto("2Coral-BlueSide"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("2Coral-BlueSide").getStartingPose()))));
    AutoChooser.addOption("2Coral-BlueSide", new ParallelCommandGroup(new PathPlannerAuto("2Coral-Center"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("2Coral-Center").getStartingPose()))));
    //AutoChooser.addOption("3Coral-RedSide", new PathPlannerAuto("3Coral-RedSide"));
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("Pipe Outtake", pipeIntakeCommands.new Outtake());
    NamedCommands.registerCommand("Pipe Level 4", opCommands.getPipe1Command());
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
