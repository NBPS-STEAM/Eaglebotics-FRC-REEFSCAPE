package frc.robot;



import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units; 

public final class Constants {

  public static final SparkBaseConfig kBrakeConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);

  public static final class OpConstantsForBall {
    public static final double Ball1Lift = 0.0; //GROUND BALL
    public static final double Ball1Pivot = 0.05; //CROSS (X)

    public static final double Ball2Lift = 1.75; //PROCESSOR BALL
    public static final double Ball2Pivot = 0.18; //SQUARE

    public static final double Ball3Lift = 1.75; //BALL LEVEL 1
    public static final double Ball3Pivot = 0.18; //TRIANGLE

    public static final double Ball4Lift = 4.75; //BALL LEVEL 2
    public static final double Ball4Pivot = 0.17; //CIRCLE
  }

  public static final class OpConstantsForPipe {
    public static final double Pipe1Lift = 0.65; //DDown
    public static final double Pipe1Pivot = 0.31; //TROUGH

    public static final double Pipe2Lift = 2.95; //DRight
    public static final double Pipe2Pivot = 0.21; //LEVEL 1

    public static final double Pipe3Lift = 5.91; //DUp
    public static final double Pipe3Pivot = 0.23; //LEVEL 2

    public static final double Pipe4Lift = 10.57; //DLeft
    public static final double Pipe4Pivot = 0.25; //HIGHEST

    public static final double PipeRetLift = 2.0;
    public static final double PipeRetPivot = 0.25;

    public static final double PipeIntakeLift = 2.53; //REDUNDENT WITH PipeRetLift
    public static final double PipeIntakePivot = 0.33; //REDUNDENT WITH PipeRetPivot
  }

  public static final class IntakeConstants {
    // Parameters related to the Ball Intake and Pipe Intake

    
    // Pipe Intake:

    // TODO: TUNE
    public static final double kPipeIntakeSpeed = 1;
    public static final double kPipeOuttakeSpeed = -1;

    // TODO: SET TO REAL MOTOR ID
    public static final int kPipeMotorId = 4; // CAN OK
    public static final int kPipeSensorChannel = 1; // CAN OK


    // Ball Intake:

    // TODO: TUNE
    public static final double kBallIntakeSpeed = 1;
    public static final double kBallOuttakeSpeed = -0.5;

    // TODO: SET TO REAL MOTOR ID
    public static final int kBallMotorId1 = 12; // CAN OK
    public static final int kBallMotorId2 = 13; // CAN OK
    public static final int kBallSensorChannel = 0; // CAN OK

  }

  public static final class IntakePositionConstants {
    public static final int kLiftMotor1Id = 5; // CAN OK (NO ENCODER, FOLLOWER MOTOR)
    public static final int kLiftMotor2Id = 11; // CAN OK (HAS ALTERNATE ENCODER, LEADING MOTOR)

    public static final double kLiftPosP = 1.5; // Used when the lift is going up (error is positive)
    public static final double kLiftNegP = 0.3; // Used when the lift is going down (error is negative)
    public static final double kLiftI = 0.0015;
    public static final double kLiftPosD = 0.0; // TODO: needs tuning
    public static final double kLiftNegD = 0.0; // TODO: needs tuning
    public static final double kLiftIZone = 0.5;
    public static final double kLiftTolerance = 0.05;
    public static final double kLiftAntigrav = 0.1775; // Antigrav constant: an amount of power added to the PID output to counteract gravity

    public static final int kPivotMotor1Id = 27; // CAN OK (HAS ABSOLUTE ENCODER)

    public static final double kPivotP = 2.5;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 0.0;
    public static final double kPivotIZone = 0.0;
    public static final double kPivotTolerance = 0.01;

    public static final double stowPivot = 0.32;
    public static final double stowLift = 0.2;
  }

  public static final class HangConstants {
    // Parameters related to the Hang Subsystem

    // TODO: TUNE
    public static final double kHangTwistPosition = 10.0;

    public static final double kHangP = 5e-5;
    public static final double kHangI = 0;
    public static final double kHangD = 0;
    public static final double kHangIz = 0;

    // TODO: SET TO REAL MOTOR ID
    public static final int kHangMotor1Id = 6; // CAN OK
    public static final int kHangMotor2Id = 7; // CAN OK
    public static final int kServo1Channel = 0; // CAN OK
    public static final int kServo2Channel = 1; // CAN OK
  }

  public static final class DriveConstants {
    //controls speed for telop
    public static double speedFactor = 0.5;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speedss
    public static  double kMaxSpeedMetersPerSecond = 5.6;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(32.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));


    // Angular offsets of the modules relative to the chassis in radians
    /* public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final double kTurningP = 1.0;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0; */

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1; // CAN OK ("swerve" canbus)
    public static final int kFrontLeftTurningCanId = 21; // CAN OK ("rio" canbus)
    public static final int kFrontLeftTurnEncoderCanId = 30; // CAN OK ("rio" canbus)

    public static final int kRearLeftDrivingCanId = 3; // CAN OK ("swerve" canbus)
    public static final int kRearLeftTurningCanId = 8; // CAN OK ("rio" canbus)
    public static final int kRearLeftTurnEncoderCanId = 33; // CAN OK ("rio" canbus)

    public static final int kFrontRightDrivingCanId = 2; // CAN OK ("swerve" canbus)
    public static final int kFrontRightTurningCanId = 20; // CAN OK ("rio" canbus)
    public static final int kFrontRightTurnEncoderCanId = 32; // CAN OK ("rio" canbus)

    public static final int kRearRightDrivingCanId = 4; // CAN OK ("swerve" canbus)
    public static final int kRearRightTurningCanId = 10; // CAN OK ("rio" canbus)
    public static final int kRearRightTurnEncoderCanId = 31; // CAN OK ("rio" canbus)

    public static final int kPigeonGyroCanId = 5; // CAN OK ("swerve" canbus)
    //public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 15;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians
    
    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = .1;
  }

  public static final class AutoConstants {
    public static final PIDConstants TRANSLATION_PID =new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants ANGLE_PID =new PIDConstants(5.0, 0.0, 0.0);
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }


  public static final class JoystickConstants {
    public static final int kXStick1 = 0;
    public static final int kYStick1 = 1;
    public static final int kLeftTrigger = 2;
    public static final int kRightTrigger = 3;
    public static final int kXStick2 = 4;
    public static final int kYStick2 = 5;

    public static final int kJoystick1Port = 0;
    public static final int kJoystick2Port = 1;
  }
  
  public static final double MAX_SPEED  = Units.feetToMeters(18.47);
}
