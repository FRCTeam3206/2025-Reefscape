// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.util.Color;
import frc.pathing.robotprofile.Motor;
import frc.robot.Constants.GameConstants.ReefLevels;
import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(27.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(27.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final SwerveModuleState[] kStatesX =
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        };

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 11;
    public static final int kRearLeftTurningCanId = 13;
    public static final int kFrontRightTurningCanId = 12;
    public static final int kRearRightTurningCanId = 14;

    public static final boolean kGyroReversed = false;

    // These values need to be tuned.
    public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.5, 0.5, 0.1);

    public static final double kFastSpeed = 0.8;
    public static final double kSlowSpeed = 0.5;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class GameConstants {
    public static enum ReefLevels {
      l1,
      l2,
      l3,
      l4;
    }

    // all in meters
    // How high up each thing is
    public static final class Positions {
      // Made up some numbers that sound plausible
      public static final double kFeeder = 1.5;
      public static final double kFloorIntake = 0.1;
      public static final double kCoralStorage = 0.5;
      // l1 is the trough
      public static final double kReefL1 = 0.46;
      public static final double kReefL2 = 0.81;
      public static final double kReefL3 = 1.21;
      public static final double kReefL4 = 1.83;
    }
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kWeaponsControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera 1 (facing forward)
    public static final String kCamera1Name = "AprilTagCamera1";
    public static final Transform3d kRobotToCamera1 =
        new Transform3d(
            Units.inchesToMeters(3.84),
            Units.inchesToMeters(10.70),
            Units.inchesToMeters(10),
            new Rotation3d(0, 0, 0));

    // Camera 2 (facing back at an angle)
    public static final String kCamera2Name = "AprilTagCamera2";
    public static final Transform3d kRobotToCamera2 =
        new Transform3d(
            Units.inchesToMeters(3.84),
            Units.inchesToMeters(10.70),
            Units.inchesToMeters(10),
            new Rotation3d(0, Math.toRadians(-20), Math.toRadians(145)));

    // Camera 1
    public static final String kCameraLeftName = "CameraLeft";
    public static final Transform3d kRobotToCameraLeft =
        new Transform3d(
            Units.inchesToMeters(3.84),
            Units.inchesToMeters(10.70),
            Units.inchesToMeters(25.00),
            new Rotation3d(0, Math.toRadians(-20), Math.toRadians(145)));

    public static final String kCameraRightName = "CameraRight";
    public static final Transform3d kRobotToCameraRight =
        new Transform3d(
            Units.inchesToMeters(3.84),
            Units.inchesToMeters(10.70),
            Units.inchesToMeters(25.00),
            new Rotation3d(0, Math.toRadians(-20), Math.toRadians(215)));

    public static final String kCameraFrontName = "CameraFront";
    public static final Transform3d kRobotToCameraFront =
        new Transform3d(
            Units.inchesToMeters(3.84),
            Units.inchesToMeters(10.70),
            Units.inchesToMeters(25.00),
            new Rotation3d(0, 0, 0));

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final Matrix<N3, N1> kUntrustworthyStdDevs =
        VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  }

  public static final class VisionSimConstants {
    public static final int kCameraWidth = 1200;
    public static final int kCameraHeight = 720;
    public static final Rotation2d kDiagonalFOV = Rotation2d.fromDegrees(70);
    public static final double kAvgDetectionNoisePixels = 0.25;
    public static final double kStdDevDetectionNoisePixels = 0.08;

    public static final int kImageCaptureFPS = 20;
    public static final int kAvgLatencyMs = 35;
    public static final int kStdDevLatencyMs = 5;
  }

  public static final class AlgaeConstants {
    public static final int kArmCanId = 42;
    public static final int kWheelsCanId = 41;

    public static final DCMotor kArmMotorType = DCMotor.getNEO(1);
    public static final DCMotor kWheelsMotorType = DCMotor.getNeo550(1);

    public static final double kIntakeSpeed = .5;
    public static final double kExtakeSpeed = -0.5;
    public static final double kHoldUpVoltage = -.6;
    public static final double kHoldDownVoltage = 0.2;

    public static final double kRetractSpeed = -0.6;
    public static final double kExtendSpeed = 0.05;
    public static final double kRetractedAngle = 0.1;
    public static final double kExtendedAngle = 1;

    public static final double kArmProportional = 0.0;
    public static final double kArmDerivative = 0.0;
    public static final double kArmIntegral = 0.0;
    public static final double kRetractPosition = 0.0;
    public static final double kExtendPosition = 0.0;
    public static final double kAtGoalTolerance = 0.05;

    public static final double kConversionFactor = 0.05; // This is NOT the correct value yet.
  }

  public static final class ArmConstants {
    // public static final int gearing = 10;
    // public static final double kUpdateFrequency = 0.02;

    public static final int kArmCANId = 23;

    // Trapezoid profile constraints
    public static final double kMaxVelocity = 1.0; // raidans/second
    public static final double kMaxAcceleration = 1.0; // radians/second^2

    // Feedforward constants
    public static final double kS = 0; // volts
    public static final double kG = .5; // volts
    public static final double kV = 0; // volts*second/radian
    public static final double kA = 0; // volts*second^2/radian

    // Feedback constants
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kAtAngleTolerance = Units.degreesToRadians(2);
    public static final double kAtVelocityTolerance = Units.degreesToRadians(2);

    public static final class Angles {
      public static final double kHorizontal = Math.PI;
      public static final double kStored = 1.64; // Works in real life.
      public static final double kFloorIntake = -.25; // Works in real life.
      public static final double kFeeder = 0.98;
      public static final double kReefL1 = 0.98;
      public static final double kReefL2 = 0.98;
      public static final double kReefL3 = kReefL2;
      public static final double kReefL4 = 0.74;
      public static final double kSafePosition = 1.54;
    }
  }

  /** for {@link frc.robot.subsystems.Lights} */
  public static final class LightsConstants {
    public static final int kPort = 0;

    /**
     * how many seconds to sync in microseconds for some reason Same as 20ms which is the usual
     * command loop i think
     */
    public static final int kMicrosecondsSync = 20000;

    /** how many lights there are */
    public static final short kLength = 89;

    public static final short kBrightestColor = 255;

    /** hue that takes you all the way back to red, in degrees */
    public static final short kMaxHue = 180;

    public static final Distance kLEDSpacing = Meters.of(1 / 120.0);
    public static final LinearVelocity kScrollSpeed = MetersPerSecond.of(1);

    public static final Color kDefaultBlue = Color.fromHSV(120, 255, 100);
    public static final Color kAlignedGreen = Color.fromHSV(0, 255, 100);
    public static final Color kCoralRed = Color.fromHSV(60, 255, 100);
    public static final Color kClimbGreen = Color.fromHSV(175, 220, 100);
  }

  public static final class CoralConstants {
    // Random numbers right now
    public static final int kCANId = 25;
    // Might not end up being "Neo550", Line 157
    public static final DCMotor kCoralMotorType = DCMotor.getNeo550(1);
    public static final double kIntakeSpeed = -1;
    public static final double kOutakeSpeed = 1;
    public static final double kScoreSpeed = -1;

    public static final int kSensorChannel = 0;

    public class Finger {
      // Made up a number
      public static final int kChannel = 0;
      // How many turns the servo is
      public static final double kFreePosition = 0.8;
      public static final double kRetainedPosition = 1.0;
    }

    public static final double kSafeScoreTime = 2;
  }

  public static final class ElevatorConstants {
    // TODO many of these arent used
    // weight in kg for the simulation, idk what counts as part of the elevator and what doesnt
    public static final double kWeight = 2;
    // voltage for the simulation
    public static final double kVoltage = 1;
    // max elevator speed
    public static final double kMaxVelocity = 2.45;
    // in seconds
    public static final double kUpdateFrequency = 0.02;

    public static final class Motor {
      public static final int kCanIdMotor1 = 21;
      public static final int kCanIdMotor2 = 22;
      /*between -1 and 1
      https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/motorcontrol/MotorController.html#set(double)
      Make it negative to reverse it*/
      public static final double kSpeed = 0.1;
      // how many motors in gearbox
      public static final int kHowManyInGearbox = 2;
    }

    // idk much about electricity so this is just made up numbers
    public static final class Voltages {
      public static final double kDown = -5;
      public static final double kUp = 40;
    }

    // something for the simulation
    public static final class Mechanism2d {
      // meters i think
      public static final double kWidth = 20;
      public static final double kHeight = 50;
      public static final double kXDistance = 10;
      public static final double kYDistance = 0;
    }

    public static final class Measurements {
      // meters
      public static final double kBottomHeight = 0;
      public static final double kTopHeight = 1.25;
      // how close it should be to the goal when the motors start slowing down
      // TODO redo this with feed forward because it's better
      public static final double kSlowDownDistance = 0.5;

      // Radius of the drum of the elevator
      // Controls the conversion factor of the SparkMax motors
      // This value is multiplied by 2pi for the circumference, then passed in as a factor
      // multiplying the rotations
      public static final double kDrumRadius = 0.0508;
      // Gearing of the gearbox on elevator (Positive values = reduction)
      public static final double kGearing = 10;
      // Standard deviation of elevator sim (set to 0 for no noise)
      public static final double[] kStandardDeviation =
          new double[] {
            0, 0,
          };
    }

    public static final class Encoder {
      // Change this later idk what it is
      public static final int kAChannel = 0;
      public static final int kBChannel = 1;
    }

    // ways the elevator can go
    // It aint really needed i just learned what an enum is and watned 2 use it
    public static enum WaysItCanMove {
      down,
      up,
      nowhere
    }
  }

  public static final class WristConstants {
    public static final int kCANId = 24;
    public static final double kHorizontalPosition = 3.1;
    public static final double kVerticalPosition = 3 * Math.PI / 2;

    public static final double kAtAngleTolerance = 0.15;

    public static final double kP = 0.25;
    public static final double kI = 0.0;
    public static final double kD = 0.02;

    public static final double kV = 0.1;

    public static final double kMaxVelocity = 1;
    public static final double kMaxAcceleration = 1;
  }

  public static final class PathingConstants {
    // Tolerances
    public static final double kTranslationTolerance = 0.01;
    public static final double kRotationTolerance = Math.toRadians(1);
    public static final double kVelocityTolerance = 0.04;
    public static final double kRotVelocityTolerance = Math.toRadians(7);

    // Safety multipliers
    public static final double kVelocitySafety = 1;
    public static final double kAccelSafety = 0.35;
    public static final double kRotVelocitySafety = 1;
    public static final double kRotAccelSafety = 1;

    private static final AprilTagFieldLayout kTagLayoutLoaded =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private static final List<Integer> kUsedIds =
        List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
    private static final List<AprilTag> kUsedTags = List.of();

    {
      for (AprilTag tag : kTagLayoutLoaded.getTags()) {
        if (kUsedIds.contains(tag.ID)) {
          kUsedTags.add(tag);
        }
      }
    }

    public static final AprilTagFieldLayout kTagLayout =
        new AprilTagFieldLayout(
            kUsedTags, kTagLayoutLoaded.getFieldLength(), kTagLayoutLoaded.getFieldWidth());

    public static Pose2d poseForTag(int tag) {
      return kTagLayoutLoaded.getTagPose(tag).get().toPose2d();
    }

    public static final double kRobotMassKg = 63.5;
    public static final double kRobotLengthWidthMeters =
        Units.inchesToMeters(36); // including bumpers. Length and width are the same.
    public static final double kCoralFaceOffset =
        Units.inchesToMeters(13)
            / 2; // page 24: pipes on the same face are 1 ft. 1 in. apart (center to center)
    public static final Motor kDriveMotor = Motor.NEO().gear(4.71);

    public static final double kAlignOffsetInches = 8.0;

    public static final Transform2d kTransformLeft =
        new Transform2d(
            kRobotLengthWidthMeters / 2 + Units.inchesToMeters(kAlignOffsetInches),
            -kCoralFaceOffset,
            Rotation2d.fromDegrees(180));
    public static final Transform2d kTransformRight =
        new Transform2d(
            kRobotLengthWidthMeters / 2 + Units.inchesToMeters(kAlignOffsetInches),
            kCoralFaceOffset,
            Rotation2d.fromDegrees(180));

    public static final double kReefCenterX = Units.inchesToMeters((144.0 + 209.49) / 2);

    // Measurements taken from April Tag coordinates
    public static enum ReefPose {
      CLOSE(18),
      CLOSE_LEFT(19),
      CLOSE_RIGHT(17),
      FAR(21),
      FAR_LEFT(20),
      FAR_RIGHT(22);

      private Pose2d leftPose;
      private Pose2d rightPose;

      private ReefPose(int tag) {
        Pose2d reefPose = poseForTag(tag);
        this.leftPose = reefPose.transformBy(kTransformLeft);
        this.rightPose = reefPose.transformBy(kTransformRight);
      }

      public Pose2d getPose(boolean right) {
        return right ? rightPose : leftPose;
      }
    }

    public static Pose2d poseFromTag(int tag, Transform2d transform) {
      return poseForTag(tag).plus(transform);
    }

    public static final double kFeederLength = 1.975; // Approximation from CAD, in meters
    public static final Transform2d kFeederTransform =
        new Transform2d(
            kRobotLengthWidthMeters / 2,
            -(kFeederLength / 2.0 - kRobotLengthWidthMeters / 2.0),
            Rotation2d.fromDegrees(180));
    public static final Pose2d kLeftFeederPose = poseFromTag(13, kFeederTransform);
    public static final Pose2d kRightFeederPose = poseFromTag(12, kFeederTransform);

    public static final Transform2d kProcessorTransform =
        new Transform2d(kRobotLengthWidthMeters / 2, 0.0, Rotation2d.fromDegrees(0));
    public static final Pose2d kProcessorPose = poseFromTag(16, kProcessorTransform);

    public static final double kStartLineInches = 144 + 65.49 + 88;
    public static final Pose2d kCenterStartPose =
        new Pose2d(
            Units.inchesToMeters(kStartLineInches - 11.875) + kRobotLengthWidthMeters / 2.0,
            kTagLayoutLoaded.getFieldWidth() / 2,
            Rotation2d.fromDegrees(180));
    public static final Pose2d kLeftStartPose =
        new Pose2d(
            Units.inchesToMeters(kStartLineInches - 11.875) + kRobotLengthWidthMeters / 2.0,
            kTagLayoutLoaded.getFieldWidth()
                - kRobotLengthWidthMeters / 2
                - Units.inchesToMeters(4.5),
            Rotation2d.fromDegrees(180));
    public static final Pose2d kRightStartPose =
        new Pose2d(
            Units.inchesToMeters(kStartLineInches - 11.875) + kRobotLengthWidthMeters / 2.0,
            kRobotLengthWidthMeters / 2.0 + Units.inchesToMeters(4.5),
            Rotation2d.fromDegrees(180));

    // new Pose2d(poseForTag(14).getX(), (poseForTag(14).getY() + poseForTag(15).getY()) / 2.0,
    // Rotation2d.fromDegrees(180))
    // .plus(new Transform2d(kRobotLengthWidthMeters / 2.0, 0, new Rotation2d()));

    public static enum NumCoralAuton {
      k1Coral,
      k2Coral,
      k3Coral,
      k4Coral;
    }
  }

  public static final class ArmSubConstants {
    public static final double kMaxVelocity = 4.0;
    public static final double kMaxAcceleration = 16.0;

    public static final double kS = 0.0;
    public static final double kG = .9;
    public static final double kV = 0.5;
    public static final double kA = 0.054;

    public static final double kP = 4.0;
    public static final double kI = 0.0;
    public static final double kD = 0.04;

    public static final double kMaxAngle = Units.degreesToRadians(150);
    public static final double kMinAngle = Units.degreesToRadians(30);

    public static final double kSafeWrist = 1.38;

    public static final double kArmReduction = 25;
    public static final double kArmPivotHeight = Units.inchesToMeters(10.5);
    public static final double kArmLength = Units.inchesToMeters(13.5);
    public static final double kArmMass = 2.78; // kg
    public static final double kArmMOI = 0.1; // 0.395; // kg*m² - estimated from CAD
  }

  public static class ElevatorSubConstants {
    public static final double kPosFactor =
        3
            * 22.0
            * Units.inchesToMeters(.25)
            / 5.0; // 5 rotations is 1 rotation of sprocket, sprocket is 22 teeth each 1/4 inch
    public static final double kVelocityFactor = kPosFactor / 60.0;

    public static final double kMaxVelocity = 50.0 * kPosFactor;
    public static final double kMaxAcceleration = 50.0 * kPosFactor;

    public static final double kS = .3;
    public static final double kG = 1.3;
    public static final double kV = .15 / kPosFactor;
    public static final double kA = .02 / kPosFactor;

    public static final double kP = 32.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kMaxHeight = 1;
    public static final double kMinHeight = 0;

    public static final double kArmReduction = 25;
    public static final double kArmPivotHeight = Units.inchesToMeters(10.5);
    public static final double kArmLength = Units.inchesToMeters(13.5);
    public static final double kArmMass = 2.78; // kg
    public static final double kArmMOI = 0.1; // 0.395; // kg*m² - estimated from CAD

    public static final double kPosOffset = Units.inchesToMeters(11);
    public static final double kL2Pos = 0.40; // .78;
    public static final double kL3Pos = 0.82;
    public static final double kL4Pos = 1.46;

    public static final double getGoalForLevel(ReefLevels level) {
      switch (level) {
        case l1:
          return 0.0;
        case l2:
          return kL2Pos;
        case l3:
          return kL3Pos;
        case l4:
          return kL4Pos;
        default:
          return 0.0;
      }
    }

    public static final double kFeederPos = 0.32;

    public static final double kAtGoalTolerance = 0.04;
  }

  public static class ClimberConstants {
    public static final int kClimberCanId = 43;
    public static final double kMinLimit = 0;
    public static final double kClimbMin = 0.38;
    public static final double kMaxLimit = 1.57;
    // public static final double kMaxLimit = 355;
  }
}
