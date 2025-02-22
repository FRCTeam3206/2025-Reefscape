package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor =
          ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
      drivingConfig
          .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(0.04, 0, 0)
          .velocityFF(drivingVelocityFeedForward)
          .outputRange(-1, 1);

      turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
      turningConfig
          .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0); // radians per second
      turningConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class Algae {
    public static final SparkMaxConfig wheelsConfig = new SparkMaxConfig();
    public static final SparkMaxConfig armConfig = new SparkMaxConfig();

    static {
      wheelsConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(25);

      // Configure basic settings of the arm motor
      armConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(25).voltageCompensation(12);
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      armConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // Set PID values for position control
          .p(0.1)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(2000)
          .maxAcceleration(10000)
          .allowedClosedLoopError(0.25);

      armConfig
          .absoluteEncoder
          .positionConversionFactor(AlgaeConstants.kConversionFactor)
          .velocityConversionFactor(AlgaeConstants.kConversionFactor);
    }
  }

  public static final class CoralArm {
    public static final SparkMaxConfig coralArmConfig = new SparkMaxConfig();

    static {
      coralArmConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(5);

      coralArmConfig.encoder.positionConversionFactor(360).velocityConversionFactor(1);


      coralArmConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control. We don't need to pass a closed
          // loop slot, as it will default to slot 0.
          .p(0.4)
          .i(0)
          .d(0)
          .outputRange(-1, 1)
          // Set PID values for velocity control in slot 1
          .p(0.0001, ClosedLoopSlot.kSlot1)
          .i(0, ClosedLoopSlot.kSlot1)
          .d(0, ClosedLoopSlot.kSlot1)
          .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
          .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

      coralArmConfig
          .closedLoop
          .maxMotion
          // Set MAXMotion parameters for position control. We don't need to pass
          // a closed loop slot, as it will default to slot 0.
          .maxVelocity(1000)
          .maxAcceleration(1000)
          .allowedClosedLoopError(1)
          // Set MAXMotion parameters for velocity control in slot 1
          .maxAcceleration(500, ClosedLoopSlot.kSlot1)
          .maxVelocity(6000, ClosedLoopSlot.kSlot1)
          .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);

      // Constants.ElevatorConstants.Controller.Kp,
      //     Constants.ElevatorConstants.Controller.Ki,
      //     Constants.ElevatorConstants.Controller.Kd,
      //     new TrapezoidProfile.Constraints(
      //         ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration)
      // We will need to make a lot more changes to the config.
    }
  }
}
