// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.pathing.utils.AllianceUtil;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.PathingConstants.ReefPose;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private SendableChooser<Command> m_autonChooser = new SendableChooser<Command>();

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Arm m_arm = new Arm();
  private boolean m_fieldRelative = true;
  private boolean m_invertControls = true;
  private double m_speedMultiplier = 0.5;

  private double m_lastTime = 0;
  private double m_loopTime = 0;

  @NotLogged private Alliance m_prevAlliance = null;

  // The driver's controller
  CommandJoystick m_driverController = new CommandJoystick(OIConstants.kDriverControllerPort);
  CommandXboxController m_weaponsController =
      new CommandXboxController(OIConstants.kWeaponsControllerPort);

  public Robot() {
    AllianceUtil.setCustomFieldDesignType(false);

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    m_lastTime = Timer.getFPGATimestamp();

    DataLogManager.start();
    // Epilogue.bind(this);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    configureButtonBindings();
    configureDefaultCommands();
    autons();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_weaponsController.x().whileTrue(m_robotDrive.setXCommand());
    m_weaponsController.back().onTrue(new InstantCommand(() -> m_fieldRelative = !m_fieldRelative));
    m_weaponsController
        .a()
        .onTrue(m_robotDrive.runOnce(() -> m_robotDrive.zeroHeading(m_robotDrive.getPose())));
    m_weaponsController.start().onTrue(new InstantCommand(() -> resetRobotToFieldCenter()));

    // m_weaponsController.rightTrigger().whileTrue(m_robotDrive.getToNearestReefCommand(true));
    // m_weaponsController.leftTrigger().whileTrue(m_robotDrive.getToNearestReefCommand(false));
    // m_driverController.b().whileTrue(m_robotDrive.getToReefPoseCommand(ReefPose.CLOSE, true));
  }

  /** Use this method to define default commands for subsystems. */
  private void configureDefaultCommands() {
    m_robotDrive.setDefaultCommand(
        m_robotDrive.driveCommand(
            adjustJoystick(
                m_driverController::getY,
                () -> m_speedMultiplier,
                () -> m_invertControls || !m_fieldRelative),
            adjustJoystick(
                m_driverController::getX,
                () -> m_speedMultiplier,
                () -> m_invertControls || !m_fieldRelative),
            adjustJoystick(m_driverController::getTwist, () -> m_speedMultiplier, () -> true),
            () -> m_fieldRelative));
    // m_elevator.setDefaultCommand(m_elevator.stop());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (m_autonChooser.getSelected() == null) {
      return m_robotDrive.stopCommand();
    }
    return m_autonChooser.getSelected();
  }

  public void autons() {
    m_autonChooser.setDefaultOption("Nothing", m_robotDrive.stopCommand());
    m_autonChooser.setDefaultOption(
        "Basic Forward",
        m_robotDrive.driveCommand(() -> -0.3, () -> 0.0, () -> 0.0, () -> false).withTimeout(1.0));
    m_autonChooser.addOption(
        "Coral on the left",
        generateAuton(
            false,
            scoreCoralCommand(ReefPose.FAR_LEFT, false, 4),
            scoreCoralCommand(ReefPose.FAR_LEFT, true, 4),
            scoreCoralCommand(ReefPose.CLOSE_LEFT, false, 4),
            scoreCoralCommand(ReefPose.CLOSE_LEFT, true, 4),
            scoreCoralCommand(ReefPose.CLOSE, false, 4)));
    m_autonChooser.addOption("Processor", m_robotDrive.getToProcessorCommand());
    SmartDashboard.putData(m_autonChooser);
  }

  public Command generateAuton(boolean right, Command... scoreCoralCommands) {
    Command auton = robotForwardCommand().andThen(scoreCoralCommands[0]);
    for (int i = 1; i < scoreCoralCommands.length; i++) {
      auton = auton.andThen(pickupCoralCommand(right)).andThen(scoreCoralCommands[i]);
    }
    return auton;
  }

  /**
   * We can run this at the begining of any autonomous routine so that we first drive forward, to
   * make sure we don't hit the cages.
   */
  public Command robotForwardCommand() {
    return m_robotDrive.driveCommand(() -> 0.5, () -> 0.0, () -> 0.0, () -> false).withTimeout(.5);
  }

  /**
   * Pick up coral from the feeder station
   *
   * @param right Whether to pick up from the right or left feeder station (from driver view, since
   *     we have a rotated field).
   */
  public Command pickupCoralCommand(boolean right) {
    return m_robotDrive.getToFeederCommand(right);
    // We can add things with mechanisms later so that it will intake.
    // This command should stop once we see that we have the coral.
  }

  /**
   * Score coral on the reef.
   *
   * @param reefPose Which location on the reef to score it on.
   * @param right Whether it should be the right side on that face (from the robot's perspective).
   * @param level Which level to score the coral on.
   * @return A Command to score coral on the reef.
   */
  public Command scoreCoralCommand(ReefPose reefPose, boolean right, int level) {
    return m_robotDrive.getToReefPoseCommand(reefPose, right);
    // We can later add things with the mechanism to make it score the coral correctly.
  }

  /**
   * Apply desired adjustments to a joystick input, such as deadbanding and nonlinear transforms.
   *
   * @param input The input value from the joystick
   * @param negate Whether to invert the input
   * @return The adjusted value from the joystick
   */
  private DoubleSupplier adjustJoystick(
      DoubleSupplier input, DoubleSupplier multiplier, BooleanSupplier negate) {
    return () -> {
      double value = input.getAsDouble();
      if (negate.getAsBoolean()) {
        value = -value;
      }
      value = MathUtil.applyDeadband(value, OIConstants.kDriveDeadband);
      value = multiplier.getAsDouble() * value;
      return value;
    };
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    double now = Timer.getFPGATimestamp();
    m_loopTime = now - m_lastTime;
    m_lastTime = now;
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    AllianceUtil.setAlliance();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // if (!DriverStation.getAlliance().isEmpty()) {
    //   var alliance = DriverStation.getAlliance().get();
    //   m_invertControls = alliance.equals(Alliance.Blue);
    //   if (m_prevAlliance == null || !m_prevAlliance.equals(alliance)) {
    //     resetRobotToFieldCenter();
    //     m_prevAlliance = alliance;
    //   }
    // }
    // if (Robot.isSimulation()) {
    //   m_invertControls = true;
    // }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    m_arm.simulationPeriodic();
  }

  public double getLoopTime() {
    return m_loopTime;
  }

  public void resetRobotToFieldCenter() {
    var field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    var heading =
        (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red)
            ? 180.0
            : 0.0;
    m_robotDrive.zeroHeading();
    m_robotDrive.resetOdometry(
        new Pose2d(
            field.getFieldLength() / 2,
            field.getFieldWidth() / 2,
            Rotation2d.fromDegrees(heading)));
  }
}
