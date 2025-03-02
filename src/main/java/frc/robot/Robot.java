// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  //private final XboxController m_controller = new XboxController(3);
  private final RobotContainer m_robotContainer; 
  private final Timer m_timer = new Timer();
 // private final CommandSwerveDrivetrain m_swerveDrivetrain;
  private final boolean kUseLimelight = false;
  //private final SwerveDrivetrain m_swerveDrivetrain = new SwerveDrivetrain(TunerSwerveDrivetrain.);

  public Robot() {


    double startTime = Timer.getFPGATimestamp();
    DataLogManager.start();
    double codeRunTime = (Timer.getFPGATimestamp() - startTime) * 1000;
    SmartDashboard.putNumber("Code Run Time (ms)", codeRunTime);
    SmartDashboard.putNumber("Match Time", codeRunTime);
    DataLogManager.log("Startup Time: " + startTime * 1000);
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();


    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
      }
    }
  }

  @Override
  public void disabledInit() {
    System.gc();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}


  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

double limelight_aim_proportional()
{    
  // kP (constant of proportionality)
  // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
  // if it is too high, the robot will oscillate.
  // if it is too low, the robot will never reach its target
  // if the robot never turns in the correct direction, kP should be inverted.
  double kP = .035;

  // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
  // your limelight 3 feed, tx should return roughly 31 degrees.
  double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

  // convert to radians per second for our drive method -- custom math by Jacob
  //targetingAngularVelocity *= TunerConstants.DrivetrainConstants.LinearVelocity.kSpeedAt12Volts / get(LimelightHelpers.RawFiducial.distToRobot);

  //invert since tx is positive when the target is to the right of the crosshair
  targetingAngularVelocity *= -1.0;

  return targetingAngularVelocity;
}

// simple proportional ranging control with Limelight's "ty" value
// this works best if your Limelight's mount height and target mount height are different.
// if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
double limelight_range_proportional()
{    
  double kP = .1;
  double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
  targetingForwardSpeed *= SwerveModule.DriveRequestType.Velocity.value;
  targetingForwardSpeed *= -1.0;
  return targetingForwardSpeed;
  }
}