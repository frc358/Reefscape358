// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  //private final XboxController m_controller = new XboxController(3);
  //private final CommandSwerveDrivetrain m_swerve = new CommandSwerveDrivetrain.createDrivetrain();
  private final RobotContainer m_robotContainer; 
  //private double tx = LimelightHelpers.getTX("null");
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    System.out.println("X : " + x);
    System.out.println("Y : " + y);
    System.out.println("Area : " + area);
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    System.out.println("tx : " + tx);
    System.out.println("ty : " + ty);
    System.out.println("ta : " + ta);
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    // Basic targeting data
    double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
    boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?
    System.out.println("Target?: " + hasTarget);
    //LimelightHelpers.printPoseEstimate(LimelightHelpers.getBotPoseEstimate("limelight", "ty"));
    //System.out.println("Pose Y: " + LimelightHelpers.getTY("limelight"));
    System.out.println("Pose X: " + tx);
    System.out.println("Pipeline Type: " + LimelightHelpers.getCurrentPipelineType("limelight"));
    //LimelightHelpers.getDetectorClass("limelight");
    //LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults("limelight");
    //System.out.println("Please work: " + results.getBotPose3d());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

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