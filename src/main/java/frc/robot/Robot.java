// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.OdometryThread;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  private final RobotContainer m_robotContainer; 
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
    boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?
    System.out.println("Target?: " + hasTarget);
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
    SwerveRequest.ApplyRobotSpeeds

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    OdometryThread.set
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



public Command getAuto(){
  RobotContainer.Swerve


  return null;
}
}