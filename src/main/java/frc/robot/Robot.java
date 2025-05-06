// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;

import java.time.format.TextStyle;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;


public class Robot extends TimedRobot {


  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer; 
  public static boolean obtainedAlliance = false;
  public boolean m_LimelightHasValidTarget = false;
  public static double tx;
  public static double ty;
  public boolean m_LimelightAlignedXLeft = false;
  public boolean m_LimelightAlignedXRight = false;
  public boolean m_LimelightAlignedY = false;
  public PoseEstimate mt2;
  private final Field2d m_field = new Field2d();
  private final Pigeon2 m_Pigeon2;

  public Robot() {
    m_robotContainer = new RobotContainer();
    LimelightHelpers.SetFiducialIDFiltersOverride("", new int[]{6,7,8,9,10,11,17,18,19,20,21,22}); // Only track these tag IDs
    SignalLogger.enableAutoLogging(false);
    // Do this in either robot or subsystem init
    SmartDashboard.putData("Field", m_field);
    m_Pigeon2 = new Pigeon2(1, TunerConstants.kCANBus);

  }

  @Override
  public void robotPeriodic() {

    if (!obtainedAlliance && DriverStation.isDSAttached() && DriverStation.isFMSAttached() && DriverStation.getAlliance().isPresent()){
      obtainedAlliance = true;
    }
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    //update boolean value before putting it on dashboard
    m_LimelightHasValidTarget = LimelightHelpers.getTV("limelight"); 
    SmartDashboard.putBoolean("AprilTag Target", m_LimelightHasValidTarget);
    tx = LimelightHelpers.getTX("limelight");
    ty = LimelightHelpers.getTY("limelight");

    SmartDashboard.putNumber("Pigeon Yaw", m_Pigeon2.getYaw().getValueAsDouble());
    
    //limelight stuff with pheonix swerve, localization
    var driveState = m_robotContainer.drivetrain.getState();
    double headingDeg = driveState.Pose.getRotation().getDegrees();
    SmartDashboard.putNumber("Heading Degrees", headingDeg); //testing by outputing value on elastic
    double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
    //LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
    mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    m_field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
    if (mt2 != null && mt2.tagCount > 0 && omegaRps < 2.0) {
      m_robotContainer.drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
      m_field.setRobotPose(mt2.pose); //sets robot pose for field sim 
    }
    
    double distance = limelight_calc_distance();
    SmartDashboard.putData(distance);
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
    if(Robot.isRedAlliance()) {
      m_Pigeon2.setYaw(180);
    }
  }

  @Override
  public void teleopPeriodic() {
  }

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

public static boolean isRedAlliance(){
  if (RobotBase.isReal()){
    return DriverStation.getAlliance().filter(value -> value == DriverStation.Alliance.Red).isPresent();
  }
  else return false; //False = blue, true = red
}

}