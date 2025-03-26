package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.subsystems.Swerve;
import frc.robot.util.AllianceUtil;
import frc.robot.LimelightHelpers;


public class DriveDistance extends Command { 
    private final CommandSwerveDrivetrain swerve;
    private double targetX, targetY;
    private Pose2d target;
    private final PIDController driveController;
    double distance;
    private SwerveRequest.FieldCentric fieldOriented =
        new SwerveRequest.FieldCentric()
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withSteerRequestType(SteerRequestType.Position);
   

    public DriveDistance(CommandSwerveDrivetrain swerve){
        this.swerve = swerve;
        driveController = new PIDController(7,0,.5);
        driveController.enableContinuousInput(0, 50);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        distance = swerve.getDistance();
        target = AllianceUtil.getReefPose();
        targetX = target.getX();
        targetY = target.getY();
    }

    @Override
    public void execute(){
        swerve.setControl(fieldOriented.withVelocityX(.1).withVelocityX(0).withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        
        return Math.abs(driveController.getError()) < 1;
    }

    @Override
    public void end(boolean interrupted){
        driveController.reset();
        swerve.setControl(fieldOriented.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
    
}
