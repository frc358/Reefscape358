package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.Constants.AlgaeArmConstants;
import edu. wpi. first. math. trajectory. TrapezoidProfile.Constraints;
//import frc.robot.RobotMath.Elevator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.SignalLogger;
//import com.ctre.phoenix6.hardware.TalonFXConfiguration;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.robot.Constants.ElevatorConstants;

//import static au.grapplerobotics.interfaces.LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT;
import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {
    //setUp
    private final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);
    private final TalonFX e_motor;
    private final TalonFXSimState e_motorSim;
    //private final TalonFX m_encoder = m_motor.getEncoder();
    private StatusSignal elevatorMain;
    private final ProfiledPIDController m_controller = new ProfiledPIDController(ElevatorConstants.kElevatorKp,
            ElevatorConstants.kElevatorKi,
            ElevatorConstants.kElevatorKd,
            new Constraints(ElevatorConstants.kMaxVelocity,
                    ElevatorConstants.kMaxAcceleration));
    private final ElevatorFeedforward m_feedForward = new ElevatorFeedforward(ElevatorConstants.kElevatorkS,
            ElevatorConstants.kElevatorkG,
            ElevatorConstants.kElevatorkV,
            ElevatorConstants.kElevatorkA);
    private ElevatorSim m_elevatorSim = null;4


    private final SysIdRoutine elevatorSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1.0).per(Second),
                Volts.of(4.0),
                null,
                state -> SignalLogger.writeString("SysIdElevator_state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    e_motor.setControl(voltageRequest.withOutput(volts.in(Volts)));
                   },
                   log -> {
                    log.motor("Main")
                        .angularVelocity(
                            Rotations.per(Minute)
                                .of(e_motor.getVelocity().getValueAsDouble()))
                        .angularPosition(
                            Rotations.of(e_motor.getVelocity().getValueAsDouble()));   
                       },
                   this));   // Sensors
    /* 
    private final LaserCan m_elevatorLaserCan = new LaserCan(0);
    private final LaserCanSim m_elevatorLaserCanSim = new LaserCanSim(0);
    private final RegionOfInterest m_laserCanROI = new RegionOfInterest(0, 0, 16, 16);
    private final TimingBudget m_laserCanTimingBudget = TimingBudget.TIMING_BUDGET_20MS;
    private final Alert m_laserCanFailure = new Alert("LaserCAN failed to configure.",
            AlertType.kError);
    private final DigitalInput m_limitSwitchLow = new DigitalInput(9);
    private DIOSim m_limitSwitchLowSim = null;
*/

    public ElevatorSubsystem() {
        //TalonFXConfiguration config = new TalonFXConfiguration();
        //TalonFXConfiguration ElevatorInitialConfig = new TalonFXConfiguration();
        //ElevatorInitialConfig.withHardwareLimitSwitch
        e_motor.sensor()

        config.smartCurrentLimit(40)
                .openLoopRampRate(ElevatorConstants.kElevatorRampRate);
        
        e_motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);//

        if (RobotBase.isSimulation()) {
            m_elevatorSim = new ElevatorSim(m_elevatorGearbox,
                    ElevatorConstants.kElevatorGearing,
                    ElevatorConstants.kElevatorCarriageMass,
                    ElevatorConstants.kElevatorDrumRadius,
                    ElevatorConstants.kElevatorMinHeightMeters,
                    ElevatorConstants.kElevatorMaxHeightMeters,
                    true,
                    0.0,
                    0.02,
                    0.0);

            m_limitSwitchLowSim = new DIOSim(m_limitSwitchLow);//
            SmartDashboard.putData("Elevator Low limit Switch", m_limitSwitchLow);
        }

        try {
            m_elevatorLaserCanSim.setRangingMode(RangingMode.LONG);
        } catch (Exception e) {
            m_laserCanFailure.set(true);
        }

    }

    public void simulationPeriodic() {
        //set input(voltage)
        m_elevatorSim.setInput(e_motorSim.getSupplyCurrent() * RoboRioSim.getVInVoltage());

        //update-every 20 milliseconds
        m_elevatorSim.update(0.02);

        e_motorSim.iterate(Elevator.convertDistanceToRotations(Meters.of(m_elevatorSim.getVelocityMetersPerSecond()))
                        .per(Second).in(RPM),
                RoboRioSim.getVInVoltage(),
                0.020);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));


        //comment out for arm demo
        //Constants.elevatorMech.setLength(getPositionMeters());
        //Constants.elevatorCarriage.setPosition(AlgaeArmConstants.kAlgaeArmLength, getPositionMeters());
    }

    public double getPositionMeters() {
        return m_encoder.getPosition() * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius)
                / ElevatorConstants.kElevatorGearing;
    }

    public double getVelocityMetersPerSecond() {
        return (m_encoder.getVelocity() / 60) * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius)
                / ElevatorConstants.kElevatorGearing;
    }

    public void reachGoal(double goal){
        double voltsOutput = MathUtil.clamp(
                m_feedForward.calculateWithVelocities(getVelocityMetersPerSecond(), m_controller.getSetpoint().velocity)
                + m_controller.calculate(getPositionMeters(), goal),
                -7,
                7);
        e_motor.setVoltage(voltsOutput);
    }

    public Command setGoal(double goal){
        return run(() -> reachGoal(goal));
    }

    public Command setElevatorHeight(double height){
        return setGoal(height).until(()->aroundHeight(height));
    }

    public boolean aroundHeight(double height){
        return aroundHeight(height, ElevatorConstants.kElevatorDefaultTolerance);
    }
    public boolean aroundHeight(double height, double tolerance){
        return MathUtil.isNear(height,getPositionMeters(),tolerance);
    }

     /**
     * Stop the control loop and motor output.
     */
    public void stop()
    {
        e_motor.set(0.0);
    }

    /**
     * Update telemetry, including the mechanism visualization.
     */
    public void updateTelemetry()
    {
    }

    @Override
    public void periodic()
    {
    }
}
