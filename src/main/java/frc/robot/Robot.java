package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Robot extends TimedRobot {

  private final XboxController m_driverController = new XboxController(0);

  // Motors (CIM → Brushed SparkMAX)
  private final SparkMax m_leftLeader  = new SparkMax(2, SparkMax.MotorType.kBrushed);
  private final SparkMax m_leftFollower = new SparkMax(3, SparkMax.MotorType.kBrushed);

  private final SparkMax m_rightLeader = new SparkMax(0, SparkMax.MotorType.kBrushed);
  private final SparkMax m_rightFollower = new SparkMax(1, SparkMax.MotorType.kBrushed);

  private DifferentialDrive m_drive;

  // Simulation
  private DifferentialDrivetrainSim m_driveSim;
  private Field2d m_fieldSim;

  public Robot() {

    // ---------------- MOTOR CONFIG ----------------
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

    // Follower setup
    leftFollowerConfig.follow(m_leftLeader);
    rightFollowerConfig.follow(m_rightLeader);

    // Apply config
    m_leftLeader.configure(leftConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    m_rightLeader.configure(rightConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    m_leftFollower.configure(leftFollowerConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    m_rightFollower.configure(rightFollowerConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    // DifferentialDrive
    m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);
    m_drive.setDeadband(0.05);

    // --------------- SIM SETUP ----------------
    m_driveSim = new DifferentialDrivetrainSim(
        DCMotor.getCIM(2),         // 2 CIMs per side
        10.71,                     // ToughBox Mini ratio
        7.5,                       // MOI estimate
        50.0,                      // mass kg
        Units.inchesToMeters(3),   // wheel radius
        Units.inchesToMeters(28),  // track width
        VecBuilder.fill(
            0.005, 0.005, 0.001,
            0.05, 0.05,
            0.005, 0.005)
    );

    m_fieldSim = new Field2d();
    SmartDashboard.putData("Field", m_fieldSim);
  }

  // ---------------AUTONOMOUS MODE----------------
  @Override
  public void autonomousPeriodic() {
    // Simple autonomous: drive forward at half speed
    m_drive.arcadeDrive(0.5, 0.0);
  }

  // ---------------- TELEOP DRIVE ----------------
  @Override
  public void teleopPeriodic() {

    double forward  = -m_driverController.getLeftY();
    double turn    = -m_driverController.getRightX();

    m_drive.arcadeDrive(forward, turn);
  }

  // --------------- SIMULATION LOOP ---------------
  @Override
  public void simulationPeriodic() {

    // Duty cycle → voltage
    double leftVolts  = m_leftLeader.get()  * RobotController.getBatteryVoltage();
    double rightVolts = m_rightLeader.get() * RobotController.getBatteryVoltage();

    // Feed drivetrainsim
    m_driveSim.setInputs(leftVolts, rightVolts);
    m_driveSim.update(0.02);

    // Display on field
    m_fieldSim.setRobotPose(m_driveSim.getPose());
  }
}
