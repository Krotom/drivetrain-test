package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

public class Robot extends TimedRobot {
  private int drive_mode;
  private final SendableChooser<Integer> driveChooser = new SendableChooser<>();

  private final XboxController m_driverController = new XboxController(0);

  private final SparkMax m_leftLeader  = new SparkMax(2, SparkMax.MotorType.kBrushed);
  private final SparkMax m_leftFollower = new SparkMax(3, SparkMax.MotorType.kBrushed);

  private final SparkMax m_rightLeader = new SparkMax(0, SparkMax.MotorType.kBrushed);
  private final SparkMax m_rightFollower = new SparkMax(1, SparkMax.MotorType.kBrushed);

  private DifferentialDrive m_drive;

  private DifferentialDrivetrainSim m_driveSim;
  private Field2d m_fieldSim;

  public Robot() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

    leftFollowerConfig.follow(m_leftLeader);
    rightFollowerConfig.follow(m_rightLeader);

    m_leftLeader.configure(leftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    m_rightLeader.configure(rightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    m_leftFollower.configure(leftFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    m_rightFollower.configure(rightFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);

    m_driveSim = new DifferentialDrivetrainSim(
        DCMotor.getCIM(2),
        10.71,
        7.5,
        50.0,
        Units.inchesToMeters(3),
        Units.inchesToMeters(28),
        VecBuilder.fill(0.005, 0.005, 0.001, 0.05, 0.05, 0.005, 0.005)
    );

    m_fieldSim = new Field2d();

    driveChooser.addOption("Tank Drive", 0);
    driveChooser.setDefaultOption("Arcade Drive", 1);
    driveChooser.addOption("Curvature Drive", 2);

    Shuffleboard.getTab("Default")
        .add("Drive Mode", driveChooser)
        .withWidget(BuiltInWidgets.kSplitButtonChooser);

    Shuffleboard.getTab("Default")
        .add("Field View", m_fieldSim);
  }

  @Override
  public void robotPeriodic() {
    Integer choice = driveChooser.getSelected();
    if (choice != null) {
      drive_mode = choice;
    }
  }

  @Override
  public void autonomousPeriodic() {
    m_drive.arcadeDrive(0.5, 0.0);
  }

  @Override
  public void teleopPeriodic() {
    double left = -m_driverController.getLeftY();
    double right = -m_driverController.getRightY();
    boolean isQuickTurn = m_driverController.getRightBumperButton();
    if (drive_mode == 0) {
      m_drive.tankDrive(left, right);
    } else if (drive_mode == 1) {
      m_drive.arcadeDrive(left, right);
    } else if (drive_mode == 2) {
      m_drive.curvatureDrive(left, right, isQuickTurn);
    }
  }

  @Override
  public void simulationPeriodic() {
    double leftVolts  = m_leftLeader.get()  * RobotController.getBatteryVoltage();
    double rightVolts = m_rightLeader.get() * RobotController.getBatteryVoltage();

    m_driveSim.setInputs(leftVolts, rightVolts);
    m_driveSim.update(0.02);

    m_fieldSim.setRobotPose(m_driveSim.getPose());
  }
}
