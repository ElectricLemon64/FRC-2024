package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SpinState;
import frc.robot.Constants.Intake.IntakeState;

public class Intake extends SubsystemBase {
    private CANSparkBase angleMotorController, intakeMotorController, feederMotorController;
    private PIDController anglePIDController;
    // for angle debug
    private double targetPitch;

    public Intake() {
        angleMotorController = new CANSparkFlex(Constants.Intake.angleMotorID, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotorController = new CANSparkFlex(Constants.Intake.intakeMotorID, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotorController.setInverted(true);
        feederMotorController = new CANSparkMax(Constants.Intake.feederMotorID, CANSparkLowLevel.MotorType.kBrushless);
        anglePIDController = new PIDController(Constants.Intake.kP, Constants.Intake.kI, Constants.Intake.kD);
    }
    
    /** @return radians */
    public double getPitch() {
        return Units.rotationsToRadians(angleMotorController.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
    }
    
    /** @param goalPitch radians */
    private void setGoalPitch(double goalPitch) {
        goalPitch = MathUtil.clamp(goalPitch, IntakeState.STOW.pitch, IntakeState.DOWN.pitch);
        //angleMotorController.getPIDController().setReference(Units.radiansToRotations(goalPitch), CANSparkBase.ControlType.kSmartMotion);
        double ff = Constants.Intake.kF * Math.cos(this.getPitch());
        angleMotorController.set(anglePIDController.calculate(goalPitch, this.getPitch()) + ff);
        targetPitch = Units.radiansToDegrees(goalPitch);
    }

    /** @return radians / second */
    public double getSpin() {
        return Units.rotationsPerMinuteToRadiansPerSecond(intakeMotorController.getEncoder().getVelocity());
    }

    private void setSpin(SpinState ss) {
        intakeMotorController.set(ss.multiplier * Constants.Intake.intakeSpin);
        feederMotorController.set(ss.multiplier * Constants.Intake.feederSpin);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake Pitch", Units.radiansToDegrees(getPitch()));
        SmartDashboard.putNumber("Intake Goal", targetPitch);
    }

    public class ChangeState extends Command {
        private final Constants.Intake.IntakeState desiredState;

        public ChangeState(Constants.Intake.IntakeState ds) {
            desiredState = ds;
            addRequirements(Intake.this);
        }

        @Override
        public void initialize() {
            setGoalPitch(desiredState.pitch);
            setSpin(desiredState.spin);
            super.initialize();
        }

        @Override
        public boolean isFinished() {
            return Math.abs(getPitch() - desiredState.pitch) < Constants.Intake.pitchTolerance &&
                Math.abs(getSpin() - desiredState.spin.multiplier * Constants.Intake.intakeSpin) < Constants.Intake.spinTolerance;
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) setGoalPitch(Constants.Intake.IntakeState.STOW.pitch);
            setSpin(SpinState.ST);
            super.end(interrupted);
        }
    }
}