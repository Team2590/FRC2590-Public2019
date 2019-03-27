/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.controllers.ConstantCurrent;
import frc.controllers.MotionProfile;
import frc.robot.RobotMap;
import frc.settings.ClimberSettings;
import frc.settings.FieldSettings;
import frc.util.NemesisPotentiometer;
import frc.util.NemesisVictor;

/**
 * Add your docs here.
 */
public class Climber implements ClimberSettings, FieldSettings, RobotMap {

    private static Climber climberInstance = null;

    public static Climber getClimberInstance() {
        if (climberInstance == null) {
            climberInstance = new Climber();
        }
        return climberInstance;
    }

    private States climberState = States.STOPPED;

    private enum States {
        STOPPED, MOVING
    }

    private NemesisVictor climberArticulateMaster;
    private NemesisVictor climberArticulateSlave;
    private NemesisVictor intakeMotor;

    private NemesisPotentiometer climberPot;

    private MotionProfile climberController;
    private ConstantCurrent manualController;

    private double setpoint;
    private double errorSum;
    private double lastError;

    private boolean manual;

    public Climber() {

        climberArticulateMaster = new NemesisVictor(CLIMBER_ARTICULATE_MASTER);
        climberArticulateSlave = new NemesisVictor(CLIMBER_ARTICULATE_SLAVE);
        intakeMotor = new NemesisVictor(CLIMBER_INTAKE);

        climberArticulateSlave.follow(climberArticulateMaster);

        climberPot = new NemesisPotentiometer(CLIMBER_POTENTIOMETER, 360.0);

        climberController = new MotionProfile(CLIMBER_KP, CLIMBER_KI, CLIMBER_KV, CLIMBER_KA, CLIMBER_MAX_VEL,
                CLIMBER_MAX_ACC, CLIMBER_TOLERANCE, climberPot, climberArticulateMaster);

        manualController = new ConstantCurrent(climberArticulateMaster);

        setpoint = getAngle();
        errorSum = 0.0;
        lastError = 0.0;

        manual = false;
    }

    public void update() {
        switch (climberState) {
        case STOPPED:
            double error = setpoint - getAngle();
            double command = 0.0;
            errorSum += error * REFRESH_RATE;
            double deltaError = error - lastError;
            // adds the error terms
            command = error * kP_HOLD_CONSTANT + errorSum * kI_HOLD_CONSTANT + deltaError * kD_HOLD_CONSTANT;

            lastError = error;

            climberArticulateMaster.set(ControlMode.PercentOutput, command);
            break;

        case MOVING:
            if (manual) {
                this.setpoint = getAngle();
                manualController.calculate();
                if (manualController.isDone()) {
                    climberState = States.STOPPED;
                }
            } else {
                climberController.calculate();
                if (climberController.isDone()) {
                    climberState = States.STOPPED;
                }
            }
            break;

        default:
            break;
        }
    }

    public void setSetpoint(double setpoint) {
        climberController.setSetpoint(setpoint);
        this.setpoint = setpoint;
        this.errorSum = 0.0;
        this.lastError = 0.0;
        this.manual = false;
        climberState = States.MOVING;
    }

    public void moveManually(double speed) {
        manualController.setSetpoint(speed);
        this.errorSum = 0.0;
        this.lastError = 0.0;
        this.manual = true;
        climberState = States.MOVING;
    }

    public void intakeClimber() {
        intakeMotor.set(ControlMode.PercentOutput, 1.0);
    }

    public void outakeClimber() {
        intakeMotor.set(ControlMode.PercentOutput, -1.0);
    }

    public void stopIntake() {
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * @return current angle of the climber
     */
    public double getAngle() {
        return climberPot.pidGet();
    }
}
