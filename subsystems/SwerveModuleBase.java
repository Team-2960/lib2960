/**
 * Copyright 2024 Ryan Fitz-Gerald
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to 
 * deal in the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 */

package frc.lib2960.subsystems;

import frc.lib2960.controllers.*;

import edu.wpi.first.math.geometry.*;

import edu.wpi.first.wpilibj2.command.*;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * Abstract class containing most of the control necessary for a swerve drive module. It is 
 * designed to be used as a parent class for a swerve drive module class that implements access 
 * to the motors and sensors of the swerve module.
 */
public abstract class SwerveModuleBase extends SubsystemBase {
    /**
     * Automatic Swerve Drive Module Control Command
     */
    public class AutoCommand extends Command {        
        /**
         * Constructor
         */
        public AutoCommand() {

            // Add module as required subsystem 
            addRequirements(SwerveModuleBase.this);
        }

        /**
         * Update module position from desired module state
         */
        @Override
        public void execute() {
            updateAutoControl();
        }

    }

    public final SwerveModuleBaseSettings settings;     /**< Module Settings */

    private final PositionController angle_pos_ctrl;    /**< Module Angle Position Controller */
    private final RateController angle_rate_ctrl;       /**< Module Angle Rate Controller */
    private final RateController drive_rate_ctrl;       /**< Module Drive Controller */

    private SwerveModuleState desired_state;            /**< Most recent Module Desired State */

    // Shuffleboard Elements
    private GenericEntry sb_anglePosTarget;
    private GenericEntry sb_anglePosCurrent;
    private GenericEntry sb_angleVoltCurrent;
    private GenericEntry sb_angleRateCurrent;
    private GenericEntry sb_angleError;

    private GenericEntry sb_driveRateTarget;
    private GenericEntry sb_driveRateCurrent;
    private GenericEntry sb_driveVolt;
    
    /**
     * Constructor
     * @param   settings    module settings
     */
    public SwerveModuleBase(SwerveModuleBaseSettings settings) {
        // Initialize variables
        this.settings = settings;

        // Initialize controllers
        this.angle_pos_ctrl = new PositionController(settings.angle_pos_ctrl);
        this.angle_rate_ctrl = new RateController(settings.angle_rate_ctrl);
        this.drive_rate_ctrl = new RateController(settings.drive_rate_ctrl);

        // Initialize desired states
        desired_state = new SwerveModuleState();

        // Set default command
        setDefaultCommand(new AutoCommand());

        // Initialize Shuffleboard
        init_ui();
    }

    /**
     * Initialize Shuffleboard
     */
    private void init_ui() {
        var layout = Shuffleboard.getTab("Drive")
            .getLayout(settings.name + " Swerve", BuiltInLayouts.kList)
            .withSize(1, 4);

        sb_anglePosTarget = layout.add("Angle Target", 0).getEntry();
        sb_anglePosCurrent = layout.add("Angle Current", 0).getEntry();
        sb_angleVoltCurrent = layout.add("Angle Voltage", 0).getEntry();
        sb_angleRateCurrent = layout.add("Angle Rate", 0).getEntry();
        sb_angleError = layout.add("Angle Error", 0).getEntry();

        sb_driveRateTarget = layout.add("Drive Target", 0).getEntry();
        sb_driveRateCurrent = layout.add("Drive Current", 0).getEntry();
        sb_driveVolt = layout.add("Drive Voltage", 0).getEntry();
    }
    
    /**
     * Gets the current swerve module positions
     * 
     * @return current swerve module positions
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePos(), getAnglePos());
    }

    /**
     * Gets the current swerve module state
     * 
     * @return current swerve module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveRate(),
                getAnglePos());
    }

    /**
     * Sets the desired module state
     * 
     * @param desiredState desired state of the swerve module
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        desired_state.optimize(getAnglePos());
        this.desired_state = desiredState;
    }

    /**
     * Calculates the motor rotation to distance ratio
     * @return  motor rotation to distance ratio
     */
    public double motorToDistRatio() {
        return settings.drive_ratio * 2 * settings.wheel_radius * Math.PI;
    }

    /**
     * Periodic Method
     */
    @Override
    public void periodic() {
        updateUI();
    }

    /**
     * Updates module based on desired state
     */
    private void updateAutoControl() {
        updateAngle(desired_state.angle);
        updateDrive(desired_state.speedMetersPerSecond);
    }

    /**
     * Updates the angle position and rate controllers
     */
    private void updateAngle(Rotation2d target_angle) {
        double current_pos = getAnglePos().getDegrees();
        double current_rate = getAngleRate();
        double target_pos = target_angle.getDegrees();
        double target_rate = angle_pos_ctrl.update(current_pos, current_rate, target_pos);
        double angle_volt = angle_rate_ctrl.update(current_pos, current_rate, target_rate);
        
        setAngleVolt(angle_volt);
    }
    
    /**
     * Updates the drive rate controllers
     */
    private void updateDrive(double target_rate) {
        // Calculate the drive output from the drive PID controller.
        setDriveVolt(drive_rate_ctrl.update(0, getDriveRate(), target_rate));  
    }

    /**
     * Updates shuffleboard
     */
    private void updateUI() {
        sb_anglePosTarget.setDouble(desired_state.angle.getDegrees());
        sb_anglePosCurrent.setDouble(getAnglePos().getDegrees());
        sb_angleVoltCurrent.setDouble(getAngleVolt());
        sb_angleRateCurrent.setDouble(getAngleRate());
        sb_angleError.setDouble(desired_state.angle.getDegrees() - getAnglePos().getDegrees());

        sb_driveRateTarget.setDouble(desired_state.speedMetersPerSecond);
        sb_driveRateCurrent.setDouble(getDriveRate());
        sb_driveVolt.setDouble(getDriveVolt());
    }

    

    public abstract Rotation2d getAnglePos();
    public abstract double getAngleRate();
    public abstract double getAngleVolt();
    public abstract double getDrivePos();
    public abstract double getDriveRate();
    public abstract double getDriveVolt();
    
    public abstract void setAngleVolt(double volt);
    public abstract void setDriveVolt(double volt);
}
