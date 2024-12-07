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

import frc.lib2960.util.*;
import frc.lib2960.controllers.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

/**
 * Base class for motorized mechanisms such as Arm joints, Elevators, Turrets, and Angle 
 * adjustments for Shooters.
 */
public abstract class MotorMechanismBase extends SubsystemBase {

    /**
     * Hold Position Command
     */
    public class HoldPositionCommand extends Command {
        private double target;                          /**< Target Position to hold */

        /**
         * Constructor
         */
        public HoldPositionCommand() {
            target = getPosition();

            addRequirements(MotorMechanismBase.this);
        }

        /**
         * Initialize command. Record current position
         */
        @Override
        public void initialize() {
            target = getPosition();
        }

        /**
         * Execute Command. Hold position when command was started
         */
        @Override
        public void execute() {
            getPosTrackingRate(target);
        }
    }

    /**
     * Set Voltage Command
     */
    public class SetVoltageCommand extends Command {
        private double target_voltage;                  /**< Target voltage for the mechanism */

        /**
         * Constructor
         * @param   target_voltage  Target voltage for the mechanism
         */
        public SetVoltageCommand(double target_voltage) {
            this.target_voltage = target_voltage;

            addRequirements(MotorMechanismBase.this);
        }

        /**
         * Execute Method. Update mechanism voltage
         */
        @Override
        public void execute() {
            updateVoltage(target_voltage);
        }

        /**
         * Sets the target voltage
         * @param   target_voltage     Target voltage for the mechanism
         */
        public void setVoltage(double target_voltage) {
            this.target_voltage = target_voltage;
        }
    }

    /**
     * Set Rate Command
     */
    public class SetRateCommand extends Command {
        private double target_rate;                     /**< Target rate for the mechanism */

        /**
         * Constructor
         * @param   target_rate     Target rate for the mechanism
         */
        public SetRateCommand(double target_rate) {
            this.target_rate = target_rate;

            addRequirements(MotorMechanismBase.this);
        }

        /**
         * Execute Method. Update mechanism rate
         */
        @Override
        public void execute() {
            updateRate(target_rate);
        }

        /**
         * IsFinished method. End if target rate is 0
         */
        @Override
        public boolean isFinished() {
            return target_rate == 0 || atLimit();
        }

        /**
         * Sets the target rate
         * @param   target_rate     Target rate for the mechanism
         */
        public void setRate(double target_rate) {
            this.target_rate = target_rate;
        }
    }

    /**
     * Set Position Command
     */
    public class SetPositionCommand extends Command {
        private double target;                          /**< Target position for the mechanism */
        private final Limits tol;                       /**< Acceptable distance from target 
                                                             position to consider mechanism "at target" */
        private final boolean auto_complete;            /**< Command automatically finishes when 
                                                             "at target" */
        /**
         * Constructor.
         *      - tol is set to def_tol of the mechanism
         *      - auto_complete is set to true
         * @param   target          Target position for the mechanism
         */
        public SetPositionCommand(double target) {
            this.target = target;
            this.tol = settings.def_tol;
            this.auto_complete = true;

            addRequirements(MotorMechanismBase.this);
        }

        /**
         * Constructor
         *      - auto_complete is set to true
         * @param   target          Target position for the mechanism
         * @param   tol             Acceptable distance from target position to consider mechanism
         *                              "at target"
         */
        public SetPositionCommand(double target, Limits tol) {
            this.target = target;
            this.tol = tol;
            this.auto_complete = true;

            addRequirements(MotorMechanismBase.this);
        }

        /**
         * Constructor
         *      - tol is set to def_tol of the mechanism
         * @param   target          Target position for the mechanism
         * @param   auto_complete   Command automatically finishes when "at target" if true. 
         *                              Continues indefinitely if false.  
         */
        public SetPositionCommand(double target, boolean auto_complete) {
            this.target = target;
            this.tol = settings.def_tol;
            this.auto_complete = auto_complete;

            addRequirements(MotorMechanismBase.this);
        }

        /**
         * Constructor
         * @param   target          Target position for the mechanism
         * @param   tol             Acceptable distance from target position to consider mechanism
         *                              "at target"
         * @param   auto_complete   Command automatically finishes when "at target" if true. 
         *                              Continues indefinitely if false.  
         */
        public SetPositionCommand(double target, Limits tol, boolean auto_complete) {
            this.target = target;
            this.tol = tol;
            this.auto_complete = auto_complete;

            addRequirements(MotorMechanismBase.this);
        }

        /**
         * Execute Method. Update mechanism rate
         */
        @Override
        public void execute() {
            double target_rate = getPosTrackingRate(target);
            setRate(target_rate);
        }

        /**
         * IsFinished method. End if target rate is 0
         */
        @Override
        public boolean isFinished() {
            return auto_complete && atTarget() && atLimit();
        }

        /**
         * Sets the target rate
         * @param   target          Target position for the mechanism
         */
        public void setTarget(double target) {
            this.target = target;
        }

        /**
         * Checks if the mechanism is at target
         * @return  true if mechanism is at the target position
         */
        public boolean atTarget() {
            double current_pos = getPosition();
            double error = current_pos - target;

            return tol.inRange(error);
        }
    }
    
    public final MotorMechanismBaseSettings settings;   /**< Mechanism settings */
    public final int motor_count;                       /**< Number of motors in the list */

    public final PositionController pos_ctrl;           /**< Position Controller */
    public final RateController[] rate_ctrls;           /**< Rate Controller */
    public int cur_stage;                               /**< Current stage index */

    public final HoldPositionCommand hold_pos_cmd;      /**< Internal hold position command */
    public final SetVoltageCommand set_voltage_cmd;     /**< Internal set voltage command */
    public final SetRateCommand set_rate_cmd;           /**< Internal set rate command */
    public final SetPositionCommand set_pos_cmd;        /**< Internal set position command */

    // ShuffleBoard
    protected final ShuffleboardLayout sb_layout;

    private GenericEntry sb_position;
    private GenericEntry sb_rate;
    
    private GenericEntry[] sb_voltages;
    private GenericEntry[] sb_currents;

    private GenericEntry sb_atLowerLimitSensor;
    private GenericEntry sb_atLowerSoftLimit;
    private GenericEntry sb_atUpperLimitSensor;
    private GenericEntry sb_atUpperSoftLimit;

    /**
     * Constructor
     * @param   settings    Mechanism Settings
     */
    public MotorMechanismBase(MotorMechanismBaseSettings settings, int motor_count) {
        this.settings = settings;
        this.motor_count = motor_count;

        // Initialize Controllers
        pos_ctrl = new PositionController(settings.pos_ctrl);

        int rate_ctrl_count = settings.stage_settings.length;
        rate_ctrls = new RateController[rate_ctrl_count];

        for(int i = 0; i < rate_ctrl_count; i++) {
            rate_ctrls[i] = new RateController(settings.stage_settings[i].rate_ctrl);
        }

        // Initialize Stage Index
        cur_stage = 0;

        // Initialize ShuffleBoard
        sb_layout = Shuffleboard.getTab(settings.tab_name)
            .getLayout(settings.name, BuiltInLayouts.kList)
            .withSize(1, 4);

        sb_position = sb_layout.add("Position", getPosition()).getEntry();
        sb_rate = sb_layout.add("Rate", getRate()).getEntry();

        sb_voltages = new GenericEntry[motor_count];
        sb_currents = new GenericEntry[motor_count];

        for(int i = 0; i < motor_count; i++) {
            sb_voltages[i] = sb_layout.add("Motor " + i + " Voltage", getMotorVoltage(i)).getEntry();
            sb_currents[i] = sb_layout.add("Motor " + i + " Current", getMotorCurrent(i)).getEntry();
        }

        sb_atLowerLimitSensor = sb_layout.add("Position", atLowerLimitSensor()).getEntry();
        sb_atLowerSoftLimit = sb_layout.add("Position", atLowerSoftLimit()).getEntry();
        sb_atUpperLimitSensor = sb_layout.add("Position", atUpperLimitSensor()).getEntry();
        sb_atUpperSoftLimit = sb_layout.add("Position", atUpperSoftLimit()).getEntry();

        // Initialize commands
        hold_pos_cmd = new HoldPositionCommand();
        set_voltage_cmd = new SetVoltageCommand(0);
        set_rate_cmd = new SetRateCommand(0);
        set_pos_cmd = new SetPositionCommand(getPosition(), false);

        // Set Default Command
        setDefaultCommand(hold_pos_cmd);
    }


    /**************************/
    /* Public Access Methods */
    /**************************/

    /**
     * Get the current stage index
     * @return current stage index
     */
    public int getStageIndex() {
        return cur_stage;
    }

    /**
     * Get the current soft limits
     * @return  current soft limits
     */
    public Limits getSoftLimits() {
        return settings.stage_settings[cur_stage].soft_limits;
    }

    /**
     * Checks if mechanism is at its lower soft limit
     * @return  true if at limit
     */
    public boolean atLowerSoftLimit() {
        
        return !pos_ctrl.settings.is_cont && getSoftLimits().inLower(getPosition());
    }

    /**
     * Checks if mechanism is at its upper soft limit
     * @return  true if at limit
     */
    public boolean atUpperSoftLimit() {
        return !pos_ctrl.settings.is_cont && getSoftLimits().inUpper(getPosition());
    }

    /**
     * Checks if mechanism is at its any soft limit
     * @return  true if at limit
     */
    public boolean atSoftLimit() {
        return getSoftLimits().inRange(getPosition());
    }

    /**
     * Checks if mechanism is at its any limit sensor
     * @return  true if at limit
     */
    public boolean atLimitSensor() {
        return atLowerLimitSensor() || atUpperLimitSensor();
    }

    /**
     * Checks if mechanism is at its any lower limit
     * @return  true if at limit
     */
    public boolean atLowerLimit() {
        return atLowerSoftLimit() || atLowerLimitSensor();
    }

    /**
     * Checks if mechanism is at its any upper limit
     * @return  true if at limit
     */
    public boolean atUpperLimit() {
        return atUpperSoftLimit() || atUpperLimitSensor();
    }

    /**
     * Checks if mechanism is at its any limit
     * @return  true if at limit
     */
    public boolean atLimit() {
        return atLowerLimitSensor();
    }

    /**
     * Checks if the mechanism is at the target position. Will always return false if in not 
     * running internal PositionTracking or HoldPosition commands. Always True if HoldPosition
     * is running. Checks atTarget in internal PositionTracking if it is running.
     * @return  True if at target, false otherwise.
     */
    public boolean atTarget() {
        boolean result = false;

        if(getCurrentCommand() == hold_pos_cmd) result = true;
        if (getCurrentCommand() == set_pos_cmd) result = set_pos_cmd.atTarget();

        return result;
    }
    

    /**************************/
    /* Public Control Methods */
    /**************************/



    /**
     * Selects the active mechanism stage. If selected stage does not exist, nothing 
     * changes.
     * @param   index   new stage index
     */
    public void setStageIndex(int index) {
        if(0 <= index && index < rate_ctrls.length) cur_stage = index;
    }

    /**
     * Sets the voltage of the mechanism
     * @param   voltage    target voltage
     */
    public void setVoltage(double voltage) {
        set_voltage_cmd.setVoltage(voltage);
        if(getCurrentCommand() != set_voltage_cmd) set_voltage_cmd.schedule();
    }

    /**
     * Sets the rate of the mechanism
     * @param   rate    target rate
     */
    public void setRate(double rate) {
        if(rate != 0) {
            set_rate_cmd.setRate(rate);
            if(getCurrentCommand() != set_rate_cmd) set_rate_cmd.schedule();
        } else {
            holdPosition();
        }
    }

    /**
     * Sets the target position of the mechanism
     * @param   position    target position
     */
    public void setPosition(double position) {
        //TODO:Allow tolerance to be updated
        set_pos_cmd.setTarget(position);
        if(getCurrentCommand() != set_pos_cmd) set_pos_cmd.schedule();
    }

    /**
     * Sets the mechanism to hold its current position
     */
    public void holdPosition() {
        Command current_cmd = getCurrentCommand();
        if(current_cmd != getDefaultCommand()) current_cmd.cancel();
    }

    /*****************************/
    /* Protected Control Methods */
    /*****************************/

    /**
     * Calculates the rate to reach the target position
     * @param   target_pos  target position
     * @return  rate to reach the desired position
     */
    protected double getPosTrackingRate(double target_pos) {
        double current_pos = getPosition();
        double current_rate = getRate();

        return pos_ctrl.update(current_pos, current_rate, target_pos);
    }

    /**
     * Updates the motor output to reach the target rate
     * @param   target_rate     target rate
     */
    protected void updateRate(double target_rate) {
        double current_pos = getPosition();
        double current_rate = getRate();

        // Set target rate to 0 if it would cause the mechanism to go out of range
        if(!pos_ctrl.settings.is_cont) {
            if(atLowerLimit() && target_rate < 0) target_rate = 0;
            if(atUpperLimit() && target_rate > 0) target_rate = 0;
        }
        
        updateVoltage(rate_ctrls[cur_stage].update(current_pos, current_rate, target_rate));
    }

    protected void updateVoltage(double voltage) {
        // Set voltage to zero if it would cause the mechanism to go out of range
        if(!pos_ctrl.settings.is_cont) {
            if(atLowerLimit() && voltage < 0) voltage = 0;
            if(atUpperLimit() && voltage > 0) voltage = 0;
        }

        setMotorVoltage(voltage);
    }

    /****************************/
    /* Protected Access Methods */
    /****************************/

    /**
     * Get shuffleboard layout for the mechanism
     * @return  Shuffleboard layout for the mechanism
     */
    protected ShuffleboardLayout getSBLayout() {
        return sb_layout;
    }

    /*********************/
    /* Subsystem Methods */
    /*********************/

    public void periodic() {
        updateUI();
    }

    /***************************/
    /* Periodic Helper Methods */
    /***************************/
    public void updateUI() {
        sb_position.setDouble(getPosition());
        sb_rate.setDouble(getRate());

        for(int i = 0; i < motor_count; i++) {
            sb_voltages[i].setDouble(getMotorVoltage(i));
            sb_currents[i].setDouble(getMotorCurrent(i));
        }
        
        sb_atLowerLimitSensor.setBoolean(atLowerLimitSensor());
        sb_atLowerSoftLimit.setBoolean(atLowerSoftLimit());
        sb_atUpperLimitSensor.setBoolean(atUpperLimitSensor());
        sb_atUpperSoftLimit.setBoolean(atUpperSoftLimit());
    }

    /********************/
    /* Abstract Methods */
    /********************/
    public abstract double getPosition();
    public abstract double getRate();
    public abstract double getMotorVoltage(int i);
    public abstract double getMotorCurrent(int i);

    public boolean atLowerLimitSensor() { return false; }
    public boolean atUpperLimitSensor() { return false; }

    public abstract void setMotorVoltage(double voltage);
}