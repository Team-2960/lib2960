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

package frc.lib2960.controllers;

import frc.lib2960.util.*;

import edu.wpi.first.math.controller.*;

// TODO Implement using units library version of the Feed Forward methods

/**
 * Implements rate control for mechanisms
 */
public class RateController {

    /**
     * Generalization of the Feed Forward class to allow overloading
     */
    public abstract class FFControlBase{
        public final FFParam param; /**< Feedforward Parameters */
        
        /**
         * Constructor
         * @param   param   Feedforward Parameters
         */
        public FFControlBase(FFParam param){
            this.param = param;
        }

        /**
         * Updates the feedforward value
         * @param   current_pos     current mechanism position
         * @param   target_rate     target mechanism rate
         */
        public abstract double update(double current_pos, double target_rate);

        /**
         * Updates the feedforward value
         * @param   current_pos     current mechanism position
         * @param   target_rate     target mechanism rate
         * @param   target_accel    target mechanism acceleration
         */
        public abstract double update(double current_pos, double target_rate, double target_accel);
    }

    /**
     * Generalized version of the SimpleMotorFeedforward controller
     */
    public class FFControlSimple extends FFControlBase{
        private final SimpleMotorFeedforward controller;    /**< Feedforward controller */

        
        /**
         * Constructor
         * @param   param   Feedforward Parameters
         */
        public FFControlSimple(FFParam param){
            super(param);

            // Initialize controller
            controller = new SimpleMotorFeedforward(param.kS, param.kV, param.kA);
        }

        /**
         * Updates the feedforward value
         * @param   current_pos     current mechanism position
         * @param   target_rate     target mechanism rate
         */
        public double update(double current_pos, double target_rate) {
            return controller.calculate(target_rate);
        }

        /**
         * Updates the feedforward value
         * @param   current_pos     current mechanism position
         * @param   target_rate     target mechanism rate
         * @param   target_accel    target mechanism acceleration
         */
        public double update(double current_pos, double target_rate, double target_accel) {
            return controller.calculate(target_rate, target_accel);
        }
    }


    /**
     * Generalized version of the ArmFeedforward controller
     */
    public class FFControlArm extends FFControlBase{
        private final ArmFeedforward controller;    /**< Feedforward controller */
        
        /**
         * Constructor
         * @param   param   Feedforward Parameters
         */
        public FFControlArm(FFParam param){
            super(param);

            controller = new ArmFeedforward(param.kS, param.kG, param.kV, param.kA);
        }

        /**
         * Updates the feedforward value
         * @param   current_pos     current mechanism position
         * @param   target_rate     target mechanism rate
         */
        public double update(double current_pos, double target_rate) {
            return controller.calculate(current_pos, target_rate);
        }

        /**
         * Updates the feedforward value
         * @param   current_pos     current mechanism position
         * @param   target_rate     target mechanism rate
         * @param   target_accel    target mechanism acceleration
         */
        public double update(double current_pos, double target_rate, double target_accel) {
            return controller.calculate(current_pos, target_rate, target_accel);
        }
    }


    /**
     * Generalized version of the ElevatorFeedforward controller
     */
    public class FFControlElevator extends FFControlBase{
        private final ElevatorFeedforward controller;    /**< Feedforward controller */
        
        /**
         * Constructor
         * @param   param   Feedforward Parameters
         */
        public FFControlElevator(FFParam param){
            super(param);

            controller = new ElevatorFeedforward(param.kS, param.kG, param.kV, param.kA);
        }

        /**
         * Updates the feedforward value
         * @param   current_pos     current mechanism position
         * @param   target_rate     target mechanism rate
         */
        public double update(double current_pos, double target_rate) {
            return controller.calculate(target_rate);
        }

        /**
         * Updates the feedforward value
         * @param   current_pos     current mechanism position
         * @param   target_rate     target mechanism rate
         * @param   target_accel    target mechanism acceleration
         */
        public double update(double current_pos, double target_rate, double target_accel) {
            return controller.calculate(target_rate, target_accel);
        }
    }

    protected final RateControllerSettings settings;    /**< Controller settings */
    
    private final PIDController pid_control;            /**< PID Controller object */
    private final FFControlBase ff_control;             /**< Feed Forward controller object */
    
    /**
     * Constructor
     * @param   settings    Rate Controller Settings
     */
    public RateController(RateControllerSettings settings) {
        this.settings = settings;

        // Initialize PID Controller
        pid_control = new PIDController(settings.pid.kP, settings.pid.kI, settings.pid.kD);

        // Initialize Feed Forward Controller
        switch(settings.ff.type) {
            case ARM:
                ff_control = new FFControlArm(settings.ff);
                break;
            case ELEVATOR:
                ff_control = new FFControlElevator(settings.ff);
                break;
            default:
                ff_control = new FFControlSimple(settings.ff);
        }
    }

    /**
     * Update the controller output value
     */
    public double update(double current_pos, double current_rate, double target_rate) {
        return update_ff(current_pos, target_rate) + update_pid(current_rate, target_rate); 
    }

    /**
     * Update PID controller output
     * @param   current_rate    current rate of the mechanism
     * @param   target_rate     target rate of the mechanism
     * @return  output for the mechanism
     */
    private double update_pid(double current_rate, double target_rate) {
        return pid_control.calculate(current_rate, target_rate);
    }

    /**
     * Update PID controller output
     * @param   current_pos     current position of the mechanism
     * @param   target_rate     target rate of the mechanism
     * @return  output for the mechanism
     */
    private double update_ff(double current_pos, double target_rate) {
        return ff_control.update(current_pos, target_rate);
    }
}