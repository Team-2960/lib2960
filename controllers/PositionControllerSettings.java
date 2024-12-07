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

import edu.wpi.first.wpilibj.TimedRobot;
import frc.lib2960.util.Limits;

/**
 * Position Controller Settings
 */
public class PositionControllerSettings {
    
    public final double max_accel;      /**< Maximum acceleration rate */
    public final double max_decel;      /**< Maximum deceleration rate */
    public final double max_rate;       /**< Maximum rate */

    public final boolean is_cont;       /**< Continuous position range  */
    public final Limits cont_range;     /**< Range of a continuous position controller */

    // TODO Move to global settings
    public final double period;         /**< Update period */
    
    /**
     * Constructor
     * @param   max_accel   Maximum acceleration rate
     * @param   max_decel   Maximum deceleration rate
     * @param   max_rate    Maximum rate
     * @param   is_cont     Continuous position roll over flag. Set to true to allow roll over 
     *                          at lower and upper limits set in cont_range. Limits ignored if
     *                          False.
     * @param   cont_range  Range of a continuous position controller
     * @param   period      Update period of the controller
     */
    public PositionControllerSettings(double max_accel, double max_decel, double max_rate, 
                                      boolean is_cont, Limits cont_range, double period) {
        this.max_accel = max_accel;
        this.max_decel = max_decel;
        this.max_rate = max_rate;
        this.is_cont = is_cont;
        this.cont_range = cont_range;
        this.period = period;
    }
    
    /**
     * Constructor. Period set to TimeRobot.kDefaultPeriod.
     * @param   max_accel   Maximum acceleration rate
     * @param   max_decel   Maximum deceleration rate
     * @param   max_rate    Maximum rate
     * @param   is_cont     Continuous position roll over flag. Set to true to allow roll over 
     *                          at Minimum amd Maximum positions. Limits ignored if False.
     * @param   cont_range  Range of a continuous position controller
     */
    public PositionControllerSettings(double max_accel, double max_decel, double max_rate, boolean is_cont, Limits cont_range) {
        this.max_accel = max_accel;
        this.max_decel = max_decel;
        this.max_rate = max_rate;
        this.is_cont = is_cont;
        this.cont_range = cont_range;
        this.period = TimedRobot.kDefaultPeriod;
    }
    
    /**
     * Constructor. 
     *      - cont_range is set to (0, 0)
     *      - is_cont is set to False
     *      - period set to TimeRobot.kDefaultPeriod.
     * @param   max_accel   Maximum acceleration rate
     * @param   max_decel   Maximum deceleration rate
     * @param   max_rate    Maximum rate
     */
    public PositionControllerSettings(double max_accel, double max_decel, double max_rate) {
        this.max_accel = max_accel;
        this.max_decel = max_decel;
        this.max_rate = max_rate;
        this.is_cont = false;
        this.cont_range = new Limits(0,0);
        this.period = TimedRobot.kDefaultPeriod;
    }
}
