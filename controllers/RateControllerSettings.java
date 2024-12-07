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
import frc.lib2960.util.FFParam;
import frc.lib2960.util.PIDParam;

/**
 * Rate Controller Settings
 */
public class RateControllerSettings {
    public final FFParam ff;    /**< Feed Forward controller parameters */
    public final PIDParam pid;  /**< PID controller parameters */
    public final double period; /**< Controller update period */ 

    /**
     * Constructor
     *  - Period set to TimedRobot.kDefaultPeriod
     * @param ff        Feed Forward controller parameters
     * @param pid       PID controller parameters
     */
    public RateControllerSettings(FFParam ff, PIDParam pid) {
        this.ff = ff;
        this.pid = pid;
        this.period = TimedRobot.kDefaultPeriod;
    }
    
    /**
     * Constructord
     * @param ff        Feed Forward controller parameters
     * @param pid       PID controller parameters
     * @param period    Controller update period
     */
    public RateControllerSettings(FFParam ff, PIDParam pid, double period) {
        this.ff = ff;
        this.pid = pid;
        this.period = period;
    }
}
