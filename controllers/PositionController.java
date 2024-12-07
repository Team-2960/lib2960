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

// TODO Implement using units library version of the Feed Forward methods

/**
 * Position Controller Class
 */
public class PositionController {
    public final PositionControllerSettings settings;    /**< Position Controller Settings */

    /**
     * Constructor
     * @param   settings    Position Controller Settings
     */
    public PositionController(PositionControllerSettings settings) {
        this.settings = settings;
    }

    /**
     * Updates the position controller
     * @param   current_pos     current mechanism position
     * @param   current_rate    current mechanism rate
     * @param   target_pos      target mechanism position
     * @return  target rate for the position controller
     */
    public double update(double current_pos, double current_rate, double target_pos) {
        // Calculate error and direction to target
        double error = target_pos - current_pos;

        if(settings.is_cont) {
            double range = this.settings.cont_range.getRange();
            double error_low = target_pos - range - current_pos;
            double error_high = target_pos + range - current_pos;
            
            if(Math.abs(error) > Math.abs(error_low)) error = error_low;
            if(Math.abs(error) > Math.abs(error_high)) error = error_high;
        }

        double dir = error > 0 ? 1 : -1;

        // Calculate target Rate
        double rate = dir * settings.max_rate;
        double accel_rate = dir * settings.max_accel * settings.period + current_rate;
        
        double decel_dist = Math.pow(settings.max_rate, 2) / (2 * settings.max_decel);
        double decel_rate = error / decel_dist  * settings.max_decel;
        
        if(Math.abs(rate - accel_rate) > settings.max_rate) rate = accel_rate;
        if(Math.abs(rate) > Math.abs(accel_rate)) rate = accel_rate;
        if(Math.abs(rate) > Math.abs(decel_rate)) rate = decel_rate;

        return rate;
    }
}