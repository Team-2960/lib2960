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

public class PneumaticsRevPHSettings {
    public static final String DEF_NAME = "Pneumatics Hub";
    public static final int DEF_CAN_ID = 1;

    public enum ControlMode {DIGITAL, ANALOG, HYBRID};

    public final String name; 
    public final int can_id;
    public final ControlMode control_mode;
    public final double min_pressure;
    public final double max_pressure;

    /**
     * Constructor
     * @param   name            name of the module
     * @param   can_id          CAN ID for the module
     * @param   control_mode    Module Control Mode
     * @param   min_pressure    Minimum allowed pressure in PSI
     * @param   max_pressure    Maximum allowed pressure in PSI
     */
    public PneumaticsRevPHSettings(String name, int can_id, ControlMode control_mode, double min_pressure, double max_pressure) {
        this.name = name;
        this.can_id = can_id;
        this.control_mode = control_mode;
        this.min_pressure = min_pressure;
        this.max_pressure = max_pressure;
    }

    /**
     * Constructor
     *      - name set to DEF_NAME
     *      - can_id set to DEF_CAN_ID
     *      - control_mode set to DIGITAL
     *      - min_pressure set to 0
     *      - max_pressure set to 120 
     */
    public PneumaticsRevPHSettings() {
        this.name = DEF_NAME;
        this.can_id = DEF_CAN_ID;
        this.control_mode = ControlMode.DIGITAL;
        this.min_pressure = 0;
        this.max_pressure = 120;
    }
}
