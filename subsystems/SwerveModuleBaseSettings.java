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

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib2960.controllers.PositionControllerSettings;
import frc.lib2960.controllers.RateControllerSettings;

/**
 * Defines module settings
 */
public class SwerveModuleBaseSettings {
            
    public String name;                 /**< Human friendly module name */
    public Translation2d translation;   /**< Module translation */

    public final double drive_ratio;    /**< Module drive gear ratio */
    public final double wheel_radius;   /**< Module drive wheel radius in meters */

    PositionControllerSettings angle_pos_ctrl;  /**< Module Angle Pos Controller */
    RateControllerSettings angle_rate_ctrl;     /**< Module Angle Rate Controller */
    RateControllerSettings drive_rate_ctrl;     /**< Module Drive Rate controller */

    /**
     * Constructor
     * @param   name            Module name
     * @param   translation     Module Translation
     * @param   drive_ratio     Drive gear ratio
     * @param   wheel_radius    Drive wheel radius
     * @param   angle_pos_ctrl  Module Angle Pos Controller Settings
     * @param   angle_rate_ctrl Module Angle Rate Controller Settings
     * @param   drive_rate_ctrl Module Drive Rate Controller Settings
     */
    public SwerveModuleBaseSettings(String name, Translation2d translation, 
                                    double drive_ratio, double wheel_radius,
                                    PositionControllerSettings angle_pos_ctrl, 
                                    RateControllerSettings angle_rate_ctrl, 
                                    RateControllerSettings drive_rate_ctrl) {

        this.name = name;
        this.translation = translation;
        this.drive_ratio = drive_ratio;
        this.wheel_radius = wheel_radius;
        this.angle_pos_ctrl = angle_pos_ctrl;
        this.angle_rate_ctrl = angle_rate_ctrl;
        this.drive_rate_ctrl = drive_rate_ctrl;
    }
}
