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

/**
 * Robot drivetrain settings
 */
public class SwerveDriveBaseSettings {
    public final double max_drive_speed;    /**< Maximum speed of the robot drivetrain */
    public final double max_angle_rate;     /**< Maximum angle rate of the robot */
    
    public final double tracking_angle_accel;   /**< Angle tracking angle acceleration */
    public final double tracking_angle_decel;   /**< Angle tracking angle deceleration */

    /**
     * Constructor
     * @param   max_drive_speed         maximum drive speed of the robot drivetrain
     * @param   max_angle_rate          maximum angle rate of the robot drivetrain
     * @param   tracking_angle_accel    angle tracking angle acceleration of the robot 
     *                                      drivetrain
     * @param   tracking_angle_decel    angle tracking angle deceleration of the robot 
     *                                      drivetrain
     * #
     */
    public SwerveDriveBaseSettings(double max_drive_speed, double max_angle_rate, 
                    double tracking_angle_accel, double tracking_angle_decel) {
        this.max_drive_speed = max_drive_speed;
        this.max_angle_rate = max_angle_rate;
        this.tracking_angle_accel = tracking_angle_accel;
        this.tracking_angle_decel = tracking_angle_decel;
    }
}
