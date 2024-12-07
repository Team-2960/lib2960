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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;

/**
 * Manages a REV Pneumatics Hub
 */
public class PneumaticsRevPH extends SubsystemBase {

    public final PneumaticsRevPHSettings settings;     /**< Module Settings */
    
    private final PneumaticHub ph;      /**< Module objet reference*/

    // ShuffleBoard
    private GenericEntry sb_pressure;   
    private GenericEntry sb_current;

    /**
     * Constructor. Settings set to default values;
     * @param   enabled     determines if the compressor is enabled when object is created
     */
    public PneumaticsRevPH(boolean enabled) {
        this.settings = new PneumaticsRevPHSettings();
        
        // Create PneumaticsHub
        ph = new PneumaticHub(settings.can_id);

        // Set Compressor Enabled
        enableCompressor(enabled);

        // Initialize Shuffleboard
        init_ui();
    }

    /**
     * Constructor
     * @param   settings    Pneumatics Hub settings
     * @param   enabled     determines if the compressor is enabled when object is created
     */
    public PneumaticsRevPH(PneumaticsRevPHSettings settings, boolean enabled) {
        this.settings = settings;

        // Create PneumaticsHub
        ph = new PneumaticHub(settings.can_id);

        // Set Compressor Enabled
        enableCompressor(enabled);

        // Initialize Shuffleboard
        init_ui();
    }

    /**
     * Initializes the class
     */
    private void init_ui() {
        // Setup ShuffleBoard
        var layout = Shuffleboard.getTab("Status")
                .getLayout(settings.name, BuiltInLayouts.kList)
                .withSize(1, 2);
        sb_pressure = layout.add("Pressure", 0).getEntry();
        sb_current = layout.add("Current", 0).getEntry();
    }

    /**
     * Enables/Disables the compressor
     * @param   enabled     true to enable the compressor, false to disable.
     */
    public void enableCompressor(boolean enabled) {
        if(enabled) {
            switch(settings.control_mode) {
                case DIGITAL:
                    ph.enableCompressorDigital();
                    break;
                case ANALOG:
                    ph.enableCompressorAnalog(settings.min_pressure, settings.max_pressure);
                    break;
                case HYBRID:
                    ph.enableCompressorHybrid(settings.min_pressure, settings.max_pressure);
                    break;
            }
        }else{
            ph.disableCompressor();
        }
    }

    /**
     * Periodic method. Updates UI.
     */
    @Override
    public void periodic() {
        updateUI();
    }

    /**
     * Updates ShuffleBoard
     */
    public void updateUI() {
        sb_pressure.setDouble(ph.getPressure(0));
        sb_current.setDouble(ph.getCompressorCurrent());
    }
}
