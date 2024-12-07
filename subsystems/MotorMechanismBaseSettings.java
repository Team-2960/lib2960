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

import frc.lib2960.controllers.PositionControllerSettings;
import frc.lib2960.util.Limits;

public class MotorMechanismBaseSettings {
    public final String name;                               /**< Mechanism Name */
    public final String tab_name;                           /**< ShuffleBoard tab name */
    public final PositionControllerSettings pos_ctrl;       /**< Position Controller settings */
    public final MotorMechStageSettings[] stage_settings;   /**< List of Stage settings */
    public final Limits def_tol;                            /**< Default acceptable distance range 
                                                                around target to be considered 
                                                                "at target" */

    /**
     * Constructor
     * @param   name            Name of the mechanism
     * @param   tab_name        ShuffleBoard tab name
     * @param   pos_ctrl        Position Controller settings
     * @param   stage_settings  List of Rate Controller settings
     * @param   def_tol         Default acceptable distance range around target to be considered 
     *                              "at target"
     */
    public MotorMechanismBaseSettings(String name, String tab_name, PositionControllerSettings pos_ctrl, 
                                      MotorMechStageSettings[] stage_settings, Limits def_tol) {
        this.name = name;
        this.tab_name = tab_name;
        this.pos_ctrl = pos_ctrl;
        this.stage_settings = stage_settings;
        this.def_tol = def_tol;
    }

    // TODO add different default permutations of constructor
    
}
