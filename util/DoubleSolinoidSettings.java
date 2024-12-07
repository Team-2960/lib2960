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

package frc.lib2960.util;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class DoubleSolinoidSettings {
    public final String name;
    public final int ph_can_id;
    public final PneumaticsModuleType module_type;
    public final int fwd_port;
    public final int rev_port;

    public DoubleSolinoidSettings(
        String name,
        int ph_can_id, 
        PneumaticsModuleType module_type, 
        int fwd_port, 
        int rev_port
    ) {
        this.name = name;
        this.ph_can_id = ph_can_id;
        this.module_type = module_type;
        this.fwd_port = fwd_port;
        this.rev_port = rev_port;
    }
}
