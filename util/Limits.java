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

/**
 * Handles limit checks
 */
public class Limits {
    public final double lower;  /**< Lower Limit Value */
    public final double upper;  /**< Upper Limit Value */

    /**
     * Constructor
     * @param   lower       Lower Limit Value
     * @param   upper       Upper Limit Value
     */
    public Limits(double lower, double upper) {
        this.lower = lower;
        this.upper = upper;
    }
    
    /**
     * Constructor
     * @param   nominal         Nominal value
     * @param   lower_bound     Acceptable distance below nominal value
     * @param   upper_bound     Acceptable distance above nominal value
     */
    public Limits(double nominal, double lower_bound, double upper_bound) {
        this.lower = nominal - lower_bound;
        this.upper = nominal + upper_bound;
    }

    /**
     * Retrieves the range of the between the limits
     * @return  the range between the limits
     */
    public double getRange() {
        return upper - lower;
    }

    /**
     * Checks if value is above lower limit
     * @param   value
     * @return  true if value is above lower limit
     */
    public boolean inLower(double value) {
        return value >= lower;
    }

    /**
     * Checks if value is below upper limit
     * @param   value
     * @return  true if value is below upper limit
     */
    public boolean inUpper(double value) {
        return value <= upper;
    }

    /**
     * Checks if value is between the limits
     * @param   value
     * @return  true if value is between the limits
     */
    public boolean inRange(double value){
        return inLower(value) && inUpper(value);
    }
}