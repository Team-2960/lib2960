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

package frc.lib2960.photonvision;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;


/**
 * Apriltag Pipeline Settings
 */
public class AprilTagPipelineSettings {
    public final String name;                       /**< Name of the pipeline */
    public final String camera_name;                /**< Camera name for the pipeline configured in PhotonVision */
    public final AprilTagFields field_layout;       /**< AprilTag Field Layout object */
    public final Transform3d robot_to_camera;       /**< Robot to Camera transformation */
    public final PoseStrategy pose_strategy;        /**< Pose Estimation Strategy */
    public final double max_dist;                   /**< Maximum acceptable target distance from camera in meters */
    public final Vector<N3> single_tag_std;         /**< Single tag standard deviation vector */
    public final Vector<N3> multi_tag_std;          /**< Multi tag standard deviation vector */

    // TODO Allow the standard deviation to increase based on distance

    /**
     * Constructor
     * @param   name                Name of the pipeline
     * @param   camera_name         Camera name for the pipeline as configured in PhotoVision
     * @param   field_layout        AprilTag Field Layout object
     * @param   robot_to_camera     Robot to Camera transformation
     * @param   pose_strategy       Pose Estimation Strategy
     * @param   max_dist            Maximum acceptable target distance from camera in meters
     * @param   single_tag_std      Single tag standard deviation vector
     * @param   multi_tag_std       Multi tag standard deviation vector
     */
    public AprilTagPipelineSettings(
        String name, 
        String camera_name, 
        AprilTagFields field_layout, 
        Transform3d robot_to_camera, 
        PoseStrategy pose_strategy,
        double max_dist,
        Vector<N3> single_tag_std,
        Vector<N3> multi_tag_std
    ) {
        this.name = name;
        this.camera_name = camera_name;
        this.field_layout = field_layout;
        this.robot_to_camera = robot_to_camera;
        this.pose_strategy = pose_strategy;
        this.max_dist = max_dist;
        this.single_tag_std = single_tag_std;
        this.multi_tag_std = multi_tag_std;
    }

    /**
     * Constructor. 
     *      - name is set to camera_name
     *      - field_layout is set to AprilTagFields.kDefault (current season)
     *      - pose_strategy is set to PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
     * @param   camera_name         Camera name for the pipeline as configured in PhotoVision
     * @param   robot_to_camera     Robot to Camera transformation
     * @param   max_dist            Maximum acceptable target distance from camera in meters
     * @param   single_tag_std      Single tag standard deviation vector
     * @param   multi_tag_std       Multi tag standard deviation vector
     */
    public AprilTagPipelineSettings(
        String camera_name, 
        Transform3d robot_to_camera, 
        double max_dist,
        Vector<N3> single_tag_std,
        Vector<N3> multi_tag_std
    ) {            
        this.name = camera_name;
        this.camera_name = camera_name;
        this.field_layout = AprilTagFields.kDefaultField;       // TODO: Find non-depricated method to get April Tag Field Locations
        this.robot_to_camera = robot_to_camera;
        this.pose_strategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        this.max_dist = max_dist;
        this.single_tag_std = single_tag_std;
        this.multi_tag_std = multi_tag_std;
    }

    /**
     * Constructor. 
     *      - name is set to camera_name
     *      - field_layout is set to AprilTagFields.kDefault (current season)
     *      - pose_strategy is set to PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
     *      - max_dist is set to 4 meters
     *      - single_tag_std is set to VecBuilder.fill(4, 4, 8)
     *      - multi_tag_std is set to VecBuilder.fill(0.5, 0.5, 1)
     * @param   camera_name         Camera name for the pipeline as configured in PhotoVision
     * @param   robot_to_camera     Robot to Camera transformation
     */
    public AprilTagPipelineSettings(String camera_name, Transform3d robot_to_camera) {
        this.name = camera_name;
        this.camera_name = camera_name;
        this.field_layout = AprilTagFields.kDefaultField;       // TODO: Find non-depricated method to get April Tag Field Locations
        this.robot_to_camera = robot_to_camera;
        this.pose_strategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        this.max_dist = 4;
        this.single_tag_std = VecBuilder.fill(4, 4, 8);
        this.multi_tag_std = VecBuilder.fill(0.5, 0.5, 1);
    }
}
