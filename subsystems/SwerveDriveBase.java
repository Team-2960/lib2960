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

import frc.lib2960.util.*;
import frc.lib2960.controllers.*;
import frc.lib2960.pathplanner.PathPlanner;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public abstract class SwerveDriveBase extends Drivetrain {
    /**
     * Command to run swerve drive by controlling the angle rate of the robot
     */
    public class AngleRateCommand extends Command {

        /**
         * Constructor
         */
        public AngleRateCommand() {
            // Add drivetrain as required subsystem
            addRequirements(SwerveDriveBase.this);
        }   

        /**
         * Update the drivetrain kinematics
         */
        @Override
        public void execute() {
            updateKinematics(desired_speeds);
        }

    }

    /**
     * Command to track a desired angle
     */
    public class AngleTrackCommand extends Command {
        private Rotation2d target;
        
        /**
         * Constructor
         * @param   target              Target angle to track
         */
        public AngleTrackCommand(Rotation2d target) {
            this.target = target;
            
            addRequirements(SwerveDriveBase.this);
        } 


        /**
         * Updates angle tracking
         */
        @Override
        public void execute() {
            ChassisSpeeds speeds = new ChassisSpeeds(
                desired_speeds.vxMetersPerSecond, 
                desired_speeds.vyMetersPerSecond,
                getAngleTrackingRate(target)
            );

            updateKinematics(speeds);
        }

        /**
         * Updates the target angle
         * @param   target  New target angle
         */
        public void updateTarget(Rotation2d target) {
            this.target = target;
        }
    }

    /**
     * Command to track a desired position
     */
    public class PointTrackCommand extends Command {
        private Translation2d target;
        private Rotation2d offset;
        
        /**
         * Constructor. Offset is set to 0 degrees.
         * @param   target              Target point to track
         */
        public PointTrackCommand(Translation2d target) {
            this.target = target;
            this.offset = new Rotation2d();
            
            addRequirements(SwerveDriveBase.this);
        } 
        
        /**
         * Constructor
         * @param   target              Target point to track
         * @param   offset              Robot angle offset from target point
         * @param   is_field_relative   sets if the linear motions are field relative
         */
        public PointTrackCommand(Translation2d target, Rotation2d offset) {
            this.target = target;
            this.offset = offset;

            addRequirements(SwerveDriveBase.this);
        } 


        /**
         * Updates point tracking
         */
        @Override
        public void execute() {
            // Calculate target angle
            Translation2d current = getEstimatedPos().getTranslation();
            Translation2d difference = target.minus(current);
            Rotation2d target_angle = difference.getAngle().plus(offset);

            // Update Kinematics
            ChassisSpeeds speeds = new ChassisSpeeds(
                desired_speeds.vxMetersPerSecond, 
                desired_speeds.vyMetersPerSecond,
                getAngleTrackingRate(target_angle)
            );

            updateKinematics(speeds);
        }

        /**
         * Updates the target point
         * @param   target  New target point
         */
        public void updateTarget(Translation2d target) {
            this.target = target;
        }

        /**
         * Updates the target offset angle
         * @param   offset  New target offset angle
         */
        public void updateOffset(Rotation2d offset) {
            this.offset = offset;
        }
        
    }

    /**
     * Command to set the modules to point toward center of the robot to prevent the robot from 
     * rolling freely.
     */
    public class CrossWheelsCommand extends Command {

        /**
         * Constructor
         * @param   dt      Drivetrain object reference
         */
        public CrossWheelsCommand() {
            addRequirements(SwerveDriveBase.this);
        }

        /**
         * Initialize method. Set drivetrain to crossed wheels
         */
        @Override
        public void initialize() {
            updateCrossWheels();
        }

    }

    // TODO implement TargetTrackCommand

    private final SwerveDriveBaseSettings settings;     /**< Drivetrain settings */

    private final SwerveModuleBase[] modules;           /**< List of drivetrain swerve modules */

    private ChassisSpeeds desired_speeds;               /**< Desired chassis speeds */

    private boolean is_field_relative = false;          /**< Field Relative enable flag */

    private boolean vision_updated = false;             /**< Vision update at least once flag */
    private boolean ignore_camera = false;              /**< Ignore vision updates flag */

    private final SwerveDriveKinematics kinematics;     /**< Swerve drive Kinematics object */
    private final SwerveDrivePoseEstimator pose_est;    /**< Swerve Drive Pose Estimator object */

    private final PositionController angle_tracker;     /**< Position tracker for angle tracking */

    private final AngleRateCommand angle_rate_cmd;      /**< Internal Angle Rate command */
    private final AngleTrackCommand angle_track_cmd;    /**< Internal Angle Tracking command */
    private final PointTrackCommand point_track_cmd;    /**< Internal Point Tracking command */
    private final CrossWheelsCommand cross_wheels_cmd;  /**< Internal Cross Wheels command */

    // Shuffleboard
    private GenericEntry sb_posEstX;
    private GenericEntry sb_posEstY;
    private GenericEntry sb_posEstR;

    private GenericEntry sb_speedX;
    private GenericEntry sb_speedY;
    private GenericEntry sb_speedR;
    private GenericEntry sb_robotTargetAngle;
    private GenericEntry sb_speedTargetR;
    private double robotTargetAngle;

    private ComplexWidget sb_field2d;
    private Field2d field2d;
    private FieldObject2d fieldTargetPoint;

    /**
     * Constructor
     * @param   modules         list of module used in the drivetrain
     * @param   module_details  module details object for the drivetrain
     */
    public SwerveDriveBase(SwerveDriveBaseSettings settings, SwerveModuleBase[] modules) {
        this.settings = settings;
        this.modules = modules;
        
        // Initialize Kinematics
        kinematics = new SwerveDriveKinematics(getModuleTranslation());

        // Initialize Pose Estimation
        pose_est = new SwerveDrivePoseEstimator(
            kinematics, 
            getAngle(), 
            getModulePositions(), 
            new Pose2d(), 
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );
        
        // Initialize angle tracking
        PositionControllerSettings angle_tracker_settings = new PositionControllerSettings(
            settings.tracking_angle_accel,
            settings.tracking_angle_decel,
            settings.max_angle_rate,
            true,
            new Limits(0, 360)
        );

        angle_tracker = new PositionController(angle_tracker_settings);

        // Initialize internal commands
        angle_rate_cmd = new AngleRateCommand();
        angle_track_cmd = new AngleTrackCommand(new Rotation2d());
        point_track_cmd = new PointTrackCommand(new Translation2d());
        cross_wheels_cmd = new CrossWheelsCommand();

        // Set default command
        setDefaultCommand(angle_rate_cmd);

        // Register Named Command with PathPlanner
        PathPlanner.registerCommand("Cross Wheels", cross_wheels_cmd);

        // Initialize Shuffleboard
        init_ui();
    }

    /**
     * Initializes Shuffleboard
     */
    private void init_ui() {
        // Initialize Shuffleboard
        var pose_layout = Shuffleboard.getTab("Drive")
                .getLayout("Drive Pose", BuiltInLayouts.kList)
                .withSize(1, 4);
        sb_posEstX = pose_layout.add("Pose X", pose_est.getEstimatedPosition().getX()).getEntry();
        sb_posEstY = pose_layout.add("Pose Y", pose_est.getEstimatedPosition().getY()).getEntry();
        sb_posEstR = pose_layout
                .add("Pose R", pose_est.getEstimatedPosition().getRotation().getDegrees()).getEntry();

        sb_speedX = pose_layout.add("Speed X", 0).getEntry();
        sb_speedY = pose_layout.add("Speed Y", 0).getEntry();
        sb_speedR = pose_layout.add("Speed R", 0).getEntry();
        sb_robotTargetAngle = pose_layout.add("Robot Target Angle", 0).getEntry();

        sb_speedTargetR = pose_layout.add("Target Speed R", 0).getEntry();

        sb_field2d = Shuffleboard.getTab("Drive").add(field2d).withWidget("Field");
    }


    /*************************/
    /* Module Access Methods */
    /*************************/

    /**
     * Generate a list of swerve drive module position translations
     * @return  list of module position translations
     */
    public Translation2d[] getModuleTranslation() {
        Translation2d result[] = new Translation2d[modules.length]; 
        
        for(int i = 0; i < modules.length; i++) result[i] = modules[i].settings.translation;
        
        return result;
    }

    /**
     * Generate list of swerve drive module states
     * @return  list of module states
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState result[] = new SwerveModuleState[modules.length];

        for(int i = 0; i < modules.length; i++) result[i] = modules[i].getState();

        return result;
    }

    /**
     * Generate list of swerve drive module positions
     * @return  list of module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition result[] = new SwerveModulePosition[modules.length];

        for(int i = 0; i < modules.length; i++) result[i] = modules[i].getPosition();

        return result;
    }


    /******************************/
    /* Drivetrain Control Methods */
    /******************************/

    /**
     * Sets field relative mode
     * @param   enable  set to true to enable field relative mode. Set to false to disable.
     */
    public void setFieldRelative(boolean enable) {
        this.is_field_relative = enable;
    }

    /**
     * Sets the drivetrain linear speeds in x and y parameters. Direction of
     * the axis used determine by field relative setting.
     * 
     * @param xSpeed speed of along the x-axis
     * @param ySpeed speed of along the y-axis
     */
    public void setSpeed(double xSpeed, double ySpeed) {
        desired_speeds.vxMetersPerSecond = xSpeed;
        desired_speeds.vyMetersPerSecond = ySpeed;
    }

    /**
     * Sets the drivetrain linear speeds by setting speed and a heading. Direction
     * of the axis used determine by field relative setting.
     * 
     * @param speed   linear speed of the robot
     * @param heading heading of the robot
     */
    public void setVector(double speed, Rotation2d heading) {
        double xSpeed = Math.cos(heading.getRadians()) * speed;
        double ySpeed = Math.sin(heading.getRadians()) * speed;

        setSpeed(xSpeed, ySpeed);
    }

    /**
     * Sets the chassis speeds relative to the field
     * @param   speeds chassis speeds relative to the field
     */
    public void setFieldRelativeSpeeds(ChassisSpeeds speeds) {
        this.setFieldRelative(true);
        this.desired_speeds = speeds;
        runAngleRateCmd();
    }

    /**
     * Sets the chassis speeds relative to the robot
     * @param   speeds chassis speeds relative to the robot
     */
    public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
        this.setFieldRelative(false);
        this.desired_speeds = speeds;
        runAngleRateCmd();
    }

    /**
     * Sets the angular rate of the robot
     * @param rSpeed angle rate for the robot
     */
    public void setAngleRate(double rSpeed) {
        desired_speeds.omegaRadiansPerSecond = rSpeed;
        runAngleRateCmd();
    }

    /**
     * Sets the robot to track a field relative angle
     * @param   target  target angle
     */
    public void setAngleTracking(Rotation2d target) {
        angle_track_cmd.updateTarget(target);
        runAngleTrackCmd();
    }

    /**
     * Sets the robot to track a point relative to the field
     * @param   target  target point
     * @param   offset  angle offset from the target point
     */
    public void setPointTracking(Translation2d target, Rotation2d offset) {
        point_track_cmd.updateTarget(target);
        point_track_cmd.updateOffset(offset);
        runPointTrackCmd();
    }

    /**
     * Sets the robot to track a point relative to the field. Angle offset is set to 0
     * @param   target  target point
     */
    public void setPointTracking(Translation2d target) {
        setPointTracking(target, new Rotation2d());
    }

    /**
     * Sets the robot to have crossed wheels
     */
    public void setCrossedWheels() {
        runCrossWheelsCmd();
    }

    /**
     * Cancels any of the non-default commands
     */
    public void disableAutoTracking() {
        runAngleRateCmd();
    }

    /*****************************/
    /* Drivetrain Access Methods */
    /*****************************/

    /**
     * Gets the robots estimated pose
     * 
     * @return estimated robot pose
     */
    public Pose2d getEstimatedPos() {
        return pose_est.getEstimatedPosition();
    }

    /**
     * Gets the current chassis speeds relative to the robot
     * @return current robot chassis speeds relative to the robot
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Gets the current chassis speeds relative to the field
     * @return current robot chassis speeds relative to the field
     */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        ChassisSpeeds speeds = getRobotRelativeSpeeds();
        speeds.toFieldRelativeSpeeds(getAngle());
        return speeds;
    }

    /**
     * Resets the estimated robot position
     * @param   new_pose    new estimated robot position
     */
    public void resetPoseEst(Pose2d new_pose) {
        pose_est.resetPosition(getAngle(), getModulePositions(), new_pose);
        vision_updated = false;
    }

        
    /*************************/
    /* Vision Access Methods */
    /*************************/

    /**
     * adds a new vision pose update
     * @param pose          Estimated pose from the vision
     * @param time_stamp    Timestamp of when the pose was captured
     */
    public void addVisionPose(Pose2d pose, double time_stamp) {
        if (!ignore_camera) {
            pose_est.addVisionMeasurement(pose, time_stamp);
            vision_updated = true;
        }
    }

    /**
     * adds a new vision pose update
     * @param pose          Estimated pose from the vision
     * @param time_stamp    Timestamp of when the pose was captured
     * @param std_dev       Standard deviation values to use for the pose estimation
     */
    public void addVisionPose(Pose2d pose, double time_stamp, Vector<N3> std_dev) {
        if (!ignore_camera) {
            pose_est.addVisionMeasurement(pose, time_stamp, std_dev);
            vision_updated = true;
        }
    }

    /*********************************/
    /* Angle Tracking Helper Methods */
    /*********************************/

    /**
     * Calculates the angle rate for tracking an angle
     * @param   target  Target angle
     * @return  Angle rate to track the target angle
     */
    public double getAngleTrackingRate(Rotation2d target) {
        double cur_pos = getAngle().getDegrees();
        double cur_rate = getAngleRate();

        return angle_tracker.update(cur_pos, cur_rate, target.getDegrees());
    }


    /*********************/
    /* Subsystem Methods */
    /*********************/

    /**
     * Subsystem periodic update method
     */
    @Override
    public void periodic() {
        updatePoseEst();
        updateUI();
    }
            
    /**************************/
    /* Private Update Methods */
    /**************************/

    /**
     * Updates pose estimation. Called by periodic method.
     */
    private void updatePoseEst() {
        pose_est.update(getAngle(), getModulePositions());
    }

    /**
     * Update User Interface. Called by periodic method.
     */
    private void updateUI() {
        Pose2d pose = getEstimatedPos();
        ChassisSpeeds speeds = getFieldRelativeSpeeds();

        sb_posEstX.setDouble(pose.getX());
        sb_posEstY.setDouble(pose.getY());
        sb_posEstR.setDouble(pose.getRotation().getDegrees());

        sb_speedX.setDouble(speeds.vxMetersPerSecond);
        sb_speedY.setDouble(speeds.vyMetersPerSecond);
        sb_speedR.setDouble(speeds.omegaRadiansPerSecond);
        sb_robotTargetAngle.setDouble(robotTargetAngle);

        field2d.setRobotPose(pose);
        field2d.getObject("fieldTargetPoint").setPose(new Pose2d(point_track_cmd.target, Rotation2d.fromDegrees(0)));
    }

    /**
     * Updates the robot kinematics
     * @param   speeds              Chassis speeds to use for the robot
     */
    private void updateKinematics(ChassisSpeeds speeds) {
        // Convert chassis speeds from field relative to robot relative if in field relative mode
        if(is_field_relative) speeds.toRobotRelativeSpeeds(getEstimatedPos().getRotation());

        // Discretize chassis speeds
        // TODO Set update period from global settings
        speeds.discretize(TimedRobot.kDefaultPeriod);

        // Update Swerve Modules
        var module_states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(module_states, settings.max_drive_speed);

        for(int i = 0; i < modules.length; i++) modules[i].setDesiredState(module_states[i]);
    }

    /**
     * Sets the swerve drive wheels to all point toward the center of the robot so the robot 
     * won't roll freely
     */
    private void updateCrossWheels() {
        for(var module: modules) {
            Rotation2d angle = module.settings.translation.getAngle();
            module.setDesiredState(new SwerveModuleState(0,angle));
        }
    }
            
    /****************************/
    /* Protected Helper Methods */
    /****************************/

    /**
     * Sets the current command to angle_rate_cmd if it is not already running
     */
    protected void runAngleRateCmd() {
        Command cur_command = getCurrentCommand();
        if(cur_command != getDefaultCommand()) cur_command.cancel();
    }

    /**
     * Sets the current command to angle_track_cmd if it is not already running
     */
    protected void runAngleTrackCmd() {
        if(getCurrentCommand() != angle_track_cmd) angle_track_cmd.schedule();
    }

    /**
     * Sets the current command to point_track_cmd if it is not already running
     */
    protected void runPointTrackCmd() {
        if(getCurrentCommand() != point_track_cmd) point_track_cmd.schedule();
    }

    /**
     * Sets the current command to point_track_cmd if it is not already running
     */
    protected void runCrossWheelsCmd() {
        if(getCurrentCommand() != cross_wheels_cmd) cross_wheels_cmd.schedule();
    }
            
    /***************************/
    /* Public Abstract Methods */
    /***************************/
    
    /**
     * Gets the current robot angle relative to the field
     * @return  robot angle relative to the field
     */
    public abstract Rotation2d getAngle();

    /**
     * Gets the current robot angle rate in degrees per second
     * @return  current robot angle rate in degrees per second
     */
    public abstract double getAngleRate();  // TODO Re-implement using Units library
}