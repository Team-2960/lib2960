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

 package frc.lib2960.pathplanner;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.config.*;
import com.pathplanner.lib.controllers.*;

import frc.lib2960.subsystems.*;
import frc.lib2960.util.*;

/**
 * Initializes the PathPlanner system and 
 */
public class PathPlanner {

    private static SendableChooser<Command> autoChooser = null; /**< Auton Selector */

    /**
     * Constructor
     * @param   dt          Differential drivetrain object reference
     * @param   delta_t     Update period
     */
    public static void init(DiffDriveBase dt, double delta_t){
        // TODO Allow other implementations
        PPLTVController controller = new PPLTVController(delta_t);
        
        init(dt, controller);
    }


    /**
     * Constructor
     * @param   dt          Swerve drivetrain object reference
     * @param   translation Translation PID parameters
     * @param   rotation    Rotation PID parameters
     */
    public static void init(SwerveDriveBase dt, PIDParam translation, PIDParam rotation) {
        // Create Holonomic controller
        PPHolonomicDriveController controller =  new PPHolonomicDriveController(
            toPIDConstants(translation),
            toPIDConstants(rotation)
        );

        // Finish initializing PathPlanner
        init(dt, controller);
    }

    /**
     * Constructor
     * @param   dt          drivetrain object reference
     * @param   controller  PathPlanner controller object
     */
    public static void init(Drivetrain dt, PathFollowingController controller) {
        // Initialize robot config from GUI Settings
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();

            // Initialize Autobuilder
            AutoBuilder.configure(
                dt::getEstimatedPos,
                dt::resetPoseEst,
                dt::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> dt.setRobotRelativeSpeeds(speeds),
                controller,
                config,
                PathPlanner::isRedAlliance,
                dt
            );
    
            // Initialize Shuffleboard
            // TODO Allow a default auton to be set
            autoChooser = AutoBuilder.buildAutoChooser();
    
            var layout = Shuffleboard.getTab("Main")
                .getLayout("Auton", BuiltInLayouts.kList)
                .withSize(1, 4);
            layout.add("Auton Selector", autoChooser);
        } catch (Exception e) {
          // Handle exception as needed
          e.printStackTrace();
        }
    }

    /**
     * Returns the selected auton command
     * @return the selected auton command
     */
    public static Command getSelectedAuto() {
        return autoChooser.getSelected();
    }

    /**
     * Registers commands with the PathPlanner system
     * @param   name    Name of the command to register
     * @param   command Command object to register
     */
    public static void registerCommand(String name, Command command) {
         NamedCommands.registerCommand(name, command);
    }
    
    /**
     * Converts PIDParam to PIDConstants values
     * @param   pid_param   PIDParam values
     * @return  PIDConstants values
     */
    public static PIDConstants toPIDConstants(PIDParam pid_param) {
        return new PIDConstants(pid_param.kP, pid_param.kI, pid_param.kD);
    }

    /**
     * Checks if the current alliance is red
     * @return  true of current alliance is red, false otherwise
     */
    public static boolean isRedAlliance() {
        boolean result = false;
        var alliance = DriverStation.getAlliance();

        if(alliance.isPresent()) result = alliance.get() == DriverStation.Alliance.Red;

        return result;
    }
}