/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib.TrajectoryTools;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * Add your docs here.
 */
public class TrajectoryGroup {

    ArrayList<Trajectory> trajArray;

    public TrajectoryGroup(Trajectory firstTraj){
        ArrayList<Trajectory> trajArray = new ArrayList<Trajectory>();
        trajArray.add(firstTraj);
    }

    public TrajectoryGroup(ArrayList<Trajectory> trajArray){
        this.trajArray = trajArray;
    }

    public void addTrajectory(Trajectory firstTraj){
        trajArray.add(firstTraj);
    }

    public void addTrajectory(ArrayList<Trajectory> trajArray){
        trajArray.addAll(trajArray);
    }

    public Pose2d getInitialPose(){
        return trajArray.get(0).getInitialPose();
    }

    public double getTotalTimeSeconds(){
        double totalTime = 0;
        for (int i = 0; i < trajArray.size(); i++){
            totalTime += trajArray.get(i).getTotalTimeSeconds();
        }
        return totalTime;
    }

    public Trajectory.State sample(double timeSeconds){
        for (int i = 0; i < trajArray.size(); i++){
            if (timeSeconds > trajArray.get(i).getTotalTimeSeconds()){
                timeSeconds -= trajArray.get(i).getTotalTimeSeconds();
            }else{
                return trajArray.get(i).sample(timeSeconds);
            }
        }
        return trajArray.get(trajArray.size()).sample(trajArray.get(trajArray.size()).getTotalTimeSeconds());
    }



}
