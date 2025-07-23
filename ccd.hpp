void Arm::updateForwardKinematics(){
    log(INFO,"Updating FK...");
    for (int i = 1; i < joints.size() ; ++i){ //from bottom to top, joints[0] doesnt move since it's the root of the system
        log(DEBUG,"Calculation of new position of joint " + to_string(i) + ". Old position is " + to_string(joints[i].getPos().x) + "," + to_string(joints[i].getPos().y));
        Vector2 newpos {joints[i-1].getPos().x + joints[i-1].getLength() * cos(joints[i-1].getAngle()) , joints[i-1].getPos().y + joints[i-1].getLength() * sin(joints[i-1].getAngle())};
        log(DEBUG,"Joint n " + to_string(i) + " new position due to the rotation of the previous joint is " + to_string(newpos.x) + " " + to_string(newpos.y));
        joints[i].setPos(newpos);
        log(DEBUG,"joint n " + to_string(i) + " updated (" + to_string(joints[i].getPos().x) + "," + to_string(joints[i].getPos().y) + ")");
    }
    //update the end effector position
    log(DEBUG, "End effector old position is " + to_string(endEffector.x) + "," + to_string(endEffector.y));
    endEffector.x = joints[joints.size()-1].getPos().x + joints[joints.size()-1].getLength() * cos(joints[joints.size()-1].getAngle());
    endEffector.y = joints[joints.size()-1].getPos().y + joints[joints.size()-1].getLength() * sin(joints[joints.size()-1].getAngle());  
    log(DEBUG, "End effector new position is " + to_string(endEffector.x) + "," + to_string(endEffector.y));
}

void Arm::rotateToTarget(Vector2 target){
    for (int i = joints.size() - 1 ; i>=0 ; --i){
        //extract joint position
        Vector2 jointpos = joints[i].getPos();

        //calculate the vector that goes from the joint to the endEff
        Vector2 toEndEff {endEffector - jointpos};

        //calculate the vector that goes from the joint to the target
        Vector2 toTarget {target - jointpos};

        //compute the needed angle to actuate the joint
        float alpha = angleBetween(toEndEff,toTarget);

        //saturate up to MAX_ANGLE_PER_STEP
        float gamma = fabs(alpha) < MAX_ANGLE_PER_STEP ? alpha : (alpha > 0 ? MAX_ANGLE_PER_STEP : -MAX_ANGLE_PER_STEP);

        if (computeDist(endEffector, target) < DAMP_THRESHOLD) {
            gamma *= DAMP_FACTOR; // Damp rotation
        }

        //actuate the joint
        log(INFO, "Joint n " + to_string(i) + " is actuated!");
        log(DEBUG, "Old joint n " + to_string(i) + " angle is " + to_string(joints[i].getAngle()));
        joints[i].rotateByAngle(gamma);
        log(DEBUG, "New joint n " + to_string(i) + " angle is " + to_string(joints[i].getAngle()));

        //update angles of the joints that are after the actuated one
        log(INFO, "Updating angles of joints that are after the actuated one");
        for (int j {i} ; j < joints.size() ; ++j){
            joints[j].rotateByAngle(gamma);
            log(DEBUG,"Joint n " + to_string(j) + " angle has been incremented of " + to_string(gamma));
        }

        //update position of the joints
        updateForwardKinematics();
    }
}

void solveInverseKin_CCD(Arm& arm, Vector2 target, float threshold = THRESH, int maxIterations = MAX_ITERATIONS){
    log(INFO, "**SOLVE INVERSE KINEMATIC** Starting distance from target: " + to_string(computeDist(target,arm.getEndEffector())) );
    for (int iter = 0; iter<maxIterations+1; ++iter){
        //check if the target is inside of the scope of the arm
        if (!arm.isTargetReachable(target)) {
            log(SILENT, "Target is not reachable"); 
            break;
        }

        //rotate towards the target
        arm.rotateToTarget(target);
        log(DEBUG,"Iteration n " + to_string(iter) + " end effector is at position: " + to_string(arm.getEndEffector().x) + " " + to_string(arm.getEndEffector().y));
        
        //check if the target is reached
        if (isTargetReached(arm.getEndEffector(),target,threshold)) {
            log(SILENT, "Target reached in " + to_string(iter+1) + " iterations.");
            log(SILENT, "EndEffector is in " + to_string(arm.getEndEffector().x) + ", " + to_string(arm.getEndEffector().y) );
            log(SILENT, "Target is " + to_string(target.x) + ", " + to_string(target.y) );
            log(SILENT, "Distance is " + to_string(computeDist(target, arm.getEndEffector())) );
            log(INFO, "Threshold is " + to_string(threshold) );
            log(SILENT, "Maximum angle per step is " + to_string(MAX_ANGLE_PER_STEP) );
            break;
        }
    
        //at last iteration, if it has not converged, print the failure 
        if (iter == maxIterations) log(SILENT, "CCD failed in reaching the target.");
    }
}

