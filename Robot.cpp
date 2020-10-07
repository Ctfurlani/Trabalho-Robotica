#include "Robot.h"

#include <unistd.h>
#include <GL/glut.h>
#include <cmath>
#include <iostream>
#include <math.h>


//////////////////////////////////////
///// CONSTRUCTORS & DESTRUCTORS /////
//////////////////////////////////////

Robot::Robot()
{
    ready_ = false;
    running_ = true;

    grid = new Grid();

    plan = new Planning();
    plan->setGrid(grid);
    plan->setMaxUpdateRange(base.getMaxLaserRange());

    // variables used for navigation
    isFollowingLeftWall_=false;

    // variables used for visualization
    viewMode=0;
    numViewModes=5;
    motionMode_=MANUAL_SIMPLE;

}

Robot::~Robot()
{
    base.closeARIAConnection();
    if(grid!=NULL)
        delete grid;
}

////////////////////////////////////
///// INITIALIZE & RUN METHODS /////
////////////////////////////////////

void Robot::initialize(ConnectionMode cmode, LogMode lmode, std::string fname)
{
    logMode_ = lmode;
//    logFile_ = new LogFile(logMode_,fname);
    ready_ = true;

    // initialize ARIA
    if(logMode_!=PLAYBACK){
        bool success = base.initialize(cmode,lmode,fname);
        if(!success){
            printf("Could not connect to robot... exiting\n");
            exit(0);
        }
    }

    ready_ = true;
    controlTimer.startLap();
}

void Robot::run()
{
    controlTimer.waitTime(0.2);

    if(logMode_==PLAYBACK){
        bool hasEnded = base.readFromLog();
        if(hasEnded){
            std::cout << "PROCESS COMPLETE. CLOSING PROGRAM." << std::endl;
            exit(0);
        }
    }else{
        bool success = base.readOdometryAndSensors();
        if(!success){
            usleep(50000);
            return;
        }

        if(logMode_==RECORDING)
            base.writeOnLog();
    }

    currentPose_ = base.getOdometry();

    pthread_mutex_lock(grid->mutex);

    // Mapping
    mappingWithHIMMUsingLaser();
    mappingWithLogOddsUsingLaser();
    mappingUsingSonar();

    pthread_mutex_unlock(grid->mutex);

    plan->setNewRobotPose(currentPose_);

    // Save path traversed by the robot
    if(base.isMoving() || logMode_==PLAYBACK){
        path_.push_back(base.getOdometry());
    }

    // Navigation
    switch(motionMode_){
        case WANDER:
            wanderAvoidingCollisions();
            break;
        case WALLFOLLOW:
            wallFollow();
            break;
        case ENDING:
            running_=false;
            break;
        default:
            break;
    }

    base.resumeMovement();

    usleep(50000);
}

//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void Robot::move(MovingDirection dir)
{
    switch(dir){
        case FRONT:
            std::cout << "moving front" << std::endl;
            break;
        case BACK:
            std::cout << "moving back" << std::endl;
            break;
        case LEFT:
            std::cout << "turning left" << std::endl;
            break;
        case RIGHT:
            std::cout << "turning right" << std::endl;
            break;
        case STOP:
            std::cout << "stopping robot" << std::endl;
    }

    if(motionMode_==MANUAL_SIMPLE)
        base.setMovementSimple(dir);
    else if(motionMode_==MANUAL_VEL)
        base.setMovementVel(dir);
    else if(motionMode_=WALLFOLLOW)
        if(dir==LEFT)
            isFollowingLeftWall_=true;
        else if(dir==RIGHT)
            isFollowingLeftWall_=false;
}

void Robot::wanderAvoidingCollisions()
{
    float linVel=0;
    float angVel=0;

    //TODO - implement obstacle avoidance




    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
}

void Robot::wallFollow()
{
    float linVel=0;
    float angVel=0;

    if(isFollowingLeftWall_)
        std::cout << "Following LEFT wall" << std::endl;
    else
        std::cout << "Following RIGHT wall" << std::endl;

    //TODO - implementar wall following usando PID



    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
}

///////////////////////////
///// MAPPING METHODS /////
///////////////////////////

float Robot::getOccupancyFromLogOdds(float logodds)
{
    return 1.0 - 1.0/(1.0+exp(logodds));
}

float Robot::getLogOddsFromOccupancy(float occupancy)
{
    return log(occupancy/(1.0-occupancy));
}

void Robot::mappingWithLogOddsUsingLaser()
{
    float lambda_r = 0.1; //  10 cm
    float lambda_phi = 1.0;  // 1 degree

    int scale = grid->getMapScale();
    float maxRange = base.getMaxLaserRange();
    int maxRangeInt = maxRange*scale;

    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;
    float robotAngle = currentPose_.theta;

    // how to access a grid cell:
//    Cell* c=grid->getCell(robotX,robotY);

    // access log-odds value of variable in c->logodds
    // how to convert logodds to occupancy values:
//    c->occupancy = getOccupancyFromLogOdds(c->logodds);
      Cell *c;
    // TODO: define fixed values of occupancy
    float p_occ = 0.8, p_free = 0.3;

    // TODO: update cells in the sensors' field-of-view
    // ============================================================================
    // you only need to check the cells at most maxRangeInt from the robot position
    // that is, in the following square region:
    //
    //  (robotX-maxRangeInt,robotY+maxRangeInt)  -------  (robotX+maxRangeInt,robotY+maxRangeInt)
    //                     |                       \                         |
    //                     |                        \                        |
    //                     |                         \                       |
    //  (robotX-maxRangeInt,robotY-maxRangeInt)  -------  (robotX+maxRangeInt,robotY-maxRangeInt)
    for (int i = -maxRangeInt; i <= maxRangeInt; ++i){
        for (int j = -maxRangeInt; j <= maxRangeInt; ++j){

            c = grid->getCell(robotX+i, robotY+j);
            // distance between cell and robot in meters
            float r = pow((c->x - robotX), 2) + pow((c->y - robotY), 2);
            r = sqrt(r)/scale;

            // cell orientation
            double _phi = (atan2(c->y - robotY, c->x - robotX)*(180/M_PI)) - robotAngle;
            float phi = normalizeAngleDEG(_phi);

            // find nearest sensor to the cell
            int k = base.getNearestLaserBeam(phi);
            float k_reading = base.getKthLaserReading(k);
            float k_ang = base.getAngleOfLaserBeam(k);


            if( (r > std::min(maxRange, k_reading + (lambda_r/2))) || (abs(phi - k_ang) > (lambda_phi/2)) ){
                c->logodds = c->logodds;
            }
            else if( (abs(r - k_reading) < (lambda_r/2) ) && (k_reading < maxRange) ){
                c->logodds = c->logodds + getLogOddsFromOccupancy(p_occ);
            }
            else if (r <= k_reading){
                c->logodds = c->logodds + getLogOddsFromOccupancy(p_free);
            }
            c->occupancy = getOccupancyFromLogOdds(c->logodds);

        }
    }

}

void Robot::mappingUsingSonar()
{
    float lambda_r = 0.5; //  50 cm
    float lambda_phi = 30.0;  // 30 degrees


    int scale = grid->getMapScale();
    float maxRange = base.getMaxLaserRange();
    int maxRangeInt = maxRange*scale;

    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;
    float robotAngle = currentPose_.theta;

    // TODO: update cells in the sensors' field-of-view
    // Follow the example in mappingWithLogOddsUsingLaser()
    for (int i = -maxRangeInt; i < maxRangeInt; ++i){
        for (int j = -maxRangeInt; j < maxRangeInt; ++j){

            Cell *c = grid->getCell(robotX+i, robotY+j);
            // distance between cell and robot in meters
            float r = pow((c->x - robotX), 2) + pow((c->y - robotY), 2);
            r = sqrt(r)/scale;

            // cell orientation
            double _phi = atan2(c->y - robotY, c->x - robotX)*180/M_PI - robotAngle;
            float phi = normalizeAngleDEG(_phi);

            // find nearest sensor to the cell
            int k = base.getNearestSonarBeam(phi);
            float k_reading = base.getKthSonarReading(k);
            float k_ang = base.getAngleOfSonarBeam(k);


            float beta = lambda_phi/2;
            float alpha = abs(phi - k_ang);
            float const_term = ((maxRange - r)/maxRange + (beta - alpha)/beta)/2;
            float occUpdate;

            if( (r > std::min(maxRange, k_reading + lambda_r/2)) || (abs(phi - k_ang) > (lambda_phi/2)) ){
                c->occupancySonar= c->occupancySonar;
            }
            else if( (abs(r - k_reading) < (lambda_r/2) ) && (k_reading < maxRange) ){ // occ
                occUpdate = 0.5*(const_term) + 0.5;
                c->occupancySonar = (occUpdate*c->occupancySonar)/( (occUpdate*c->occupancySonar)+((1-occUpdate)*(1-c->occupancySonar)) );
            }
            else if (r <= k_reading){//free
                occUpdate = 0.5*(1 - const_term );
                c->occupancySonar = (occUpdate*c->occupancySonar)/( (occUpdate*c->occupancySonar)+((1-occUpdate)*(1-c->occupancySonar)) );
            }
            if (c->occupancySonar < 0.01) c->occupancySonar = 0.01;
            if (c->occupancySonar > 0.99) c->occupancySonar = 0.99;
        }
    }

}

void Robot::mappingWithHIMMUsingLaser()
{
    float lambda_r = 0.2; //  20 cm
    float lambda_phi = 1.0;  // 1 degree

    int scale = grid->getMapScale();
    float maxRange = base.getMaxLaserRange();
    int maxRangeInt = maxRange*scale;

    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;
    float robotAngle = currentPose_.theta;

    // TODO: update cells in the sensors' field-of-view
    // Follow the example in mappingWithLogOddsUsingLaser()
    for (int i = -maxRangeInt; i < maxRangeInt; ++i){
        for (int j = -maxRangeInt; j < maxRangeInt; ++j){

            Cell *c = grid->getCell(robotX+i, robotY+j);
            // distance between cell and robot in meters
            float r = pow((c->x - robotX), 2) + pow((c->y - robotY), 2);
            r = sqrt(r)/scale;

            // cell orientation
            double _phi = atan2(c->y - robotY, c->x - robotX)*180/M_PI - robotAngle;
            float phi = normalizeAngleDEG(_phi);

            // find nearest sensor to the cell
            int k = base.getNearestLaserBeam(phi);
            float k_reading = base.getKthLaserReading(k);
            float k_ang = base.getAngleOfLaserBeam(k);


            if( (r > std::min(maxRange, k_reading + lambda_r/2)) || (abs(phi - k_ang) > (lambda_phi/2)) ){
                c->himm = c->himm;
            }
            else if( (abs(r - k_reading) < (lambda_r/2) ) && (k_reading < maxRange) ){// occ
                c->himm+=3;
                if (c->himm > 15) c->himm = 15;
            }
            else if (r <= k_reading){//free
                --c->himm;
                if (c->himm < 0) c->himm = 0;
            }

        }
    }

}

/////////////////////////////////////////////////////
////// METHODS FOR READING & WRITING ON LOGFILE /////
/////////////////////////////////////////////////////

// Prints to file the data that we would normally be getting from sensors, such as the laser and the odometry.
// This allows us to later play back the exact run.
void Robot::writeOnLog()
{
    logFile_->writePose("Odometry",currentPose_);
    logFile_->writeSensors("Sonar",base.getSonarReadings());
    logFile_->writeSensors("Laser",base.getLaserReadings());
}

// Reads back into the sensor data structures the raw readings that were stored to file
// While there is still information in the file, it will return 0. When it reaches the end of the file, it will return 1.
bool Robot::readFromLog() {

    if(logFile_->hasEnded())
        return true;

    base.setOdometry(logFile_->readPose("Odometry"));
    base.setSonarReadings(logFile_->readSensors("Sonar"));
    base.setLaserReadings(logFile_->readSensors("Laser"));

    return false;
}

////////////////////////
///// DRAW METHODS /////
////////////////////////

void Robot::draw(float xRobot, float yRobot, float angRobot)
{
    float scale = grid->getMapScale();
    glTranslatef(xRobot,yRobot,0.0);
    glRotatef(angRobot,0.0,0.0,1.0);
    glScalef(1.0/scale,1.0/scale,1.0/scale);

    // sonars and lasers draw in cm
    if(viewMode==1)
        base.drawSonars(true);
    else if(viewMode==2)
        base.drawSonars(false);
    else if(viewMode==3)
        base.drawLasers(true);
    else if(viewMode==4)
        base.drawLasers(false);

    // robot draw in cm
    base.drawBase();

    glScalef(scale,scale,scale);
    glRotatef(-angRobot,0.0,0.0,1.0);
    glTranslatef(-xRobot,-yRobot,0.0);
}

/////////////////////////
///// OTHER METHODS /////
/////////////////////////

bool Robot::isReady()
{
    return ready_;
}

bool Robot::isRunning()
{
    return running_;
}

const Pose& Robot::getCurrentPose()
{
    return currentPose_;
}

void Robot::drawPath()
{
    float scale = grid->getMapScale();

    if(path_.size() > 1){
        glScalef(scale,scale,scale);
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            for(unsigned int i=0;i<path_.size()-1; i++){
                glColor3f(1.0,0.0,1.0);

                glVertex2f(path_[i].x, path_[i].y);
                glVertex2f(path_[i+1].x, path_[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
        glScalef(1.0/scale,1.0/scale,1.0/scale);

    }
}

void Robot::waitTime(float t){
    float l;
    do{
        usleep(1000);
        l = controlTimer.getLapTime();
    }while(l < t);
    controlTimer.startLap();
}
