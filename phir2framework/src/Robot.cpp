#include "Robot.h"

#include <unistd.h>
#include <GL/glut.h>
#include <cmath>
#include <iostream>


//////////////////////////////////////
///// CONSTRUCTORS & DESTRUCTORS /////
//////////////////////////////////////

Robot::Robot()
{
    ready_ = false;
    running_ = true;
    firstIteration = true;

    grid = new Grid();
    mcl = NULL;

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

void Robot::initialize(ConnectionMode cmode, LogMode lmode, std::string fname, std::string mapName)
{
    logMode_ = lmode;

    mcl = new MCL(base.getMaxLaserRange(),mapName,grid->mutex);

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

    Pose odometry = base.getOdometry();
    if(firstIteration){
        prevLocalizationPose_ = odometry;
        firstIteration = false;
    }

    Action u;
    u.rot1 = atan2(odometry.y-prevLocalizationPose_.y,odometry.x-prevLocalizationPose_.x)-DEG2RAD(odometry.theta);
    u.trans = sqrt(pow(odometry.x-prevLocalizationPose_.x,2)+pow(odometry.y-prevLocalizationPose_.y,2));
    u.rot2 = DEG2RAD(odometry.theta)-DEG2RAD(prevLocalizationPose_.theta)-u.rot1;

    // check if there is enough robot motion
    if(u.trans > 0.1 || fabs(u.rot1) > DEG2RAD(30) || fabs(u.rot2) > DEG2RAD(30))
    {
        std::cout << currentPose_ << std::endl;
        mcl->run(u,base.getLaserReadings());
        prevLocalizationPose_ = odometry;
    }

    currentPose_ = odometry;

    pthread_mutex_lock(grid->mutex);

    // Mapping
    mappingWithHIMMUsingLaser();
    mappingWithLogOddsUsingLaser();
    mappingUsingSonar();

    pthread_mutex_unlock(grid->mutex);

    plan->setNewRobotPose(currentPose_);

    // Save path traversed by the robot
    if(base.isMoving() || logMode_==PLAYBACK){
        path_.push_back(currentPose_);
    }

    // Navigation
    switch(motionMode_){
        case WANDER:
            wanderAvoidingCollisions();
            break;
        case WALLFOLLOW:
            wallFollow();
            break;
        case POTFIELD_0:
            followPotentialField(0);
            break;
        case POTFIELD_1:
            followPotentialField(1);
            break;
        case POTFIELD_2:
            followPotentialField(2);
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
    else if(motionMode_==WALLFOLLOW)
        if(dir==LEFT)
            isFollowingLeftWall_=true;
        else if(dir==RIGHT)
            isFollowingLeftWall_=false;
}

void Robot::wanderAvoidingCollisions()
{

    float minLeftSonar  = base.getMinSonarValueInRange(0,2);
    float minFrontSonar = base.getMinSonarValueInRange(3,4);
    float minRightSonar = base.getMinSonarValueInRange(5,7);

    float minLeftLaser  = base.getMinLaserValueInRange(0,74);
    float minFrontLaser = base.getMinLaserValueInRange(75,105);
    float minRightLaser = base.getMinLaserValueInRange(106,180);

    float linVel=0;
    float angVel=0;

   //TODO - implementar desvio de obstaculos
   float min_distance = 1.0;
   std::cout << minLeftSonar << " " <<  minLeftLaser<< "n";
   std::cout << minFrontSonar << " " <<  minFrontLaser<< "n";
   std::cout << minRightSonar << " " <<  minRightLaser<< "n";

   if (minFrontLaser <= min_distance || minFrontSonar <= min_distance){ // Obstacle in front
       if ((minLeftLaser < minRightLaser) || (minLeftSonar < minRightSonar)){
           linVel = 0.0;
           angVel = -1;
       }
       else{
           linVel = 0.0;
           angVel = 1;
       }
   }
   else{
       if (minLeftLaser <= min_distance || minLeftSonar <= min_distance){
           linVel = 20;
           angVel = -1;
       }
       else if (minRightLaser <= min_distance || minRightSonar <= min_distance){
           linVel = 20;
           angVel = 1;
       }
       else{
           linVel = 20;
           angVel = 0;
       }

   }

   base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
}

void Robot::wallFollow()
{
   float minLeftSonar  = base.getMinSonarValueInRange(0,2);
   float minFrontSonar = base.getMinSonarValueInRange(3,4);
   float minRightSonar = base.getMinSonarValueInRange(5,7);

   float minLeftLaser  = base.getMinLaserValueInRange(0,74);
   float minFrontLaser = base.getMinLaserValueInRange(75,105);
   float minRightLaser = base.getMinLaserValueInRange(106,180);

   float linVel=0.5;
   float angVel=0;

   if(isFollowingLeftWall_)
       std::cout << "Following LEFT wall" << std::endl;
   else
       std::cout << "Following RIGHT wall" << std::endl;

   //TODO - implementar wall following usando PID
   float tp = 1.5;
   float td = 11.5;
   float ti = 0.0001;
   float CTE;
   float SP = 1.0;
   static std::vector<float> vec_CTE;

   if(isFollowingLeftWall_){
       CTE = minLeftLaser-SP;
       CTE = -CTE;
       if(m_wallFollowState == true){
           m_wallFollowState = false;
           m_prev_CTE = 0;
           vec_CTE.clear();
       }
   }
   else{
       CTE = minRightLaser-SP;
       if(m_wallFollowState == false){
           m_wallFollowState = true;
           m_prev_CTE = 0;
           vec_CTE.clear();
       }
   }

   float deriv_CTE = CTE - m_prev_CTE;
   float sum_CTE=0;
   vec_CTE.push_back(CTE);
   if (vec_CTE.size() > 50){
       vec_CTE.erase(vec_CTE.begin());
   }
   for (std::vector<float>::iterator it = vec_CTE.begin(); it != vec_CTE.end(); ++it){
       sum_CTE += *it;
   }
   std::cout << "CTE: " << CTE << " prev: " << m_prev_CTE << " sum: " << sum_CTE<< std::endl;
   m_prev_CTE = CTE;
   angVel = -tp*CTE - td*deriv_CTE - ti*sum_CTE;
   std::cout << "Ang Vel: " << angVel << std::endl;

   if(minFrontLaser < 0.8){
       linVel = 0.01;
    }else{
       linVel = 0.5;
    }
    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
}

void Robot::followPotentialField(int t)
{
   int scale = grid->getMapScale();
   int robotX=currentPose_.x*scale;
   int robotY=currentPose_.y*scale;
   float robotAngle = currentPose_.theta;

   // how to access the grid cell associated to the robot position
   Cell* c=grid->getCell(robotX,robotY);

   float linVel = 0.4;
   float angVel = 0.0;
   float tp = 3.0;

   int gradientWindow = 4;
   float dirY=0;
   float dirX=0;
   int num = 0;
   for(int adjX = -sqrt(gradientWindow); adjX<=sqrt(gradientWindow); adjX++){
       for(int adjY = -sqrt(gradientWindow); adjY<=sqrt(gradientWindow); adjY++){
           Cell * windowCell = grid->getCell(c->x+adjX, c->y+adjY);
           dirY+=windowCell->dirY[t];
           dirX+=windowCell->dirX[t];
           ++num;
       }
   }

   dirY=dirY/num;
   dirX=dirX/num;


   float phi = RAD2DEG(atan2(dirY, dirX))-robotAngle;
   phi = normalizeAngleDEG(phi)/90;

   std::cout << "PHI: " << phi << std::endl;


   angVel = tp * phi;

   if(abs(phi) > 170){
       linVel = 0;
    }
    base.setWheelsVelocity_fromLinAngVelocity(linVel,angVel);
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

    Cell *c;

    float p_occ = 0.8, p_free = 0.3;


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
    float maxRange = base.getMaxSonarRange();
    int maxRangeInt = maxRange*scale;

    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;
    float robotAngle = currentPose_.theta;


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

Pose Robot::readInitialPose()
{
    Pose p = logFile_->readPose("Start");
    p.theta = DEG2RAD(p.theta);
    return p;
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

void Robot::drawMCL()
{
    mcl->draw(grid->viewMode-6);
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
