#include "Planning.h"

#include <queue>
#include <float.h> //DBL_MAX

////////////////////////
///                  ///
/// Métodos Públicos ///
///                  ///
////////////////////////

Planning::Planning()
{
    newRobotPosition.x = 0;
    newRobotPosition.y = 0;

    newGridLimits.minX = newGridLimits.minY = 1000;
    newGridLimits.maxX = newGridLimits.maxY = -1000;
}

Planning::~Planning()
{}

void Planning::setGrid(Grid *g)
{
    grid = g;
}

void Planning::setMaxUpdateRange(int r)
{
    maxUpdateRange = 1.2*r*grid->getMapScale();
}

void Planning::setNewRobotPose(Pose p)
{
    newRobotPosition.x = (int)(p.x*grid->getMapScale());
    newRobotPosition.y = (int)(p.y*grid->getMapScale());

    newGridLimits.minX = std::min(newGridLimits.minX,newRobotPosition.x-maxUpdateRange);
    newGridLimits.maxX = std::max(newGridLimits.maxX,newRobotPosition.x+maxUpdateRange);
    newGridLimits.minY = std::min(newGridLimits.minY,newRobotPosition.y-maxUpdateRange);
    newGridLimits.maxY = std::max(newGridLimits.maxY,newRobotPosition.y+maxUpdateRange);
}

void Planning::run()
{
    pthread_mutex_lock(grid->mutex);

    // resetCellsTypes();

    // update robot position and grid limits using last position informed by the robot
    robotPosition = newRobotPosition;
    gridLimits = newGridLimits;

    updateCellsTypes();

    pthread_mutex_unlock(grid->mutex);
}

/////////////////////////////////////////////
///                                       ///
/// Métodos para classificacao de celulas ///
///                                       ///
/////////////////////////////////////////////

void Planning::resetCellsTypes()
{
    for(int i=gridLimits.minX;i<=gridLimits.maxX;i++){
        for(int j=gridLimits.minY;j<=gridLimits.maxY;j++){

            Cell* c = grid->getCell(i,j);

            c->occType = UNEXPLORED;
            c->planType = REGULAR;
        }
    }
}

void Planning::updateCellsTypes()
{
    Cell* c;

    for(int cellX = -maxUpdateRange; cellX<=+maxUpdateRange; cellX++){
        for(int cellY =-maxUpdateRange; cellY<=robotPosition.y+maxUpdateRange; cellY++){

            c = grid->getCell(robotPosition.x + cellX, robotPosition.y + cellY);

            if((c->occType==OCCUPIED && c->occupancy < 0.6) || (c->occType == UNEXPLORED && c->occupancy < 0.4)){
                c->occType = FREE;
            }else if((c->occType==FREE && c->occupancy > 0.8) || (c->occType == UNEXPLORED && c->occupancy > 0.6)){
                c->occType = OCCUPIED;
            }

            for(int adjX = -3; adjX<=+3; adjX++){
                for(int adjY = -3; adjY<=+3; adjY++){
                    Cell* adjacentCell = grid->getCell(c->x+adjX, c->y+adjY);
                    if(adjacentCell->occType == UNEXPLORED && c->occType == FREE && abs(c->x-adjacentCell->x)<=1 && abs(c->y-adjacentCell->y)<=1){
                        adjacentCell->planType = FRONTIER;
                    }else if(adjacentCell->occType == FREE && c->occType == OCCUPIED){
                        adjacentCell->planType = DANGER;
                    }
                }
            }

        }
    }
}

