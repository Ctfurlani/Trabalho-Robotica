#include "Planning.h"

#include <queue>
#include <float.h> //DBL_MAX
#include <GL/glut.h>

////////////////////////
///                  ///
/// Métodos Públicos ///
///                  ///
////////////////////////

Planning::Planning()
{
    newRobotPosition.x = 0;
    newRobotPosition.y = 0;

    robotPosition = newRobotPosition;

    newGridLimits.minX = newGridLimits.minY = 1000;
    newGridLimits.maxX = newGridLimits.maxY = -1000;

    gridLimits = newGridLimits;
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

    resetCellsTypes();

    // update robot position and grid limits using last position informed by the robot
    robotPosition = newRobotPosition;
    gridLimits = newGridLimits;

    updateCellsTypes();

    pthread_mutex_unlock(grid->mutex);

    initializePotentials();

    for(int i=0; i<100; i++){
        iteratePotentials();
    }

    updateGradient();
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
            c->planType = REGULAR;
        }
    }
}

void Planning::updateCellsTypes()
{
    Cell* c;

    for(int cellX = -maxUpdateRange; cellX<=+maxUpdateRange; cellX++){
        for(int cellY =-maxUpdateRange; cellY<=+maxUpdateRange; cellY++){

            c = grid->getCell(robotPosition.x + cellX, robotPosition.y + cellY);

            if((c->occType==OCCUPIED && c->occupancy < 0.6) || (c->occType == UNEXPLORED && c->occupancy < 0.4)){
                c->occType = FREE;
            }else if((c->occType==FREE && c->occupancy > 0.8) || (c->occType == UNEXPLORED && c->occupancy > 0.6)){
                c->occType = OCCUPIED;
            }

            if(c->occType == FREE){                                                                  //parede
                for(int adjX = -8; adjX<=+8; adjX++){
                    for(int adjY = -8; adjY<=+8; adjY++){
                        if(c->x-adjX > robotPosition.x-maxUpdateRange && c->x+adjX < robotPosition.x+maxUpdateRange
                           && c->y-adjY > robotPosition.y-maxUpdateRange && c->y+adjY < robotPosition.y+maxUpdateRange){         //se estiver dentro do range de atualizacão

                            Cell* adjacentCell = grid->getCell(c->x+adjX, c->y+adjY);

                            int x_dist = abs(c->x-adjacentCell->x);
                            int y_dist = abs(c->y-adjacentCell->y);

                            if(adjacentCell->occType == OCCUPIED){                                          //celula livre proxima a parede
                                if(x_dist<=3 && y_dist<=3){                                             //distancia <= 3 da parede
                                    c->planType = DANGER;
                                }else if(x_dist<=8 && y_dist<=8){                                       //distancia entre 4 e 8 da parede
                                    if(c->planType != DANGER)
                                        c->planType = NEAR_WALLS;                            //evita reclassificacão de celulas
                                }
                            }
                        }
                    }
                }

                for(int inc_x=-1; inc_x<=1; inc_x++){
                    for(int inc_y=-1; inc_y<=1; inc_y++){
                        Cell* frontierCell = grid->getCell(c->x+inc_x, c->y+inc_y);
                        if(frontierCell->occType==UNEXPLORED){

                            if(c->planType==NEAR_WALLS){
                                frontierCell->planType=FRONTIER_NEAR_WALL;
                            }else if(frontierCell->planType!=FRONTIER_NEAR_WALL){
                                frontierCell->planType=FRONTIER;
                            }

                        }
                    }
                }
            }
        }
    }
}

void Planning::initializePotentials()
{
    Cell *c;

    // the potential of a cell is stored in:
    // c->pot[i]
    // the preference of a cell is stored in:
    // c->pref

    // TODO: initialize the potential field in the known map
    //
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)




}

void Planning::iteratePotentials()
{
    Cell* c;
    Cell *left,*right,*up,*down;

    // the update of a FREE cell in position (i,j) will use the potential of the four adjacent cells
    // where, for example:
    //     left  = grid->getCell(i-1,j);


    // TODO: iterate the potential field in the known map
    //
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)





}

void Planning::updateGradient()
{
    Cell* c;

    // the components of the descent gradient of a cell are stored in:
    // c->dirX[i] and c->dirY[i], for pot[i]

    Cell *left,*right,*up,*down;

    // the gradient of a FREE cell in position (i,j) is computed using the potential of the four adjacent cells
    // where, for example:
    //     left  = grid->getCell(i-1,j);


    // TODO: compute the gradient of the FREE cells in the known map
    //
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)







}

