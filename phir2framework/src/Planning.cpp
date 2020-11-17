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

    // If you want to access all observed cells (since the start), use this range
    //
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)

    // TODO: classify cells

    // the occupancy type of a cell can be defined as:
    // c->occType = UNEXPLORED
    // c->occType = OCCUPIED
    // c->occType = FREE

    // the planning type of a cell can be defined as:
    // c->planType = REGULAR
    // c->planType = FRONTIER
    // c->planType = DANGER
    // c->planType = NEAR_WALLS
    // c->planType = FRONTIER_NEAR_WALL
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
    for(int xCell = gridLimits.minX; xCell <= gridLimits.maxX; xCell++){
       for(int yCell = gridLimits.minY; yCell <= gridLimits.maxY; yCell++){
           c = grid->getCell(xCell, yCell);

           //potenciais
           if(c->planType == DANGER || c->occType == OCCUPIED){
               c->pot[0] = 1.0;
               c->pot[1] = 1.0;
               c->pot[2] = 1.0;
           }else if(c->planType == FRONTIER_NEAR_WALL){
               c->pot[0] = 0.0;
               c->pot[1] = 0.0;
               c->pot[2] = 0.0;
           }else if(c->planType == FRONTIER){
               c->pot[0] = 0.0;
               c->pot[1] = 0.0;
               c->pot[2] = 1.0;
           }

           //Preferencias
           float pref = 0.4;
           if(c->planType == NEAR_WALLS){
               c->pref = pref;
           }else if(c->occType == FREE){
               c->pref = -pref;
           }
       }
   }
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

    for(int xCell = gridLimits.minX; xCell <= gridLimits.maxX; xCell++){
        for(int yCell = gridLimits.minY; yCell <= gridLimits.maxY; yCell++){
            c = grid->getCell(xCell, yCell);

            if(c->occType == FREE){
                left = grid->getCell(c->x-1, c->y);
                right = grid->getCell(c->x+1, c->y);
                down = grid->getCell(c->x, c->y-1);
                up = grid->getCell(c->x, c->y+1);

                float medPotA = (left->pot[0] + right->pot[0] + down->pot[0] + up->pot[0]) / 4;
                float medPotC = (left->pot[2] + right->pot[2] + down->pot[2] + up->pot[2]) / 4;

                c->pot[0] = medPotA;
                c->pot[2] = medPotC;


                float h = (left->pot[1] + right->pot[1] + down->pot[1] + up->pot[1]) / 4;
                float d = fabs((left->pot[1] - right->pot[1])/2) + fabs((down->pot[1] - up->pot[1])/2);

                c->pot[1] = h-(c->pref/4)*d;
            }

        }
    }

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
    for(int xCell = gridLimits.minX; xCell <= gridLimits.maxX; xCell++){
        for(int yCell = gridLimits.minY; yCell <= gridLimits.maxY; yCell++){
            c = grid->getCell(xCell, yCell);

            if(c->occType == FREE){
                left = grid->getCell(c->x-1, c->y);
                right = grid->getCell(c->x+1, c->y);
                down = grid->getCell(c->x, c->y-1);
                up = grid->getCell(c->x, c->y+1);

                c->dirX[0] = -(right->pot[0] - left->pot[0])/2;
                c->dirX[1] = -(right->pot[1] - left->pot[1])/2;
                c->dirX[2] = -(right->pot[2] - left->pot[2])/2;

                c->dirY[0] = -(up->pot[0] - down->pot[0])/2;
                c->dirY[1] = -(up->pot[1] - down->pot[1])/2;
                c->dirY[2] = -(up->pot[2] - down->pot[2])/2;

                float pNormA = sqrt(pow(c->dirX[0],2)+pow(c->dirY[0], 2));
                float pNormB = sqrt(pow(c->dirX[1],2)+pow(c->dirY[1], 2));
                float pNormC = sqrt(pow(c->dirX[2],2)+pow(c->dirY[2], 2));

                if(pNormA != 0.0){
                    c->dirX[0] = c->dirX[0]/pNormA;
                    c->dirY[0] = c->dirY[0]/pNormA;
                }

                if(pNormB != 0.0){
                    c->dirX[1] = c->dirX[1]/pNormB;
                    c->dirY[1] = c->dirY[1]/pNormB;
                }

                if(pNormC != 0.0){
                    c->dirX[2] = c->dirX[2]/pNormC;
                    c->dirY[2] = c->dirY[2]/pNormC;
                }

            }else{
                c->dirX[0] = 0.0;
                c->dirX[1] = 0.0;
                c->dirX[2] = 0.0;

                c->dirY[0] = 0.0;
                c->dirY[1] = 0.0;
                c->dirY[2] = 0.0;
            }

        }
    }

}
