#include "../include/scheduler.h"
#include <semaphore.h>

void *simpleRobotRoutine(void *arg) {
  Robot robot = (Robot)arg;
  Task task = robot->task;
#ifdef DEBUG
    debug_printf(__func__, "Robot%c[%d] starts...\n", 
        RobotTypeToChar(robot->robotType), robot->id);
#endif 
  int jobID;
  while (!queueIsEmpty(task->jobQ)) {
    if(queueIsEmpty(task->jobQ)){
      break;
    }
    queueDequeueFront(task->jobQ, &jobID);
#ifdef DEBUG
    debug_printf(__func__, "Robot%c[%d]: working on job %d...\n", 
        RobotTypeToChar(robot->robotType), robot->id, jobID);
#endif
    simpleWork(jobID, robot);
  }
  pthread_exit(NULL);
}




void simpleWork(int jobID, Robot robot) {
  Task task = robot->task;
  double timer = getCurrentTime();
  switch (jobID) {
  case SKELETON:
#ifdef DEBUG
    debug_printf(__func__, "Robot%c[%d] making skeleton...\n", 
        RobotTypeToChar(robot->robotType), robot->id);
    fflush(stdout);
#endif
    sem_wait(&task->SKELETON_lock);
    sem_wait(&task->lock012);
    makeSkeleton(robot);
    sem_post(&task->lock012);
    break;
  case ENGINE:
#ifdef DEBUG
    debug_printf(__func__, "Robot%c[%d] making engine...\n", 
        RobotTypeToChar(robot->robotType), robot->id);
    fflush(stdout);
#endif
    sem_wait(&task-> ENGINE_lock);
    sem_wait(&task->lock012);
    makeEngine(robot);
    sem_post(&task->lock012);
    break;
  case CHASSIS:
#ifdef DEBUG
    debug_printf(__func__, "Robot%c[%d] making chassis...\n", 
        RobotTypeToChar(robot->robotType), robot->id);
    fflush(stdout);
#endif
    sem_wait(&task->CHASSIS_lock);
    sem_wait(&task->lock012);
    makeChassis(robot);
    sem_post(&task->lock012);
    break;
  case BODY:
    //As we know there will have body task to recycle the storage of engine chassis and skeleton, let the robots to do
    sem_wait(&task->BODY_lock);
    sem_post(&task->ENGINE_lock);
    sem_post(&task->CHASSIS_lock);
    sem_post(&task->SKELETON_lock);
    // task->bodyIsHere = task->bodyIsHere + 1;
    makeBody(robot);
    // sem_post(&task->BODY_lock);
    // task->bodyIsHere = task->bodyIsHere - 1;
    break;
  case WINDOW:
    sem_wait(&task->WINDOW_lock);
    makeWindow(robot);
    break;
  case TIRE:
    sem_wait(&task->TIRE_lock);
    makeTire(robot);
    break;
  case BATTERY:
    sem_wait(&task->BATTERY_lock);
    makeBattery(robot);
    break;
  case CAR:
    //As we confirm there will be a make car recycle the storage let the robots do window tire etc
    sem_post(&task->WINDOW_lock);
    sem_post(&task->WINDOW_lock);
    sem_post(&task->WINDOW_lock);
    sem_post(&task->WINDOW_lock);
    sem_post(&task->WINDOW_lock);
    sem_post(&task->WINDOW_lock);
    sem_post(&task->WINDOW_lock);
    sem_post(&task->TIRE_lock);
    sem_post(&task->TIRE_lock);
    sem_post(&task->TIRE_lock);
    sem_post(&task->TIRE_lock);
    sem_post(&task->BATTERY_lock);
    makeCar(robot);
    sem_post(&task->BODY_lock);
    break;
  default:
    err_printf(__func__, __LINE__, "Error!! Robot%c[%d] gets invalid jobID %d\n", 
        RobotTypeToChar(robot->robotType), robot->id, jobID);
    break;
  }
  timer = getCurrentTime() - timer;
#ifdef DEBUG
  debug_printf(__func__, "Robot%c[%d] job %d done! Time: %lf\n", 
      RobotTypeToChar(robot->robotType), robot->id, jobID, timer);
#endif
}


