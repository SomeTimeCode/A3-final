#include "../include/scheduler.h"

void *simpleRobotRoutine(void *arg) {
  Robot robot = (Robot)arg;
  Task task = robot->task;
#ifdef DEBUG
    debug_printf(__func__, "Robot%c[%d] starts...\n", 
        RobotTypeToChar(robot->robotType), robot->id);
#endif 
  int jobID;
  //the job incoming in the queue
  int front;
  //the number of robots are free in the time with respectively type
  int a;
  int b;
  int c;
  //different robots have different preference to choose to job
  if(robot->robotType == TypeA){
    while (!queueIsEmpty(task->jobQ)) {
      sem_wait(&task->pick);
      b = task->typeB;
      c = task->typeC;
      if(queueIsEmpty(task->jobQ)){
        sem_post(&task->pick);
        break;
      }
      front= *queueFront(task->jobQ);
      if((front== 0 || front== 1|| front==5 || front==7) && c != 0){
        sem_post(&task->pick);
        continue;
      }
      if(front== 6 && b != 0){
        sem_post(&task->pick);
        continue;
      }
      //reduce the number of robots available if assigned a job to do
      task->typeA = task->typeA - 1;
      queueDequeueFront(task->jobQ, &jobID);
      sem_post(&task->pick);
  #ifdef DEBUG
      debug_printf(__func__, "Robot%c[%d]: working on job %d...\n", 
          RobotTypeToChar(robot->robotType), robot->id, jobID);
  #endif
      simpleWork(jobID, robot);
      task->typeA = task->typeA + 1;
    }
    pthread_exit(NULL);
  }else if(robot->robotType == TypeB){
    while (!queueIsEmpty(task->jobQ)) {
      sem_wait(&task->pick);
      a = task->typeA;
      c = task->typeC;
      if(queueIsEmpty(task->jobQ)){
        sem_post(&task->pick);
        break;
      }
      front= *queueFront(task->jobQ);
      if((front== 0 || front== 1|| front==5 || front==7) && c != 0){
        sem_post(&task->pick);
        continue;
      }
      if((front==2||front==3||front==4) && (a!= 0)){
        sem_post(&task->pick);
        continue;
      }
      task->typeB = task->typeB - 1;
      queueDequeueFront(task->jobQ, &jobID);
      sem_post(&task->pick);
  #ifdef DEBUG
      debug_printf(__func__, "Robot%c[%d]: working on job %d...\n", 
          RobotTypeToChar(robot->robotType), robot->id, jobID);
  #endif
      simpleWork(jobID, robot);
      task->typeB = task->typeB + 1;
    }
    pthread_exit(NULL);
  }else{
    while (!queueIsEmpty(task->jobQ)) {
      sem_wait(&task->pick);
      a = task->typeA;
      b = task->typeB;
      if(queueIsEmpty(task->jobQ)){
        sem_post(&task->pick);
        break;
      }
      front= *queueFront(task->jobQ);
      if((front== 6) && b != 0){
        sem_post(&task->pick);
        continue;
      }
      if((front==2||front==3||front==4) && (a!= 0)){
        printf("hi");
        sem_post(&task->pick);
        continue;
      }
      task->typeC = task->typeC - 1;
      queueDequeueFront(task->jobQ, &jobID);
      sem_post(&task->pick);
  #ifdef DEBUG
      debug_printf(__func__, "Robot%c[%d]: working on job %d...\n", 
          RobotTypeToChar(robot->robotType), robot->id, jobID);
  #endif
      simpleWork(jobID, robot);
      task->typeC = task->typeC + 1;
    }
    pthread_exit(NULL);
  }
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
    sem_wait(&task->BODY_lock);
    sem_post(&task->ENGINE_lock);
    sem_post(&task->CHASSIS_lock);
    sem_post(&task->SKELETON_lock);
    makeBody(robot);
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

