/**************************************************************************
  CSC C85 - Fall 2013 - UTSC RoboSoccer AI core

  This file is where the actual planning is done and commands are sent
  to the robot.

  Please read all comments in this file, and add code where needed to
  implement your game playing logic. 

  Things to consider:

  - Plan - don't just react
  - Use the heading vectors!
  - Mind the noise (it's everywhere)
  - Try to predict what your oponent will do
  - Use feedback from the camera

  What your code should not do: 

  - Attack the opponent, or otherwise behave aggressively toward the
    oponent
  - Hog the ball (kick it, push it, or leave it alone)
  - Sit at the goal-line or inside the goal
  - Run completely out of bounds

  AI scaffold: Parker-Lee-Estrada, Summer 2013

  Release version: 0.1
***************************************************************************/

#include "imagecapture/imageCapture.h"
#include "roboAI.h"     // <--- Look at this header file!
#include <nxtlibc/nxtlibc.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define rad_to_degree 57.2957795
// global variable
int c = 0;
double h = 0.0;
double total = 0.0;
int side = 0;
int a = 0;
int change = 0;
double mx, my = 0.0;
int flag = 0;
double perfect_line_x = 0.0;
double perfect_line_y = 0.0;
double before_turn_x = 0.0;
double before_turn_y = 0.0;

int count = 0;

// come back to respective state.
int where = 0;

void clear_motion_flags(struct RoboAI *ai)
{
 // Reset all motion flags. See roboAI.h for what each flag represents
 ai->st.mv_fwd=0; 
 ai->st.mv_back=0;
 ai->st.mv_bl=0;
 ai->st.mv_br=0;
 ai->st.mv_fl=0;
 ai->st.mv_fr=0;
}

struct blob *id_coloured_blob(struct RoboAI *ai, struct blob *blobs, int col)
{
 // ** DO NOT CHANGE THIS FUNCTION **
 // Find a blob with the specified colour. If similar colour blobs around,
 // choose the most saturated one.
 // Colour parameter: 0 -> R
 //                   1 -> G
 //                   2 -> B
 struct blob *p, *fnd;
 double BCRT=1.0;     // Ball colour ratio threshold
 double c1,c2,c3,m;
 int i;

 p=blobs;
 fnd=NULL;
 while (p!=NULL)
 {
  if (col==0) {c1=p->R; c2=p->G; c3=p->B;}   // detect red
  else if (col==1) {c1=p->G; c2=p->R; c3=p->B;} // detect green
  else if (col==2){c1=p->B; c2=p->G; c3=p->R;}  // detect blue

  if (c1/c2>BCRT&&c1/c3>BCRT&&p->tracked_status==1) // tracked coloured blob!
  {
//   fprintf(stderr,"col=%d, c1/c2=%f, c1/c3=%f, blobId=%d\n",col,c1/c2,c1/c3,p->blobId);
   if (c1/c2<c1/c3) m=c1/c2; else m=c1/c3;

   if (fnd==NULL) fnd=p;      // first one so far
   else 
    if (col==0)         // Fond a more colorful one!
       if (m>(fnd->R/fnd->G) || m>(fnd->R/fnd->B)) fnd=p;
    else if (col==1)
       if (m>(fnd->G/fnd->R) || m>(fnd->G/fnd->B)) fnd=p;
    else
       if (m>(fnd->B/fnd->G) || m>(fnd->B/fnd->R)) fnd=p;
  }
  p=p->next;
 }
// if (fnd!=NULL) fprintf(stderr,"Selected for col=%d, blobId=%d\n",col,fnd->blobId);
 return(fnd);
}

void track_agents(struct RoboAI *ai, struct blob *blobs)
{
 // ** DO NOT CHANGE THIS FUNCTION **
 // Find the current blobs that correspond to the bot, opponent, and ball
 //  by detecting blobs with the specified colours. One bot is green,
 //  one bot is blue. Ball is always red.

 // NOTE: Add a command-line parameter to specify WHICH bot is own

 struct blob *p;
 double mg,vx,vy;
 double NOISE_VAR=15;

 // Find the ball
 p=id_coloured_blob(ai,blobs,0);
 if (p)
 {
  if (ai->st.ball!=NULL&&ai->st.ball!=p)
  {
   ai->st.ball->idtype=0;
   ai->st.ball->p=0;
  }
  ai->st.ballID=1;
  ai->st.ball=p;
  ai->st.bvx=p->cx[0]-ai->st.old_bcx;
  ai->st.bvy=p->cy[0]-ai->st.old_bcy;
  ai->st.old_bcx=p->cx[0]; 
  ai->st.old_bcy=p->cy[0];
  ai->st.ball->p=0;
  ai->st.ball->idtype=3;
  vx=ai->st.bvx;
  vy=ai->st.bvy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)     // Enable? disable??
  {
   ai->st.ball->vx[0]=vx;
   ai->st.ball->vy[0]=vy;
   vx/=mg;
   vy/=mg;
   ai->st.ball->mx=vx;
   ai->st.ball->my=vy;
  }
 }
 else {
  ai->st.ball=NULL;
 }
 
 // ID our bot
 if (ai->st.botCol==0) p=id_coloured_blob(ai,blobs,1);
 else p=id_coloured_blob(ai,blobs,2);
 if (p)
 {
  if (ai->st.self!=NULL&&ai->st.self!=p)
  {
   ai->st.self->idtype=0;
   ai->st.self->p=0;
  }
  ai->st.selfID=1;
  ai->st.self=p;
  ai->st.svx=p->cx[0]-ai->st.old_scx;
  ai->st.svy=p->cy[0]-ai->st.old_scy;
  ai->st.old_scx=p->cx[0]; 
  ai->st.old_scy=p->cy[0];
  ai->st.self->p=0;
  ai->st.self->idtype=1;
 }
 else ai->st.self=NULL;

 // ID our opponent
 if (ai->st.botCol==0) p=id_coloured_blob(ai,blobs,2);
 else p=id_coloured_blob(ai,blobs,1);
 if (p)
 {
  if (ai->st.opp!=NULL&&ai->st.opp!=p)
  {
   ai->st.opp->idtype=0;
   ai->st.opp->p=0;
  }
  ai->st.oppID=1;
  ai->st.opp=p;
  ai->st.ovx=p->cx[0]-ai->st.old_ocx;
  ai->st.ovy=p->cy[0]-ai->st.old_ocy;
  ai->st.old_ocx=p->cx[0]; 
  ai->st.old_ocy=p->cy[0];
  ai->st.opp->p=0;
  ai->st.opp->idtype=2;
 }
 else ai->st.opp=NULL;

}

void id_bot(struct RoboAI *ai, struct blob *blobs)
{
 // ** DO NOT CHANGE THIS FUNCTION **
 // This routine calls track_agents() to identify the blobs corresponding to the
 // robots and the ball. It commands the bot to move forward slowly so heading
 // can be established from blob-tracking.
 //
 // NOTE 1: All heading estimates are noisy. 
 //
 // NOTE 2: Heading estimates are only valid when the robot moves with a
 //         forward or backward direction. Turning destroys heading data
 //         (why?)
 //
 // You should *NOT* call this function during the game. This is only for the
 // initialization step. Calling this function during the game will result in
 // unpredictable behaviour since it will update the AI state.
 
 struct blob *p;
 static double stepID=0;
 double frame_inc=1.0/5.0;

 drive_speed(30);   // Need a few frames to establish heading

 track_agents(ai,blobs);

 if (ai->st.selfID==1&&ai->st.self!=NULL)
  fprintf(stderr,"Successfully identified self blob at (%f,%f)\n",ai->st.self->cx[0],ai->st.self->cy[0]);
 if (ai->st.oppID==1&&ai->st.opp!=NULL)
  fprintf(stderr,"Successfully identified opponent blob at (%f,%f)\n",ai->st.opp->cx[0],ai->st.opp->cy[0]);
 if (ai->st.ballID==1&&ai->st.ball!=NULL)
  fprintf(stderr,"Successfully identified ball blob at (%f,%f)\n",ai->st.ball->cx[0],ai->st.ball->cy[0]);

 stepID+=frame_inc;
 if (stepID>=1&&ai->st.selfID==1)
 {
  ai->st.state+=1;
  stepID=0;
  all_stop();
 }
 else if (stepID>=1) stepID=0;

 return;
}

int setupAI(int mode, int own_col, struct RoboAI *ai)
{
 // ** DO NOT CHANGE THIS FUNCTION **
 // This sets up the initial AI for the robot. There are three different modes:
 //
 // SOCCER -> Complete AI, tries to win a soccer game against an opponent
 // PENALTY -> Score a goal (no goalie!)
 // CHASE -> Kick the ball and chase it around the field
 //
 // Each mode sets a different initial state (0, 100, 200). Hence, 
 // AI states for SOCCER will be 0 through 99
 // AI states for PENALTY will be 100 through 199
 // AI states for CHASE will be 200 through 299
 //
 // You will of course have to add code to the AI_main() routine to handle
 // each mode's states and do the right thing.
 //
 // Your bot should not become confused about what mode it started in!
         
 switch (mode) {
 case AI_SOCCER:
  fprintf(stderr,"Standard Robo-Soccer mode requested\n");
        ai->st.state=0;   // <-- Set AI initial state to 0
        break;
 case AI_PENALTY:
  fprintf(stderr,"Penalty mode! let's kick it!\n");
  ai->st.state=100; // <-- Set AI initial state to 100
        break;
 case AI_CHASE:
  fprintf(stderr,"Chasing the ball...\n");
  ai->st.state=200; // <-- Set AI initial state to 200
        break;  
 default:
  fprintf(stderr, "AI mode %d is not implemented, setting mode to SOCCER\n", mode);
  ai->st.state=0;
  }

 all_stop();      // Stop bot,
 ai->runAI = AI_main;   // and initialize all remaining AI data
 ai->st.ball=NULL;
 ai->st.self=NULL;
 ai->st.opp=NULL;
 ai->st.side=0;
 ai->st.botCol=own_col;
 ai->st.old_bcx=0;
 ai->st.old_bcy=0;
 ai->st.old_scx=0;
 ai->st.old_scy=0;
 ai->st.old_ocx=0;
 ai->st.old_ocy=0;
 ai->st.bvx=0;
 ai->st.bvy=0;
 ai->st.svx=0;
 ai->st.svy=0;
 ai->st.ovx=0;
 ai->st.ovy=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ballID=0;
 clear_motion_flags(ai);
 fprintf(stderr,"Initialized!\n");

 return(1);
}

void AI_main(struct RoboAI *ai, struct blob *blobs, void *state)
{
 /*************************************************************************
  You will be working with a state-based AI. You are free to determine
  how many states there will be, what each state will represent, and
  what actions the robot will perform based on the state as well as the
  state transitions.

  You must *FULLY* document your state representation in the report

  Here two states are defined:
  State 0,100,200 - Before robot ID has taken place (this state is the initial
                  state, or is the result of pressing 'r' to reset the AI)
  State 1,101,201 - State after robot ID has taken place. At this point the AI
                  knows where the robot is, as well as where the opponent and
                  ball are (if visible on the playfield)

  Relevant UI keyboard commands:
  'r' - reset the AI. Will set AI state to zero and re-initialize the AI
  data structure.
  't' - Toggle the AI routine (i.e. start/stop calls to AI_main() ).
  'o' - Robot immediate all-stop! - do not allow your NXT to get damaged!

  ** Do not change the behaviour of the robot ID routine **
 *************i*************************************************************/

 if (ai->st.state==0||ai->st.state==100||ai->st.state==200)   // Initial set up - find own, ball, and opponent blobs
 {
  // Carry out self id process.
  fprintf(stderr,"Initial state, self-id in progress...\n");
  id_bot(ai,blobs);
  //initial my setup
  flag = 0;
  if ((ai->st.state%100)!=0)  // The id_bot() routine will change the AI state to initial state + 1
  {       // if robot identification is successful.
   if (ai->st.self->cx[0]>=512) ai->st.side=1; else ai->st.side=0;
   all_stop();
   clear_motion_flags(ai);
   fprintf(stderr,"Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], AI state=%d\n",ai->st.self->cx[0],ai->st.self->cy[0],ai->st.self->mx,ai->st.self->my,ai->st.state);
  }

}else if ((1 <= ai->st.state) && (99 > ai->st.state))
 {

  if (ai->st.state == 1){
    // determine which side we are on
    int s = side_checker(ai);

    if (s){
      fprintf(stderr, "left of camera \n");
      ai->st.state = 2;
    }else{
      fprintf(stderr, "right of camera\n");
      ai->st.state = 52;
    }
  } 

  if (ai->st.state == 2){

    if (150.0 < ai->st.self->cx[0]){
      fprintf(stderr, "%f   #########\n", ai->st.self->cx[0]);
      drive_speed(-50);
    }else{
      ai->st.state = 3;
    }

  }

  if (ai->st.state == 3){
    fprintf(stderr, "do the three way \n");
    drive_speed(70);
    usleep(1000000);
    drive_speed(-70);
    usleep(1000000);
    pivot_left_speed(30);
    usleep(500000);
    drive_speed(70);
    usleep(1000000);
    drive_speed(-70);
    usleep(1000000);
    pivot_right_speed(30);
    usleep(500000);
    drive_speed(70);
    usleep(1000000);
    drive_speed(-70);
    usleep(1000000);
    pivot_right_speed(30);
    usleep(500000);
    drive_speed(70);
    usleep(1000000);
    drive_speed(-70);
    usleep(1000000);
    pivot_left_speed(30);
    usleep(500000);

    
  }



  if (ai->st.state == 52){

    if( (ai->st.ball->cx[0]*2)-150 > ai->st.self->cx[0] ){
      drive_speed(-50);
    }else{
      ai->st.state = 53;
    }
    
  }

  if (ai->st.state == 53 ){
    fprintf(stderr, "do the three way \n");
    drive_speed(70);
    usleep(1000000);
    drive_speed(-70);
    usleep(1000000);
    pivot_left_speed(30);
    usleep(500000);
    drive_speed(70);
    usleep(1000000);
    drive_speed(-70);
    usleep(1000000);
    pivot_right_speed(30);
    usleep(500000);
    drive_speed(70);
    usleep(1000000);
    drive_speed(-70);
    usleep(1000000);
    pivot_right_speed(30);
    usleep(500000);
    drive_speed(70);
    usleep(1000000);
    drive_speed(-70);
    usleep(1000000);
    pivot_left_speed(30);
    usleep(500000);
  }








 }else if ((101 <= ai->st.state) && (199 > ai->st.state))
 {            // if we choose penalty kick
 

              if (ai->st.state == 101){
              // check which side we are on.
                a = above_or_below(ai);
                if (a == 1){
                  // above
                  ai->st.state = 152;
                }else if(a == -1){
                  // below
                  ai->st.state = 102;
                }


              }else if (ai->st.state == 102){
                // bot is below the ball.
                if (isPerpendicular(ai)){
                  // bot is perpendicular to the field
                  ai->st.state = 110;
                }else{
                  // bot is NOT perpendicular to the field
                  ai->st.state = 103;
                }

              }else if (ai->st.state == 103){
                // bot is below the ball, NOT perpendicular to the field
                if (change==0){

                makePerpendicular(ai,blobs,1);
                change=1;
                }else if (change == 1){
                  change = 2;
                  makePerpendicular(ai,blobs,1);
                  
                  if ((ai->st.self->my != 0.0) && (ai->st.self->mx != 0.0)){
                    my = (ai->st.self->my) * -1.0;
                    mx = (ai->st.self->mx) * -1.0;
                  }
                  ai->st.self->my = 0.0;
                  ai->st.self->mx = 0.0;
                  fprintf(stderr, "%f\n", my);
                      if (my > 0.0){
                      fprintf(stderr, "we got to turn 180* \n");
                      pivot_right_speed(40);
                      usleep(2100000);
                    }

                }else if (change == 2){
                  makePerpendicular(ai,blobs,0);
                  change=3;

                }else if (change == 3){
                  makePerpendicular(ai,blobs,-1);
                  change = 4;
                 
                }
                else if (change == 4){
                  makePerpendicular(ai,blobs,-1);
                  ai->st.self->my = 0.0;
                  ai->st.self->mx = 0.0;
                  change=0;
                }
                ai->st.state = 102;
              }


              else if (ai->st.state == 110){
                center_y(ai,blobs,-1);
               
              }
              else if (ai->st.state == 111){

                if (fabs(ai->st.self->cx[0] - ai->st.ball->cx[0]) < 200 ){
                    fprintf(stderr, "getting ready to kick \n");
                    kick();
                    usleep(1250000);
                    ai->st.state = 198;
                }else{
                    
                    drive_speed(30);



                }
              }




              else if (ai->st.state == 152){
                // bot is below the ball.
                if (isPerpendicular(ai)){
                  // bot is perpendicular to the field
                  ai->st.state = 160;
                }else{
                  // bot is NOT perpendicular to the field
                  ai->st.state = 153;
                }

              }else if (ai->st.state ==153){






             // bot is below the ball, NOT perpendicular to the field
                if (change==0){

                makePerpendicular(ai,blobs,1);
                change=1;

                }else if (change == 1){
                  change = 2;
                  makePerpendicular(ai,blobs,1);

                  if ((ai->st.self->my != 0.0) && (ai->st.self->mx != 0.0)){

                    my = (ai->st.self->my) * -1.0;
                    mx = (ai->st.self->mx) * -1.0;
                  }

                  ai->st.self->my = 0.0;
                  ai->st.self->mx = 0.0;
                  fprintf(stderr, "%f\n", my);
                      if (my < 0.0){
                      fprintf(stderr, "turn 180* \n");
                      pivot_right_speed(40);
                      usleep(2100000);
                      }

                }else if (change == 2){
                  makePerpendicular(ai,blobs,0);
                  change=3;

                }else if (change == 3){
                  makePerpendicular(ai,blobs,-1);
                  change = 4;
                 
                }
                else if (change == 4){
                  makePerpendicular(ai,blobs,-1);
                  ai->st.self->my = 0.0;
                  ai->st.self->mx = 0.0;
                  change=0;
                }
                ai->st.state = 152;

              }else if (ai->st.state == 160){
                center_y(ai,blobs,1);
               

              }else if (ai->st.state ==161){

                if (fabs(ai->st.self->cx[0] - ai->st.ball->cx[0]) < 200 ){
                    kick();
                    usleep(1250000);
                    ai->st.state = 198;
                }else {
                    drive_speed(30);
                }



              }else if (ai->st.state == 198){                 
                fprintf(stderr, "Let me rest\n");
                all_stop();
                if (where == 2){
                  // re init stuff
                      int c = 0;
                      double h = 0.0;
                      double total = 0.0;
                      int side = 0;
                      int a = 0;
                      int change = 0;
                      double mx, my = 0.0;
                  //
                      ai->st.ball->vy[0] = 320;
                      ai->st.ball->vy[1] = 320;
                      ai->st.ball->vy[2] = 320;
                      ai->st.ball->vy[3] = 320;
                      ai->st.ball->vx[0] = 320;
                      ai->st.ball->vx[1] = 320;
                      ai->st.ball->vx[2] = 320;
                      ai->st.ball->vx[3] = 320;
                  // 

                  fprintf(stderr, "Going to track ball again, give me some time\n");
                  sleep(10);
                  fprintf(stderr, "I rested enuf, back to work\n");
                  ai->st.state = 200;
                } 
              }
  }else if ((201 <= ai->st.state) && (299 > ai->st.state)){
    where = 2;

    fprintf(stderr, "vx0 %f\n", ai->st.ball->vx[0]);
    fprintf(stderr, "vx1 %f\n", ai->st.ball->vx[1]);
    fprintf(stderr, "vx2 %f\n", ai->st.ball->vx[2]);
    fprintf(stderr, "vx3 %f\n", ai->st.ball->vx[3]);
    

    fprintf(stderr, "vy0 %f\n", ai->st.ball->vy[0]);
    fprintf(stderr, "vy1 %f\n", ai->st.ball->vy[1]);
    fprintf(stderr, "vy2 %f\n", ai->st.ball->vy[2]);
    fprintf(stderr, "vy3 %f\n", ai->st.ball->vy[3]);


    if(   (ai->st.ball->vx[0] > -1.7 && ai->st.ball->vx[0] < 1.7) &&
          (ai->st.ball->vy[0] > -1.7 && ai->st.ball->vy[0] < 1.7) &&
          (ai->st.ball->vx[1] > -1.7 && ai->st.ball->vx[1] < 1.7) &&
          (ai->st.ball->vy[1] > -1.7 && ai->st.ball->vy[1] < 1.7) &&
          (ai->st.ball->vx[2] > -1.7 && ai->st.ball->vx[2] < 1.7) &&
          (ai->st.ball->vy[2] > -1.7 && ai->st.ball->vy[2] < 1.7) &&
          (ai->st.ball->vx[3] > -1.7 && ai->st.ball->vx[3] < 1.7) &&
          (ai->st.ball->vy[3] > -1.7 && ai->st.ball->vy[3] < 1.7) )
    {
          ai->st.state = 100;
    }



  }
  else{
  /****************************************************************************
   TO DO:
   You will need to replace this 'catch-all' code with actual program logic to
   have the robot do its work depending on its current state.
   After id_bot() has successfully completed its work, the state should be
   1 - if the bot is in SOCCER mode
   101 - if the bot is in PENALTY mode
   201 - if the bot is in CHASE mode

   Your AI code needs to handle these states and their associated state
   transitions which will determine the robot's behaviour for each mode.
  *****************************************************************************/
  track_agents(ai,blobs);   // Currently, does nothing but endlessly track
  fprintf(stderr,"Just trackin'!\n"); // bot, opponent, and ball.
 }
}



/**********************************************************************************
 TO DO:

 Add the rest of your game playing logic below. Create appropriate functions to
 handle different states (be sure to name the states/functions in a meaningful
 way), and do any processing required in the space below.

 AI_main() should *NOT* do any heavy lifting. It should only call appropriate
 functions based on the current AI state.

 You will lose marks if AI_main() is cluttered with code that doesn't belong
 there.
**********************************************************************************/


int above_or_below(struct RoboAI *ai){
    if (h < 10.0){
      h += 1.0;
      total += (ai->st.ball->cy[0] - ai->st.self->cy[0]);
      if (h == 10.0){
        if ((total) > 0.0){
          fprintf(stderr, "bot is above \n");
          return 1;
        }else{
          fprintf(stderr, "bot is below \n");
          return -1;
        }
      }
    }else{
      h = 0.0;
      total = 0.0;
      side = 0;
      return 0;
    }
  }
int isPerpendicular(struct RoboAI *ai)
{

      if (a == -1){
        if (my < -0.960 && my > -0.999)
        {
          fprintf(stderr, "perpendicular!!!!\n");
          return 1;
        }else{
          return 0;
        }
      }else if (a == 1){
        if (my > 0.960 && my < 0.999)
        {
          fprintf(stderr, "perpendicular!!!!\n");
          return 1;
        }else{
          return 0;
        }
      }

}

void final_check(int count, double left_right){
  fprintf(stderr, "in final_check ########\n");
  fprintf(stderr, "count is  %d\n", count);

  if (count < 5){
    fprintf(stderr, "DDDDDDDD\n");
    drive_speed(30);
    usleep(100000);
  }


  else if ((count == 5) && (left_right)) {
    fprintf(stderr, "TTTTT_LLLLLLLLLLLLLLL\n");
    pivot_left_speed(15);
    usleep(100000);

  }
  
  else if ((count == 5) && !(left_right)) {
    printf(stderr, "TTTTT_RRRRRRRRRRRRRRRR\n");
    pivot_right_speed(15);
    usleep(100000);
  }

  else if (count > 5){
    printf(stderr, "R R R R R R R R R R\n");
    drive_speed(-30);
    usleep(100000);
  }
}


void makePerpendicular(struct RoboAI *ai, struct blob *blobs, int flag){
 struct blob *p;
 static double stepID=0;
 double frame_inc=1.0/50.0;

 if ((flag == 0)  && (mx > 0.0)){
        if (a == 1){
          // above case
         pivot_right_speed(15);
        }else{
         pivot_left_speed(15);  // Need a few frames to establish heading
        }
 }else if ((flag == 0)  && (mx < 0.0)){
  
        if (a == 1){
          // above case
          pivot_left_speed(15);
        }else{

          pivot_right_speed(15);
        }

 }else if (flag == 1){
  drive_speed(50);
 }else if (flag == -1){
  drive_speed(-50);
 }
 track_agents(ai,blobs);

 stepID+=frame_inc;
 if (stepID>=50&&ai->st.selfID==1)
 {
  stepID=0;
  all_stop();
 }
 else if (stepID>=1) stepID=0;
 return;
}

double angle_between(struct RoboAI *ai){
  //heading vectors(x,y) of robot while approaching the ball to kick.
  double rx, ry = 0.0;
  // vectors heading from robot's center to the ball

  double angle = 0.0;


  //updating rx,ry with current vectors of the bot.
  rx = ai->st.self->cx[0] - before_turn_x;
  ry = ai->st.self->cy[0] - before_turn_y;



  //magnitude of each vectors
  double mag_rxy = sqrt(pow(rx,2.0) + pow(ry,2.0));
  double mag_rbxy = sqrt(pow(perfect_line_x,2.0) + pow(perfect_line_y,2.0));

  angle = acos(((rx * perfect_line_x) + (ry * perfect_line_y)) / (mag_rxy * mag_rbxy));

  return angle * rad_to_degree;

}




int perfect_kick(struct RoboAI *ai){
  //heading vectors(x,y) of robot while approaching the ball to kick.
 double rx, ry = 0.0;
 // vectors heading from robot's center to the ball
 double rbx, rby = 0.0;
 double angle = 0.0;


 //updating rx,ry with current vectors of the bot.
 rx = ai->st.self->mx;
 ry = ai->st.self->my;

 rbx = (ai->st.ball->cx[0] - ai->st.self->cx[0]);
 rby = (ai->st.ball->cy[0] - ai->st.self->cy[0]); 

 if (((rbx * ry) - (rby * rx)) > 0.0) { 
   return 1; //turn right
   
 }
 return 0; //turn left
}

void center_y(struct RoboAI *ai, struct blob *blobs, int a){
  mx = 0.0;
  my = 0.0;
  if (a == 1){

          if (fabs(ai->st.ball->cy[0] - ai->st.self->cy[0]) < 20 ){
            all_stop();
            c++;
            if (c > 1){
              if ((ai->st.ball->cx[0] - ai->st.self->cx[0]) > 0){
                fprintf(stderr, "############# turn left #################\n");
                before_turn_x = ai->st.self->cx[0];
                before_turn_y = ai->st.self->cy[0];
                ai->st.self->mx = 0;
                ai->st.self->my = 0;

                perfect_line_x = ai->st.ball->cx[0] - ai->st.self->cx[0];
                perfect_line_y = ai->st.ball->cy[0] - ai->st.self->cy[0];
                pivot_left_speed(40);
                usleep(1250000);

                all_stop();

              
              

                ai->st.state = 161;
                mx = 0.0;
                my = 0.0;}
                else{
              fprintf(stderr, "############# turn right #################\n");
                before_turn_x = ai->st.self->cx[0];
                before_turn_y = ai->st.self->cy[0];
                ai->st.self->mx = 0;
                ai->st.self->my = 0;

                perfect_line_x = ai->st.ball->cx[0] - ai->st.self->cx[0];
                perfect_line_y = ai->st.ball->cy[0] - ai->st.self->cy[0];
                pivot_right_speed(40);
                usleep(1150000);
                all_stop();
                        
              

                ai->st.state = 161;
                mx = 0.0;
                my = 0.0;

                }
            }
      }else if (fabs(ai->st.ball->cy[0] - ai->st.self->cy[0]) >20){
        
        drive_speed(30);
        c = 0;
      }
    }

  else if (a == -1){
      


          if (fabs(ai->st.ball->cy[0] - ai->st.self->cy[0]) < 20 ){
            all_stop();
            c++;
            fprintf(stderr, "##### %f ######## %d ######\n",(ai->st.ball->cy[0] - ai->st.self->cy[0]),c);
            if (c > 1){
              if ((ai->st.ball->cx[0] - ai->st.self->cx[0]) < 0){
                fprintf(stderr, "############# turn left #################\n");
                before_turn_x = ai->st.self->cx[0];
                before_turn_y = ai->st.self->cy[0];
                ai->st.self->mx = 0;
                ai->st.self->my = 0;
                perfect_line_x = ai->st.ball->cx[0] - ai->st.self->cx[0];
                perfect_line_y = ai->st.ball->cy[0] - ai->st.self->cy[0];
                pivot_left_speed(40);
                usleep(1250000);
                all_stop();
    
            

                ai->st.state = 111;
                mx = 0.0;
                my = 0.0;
              }
            else{
                fprintf(stderr, "############# turn right #################\n");
                before_turn_x = ai->st.self->cx[0];
                before_turn_y = ai->st.self->cy[0];
                ai->st.self->mx = 0;
                ai->st.self->my = 0;
                perfect_line_x = ai->st.ball->cx[0] - ai->st.self->cx[0];
                perfect_line_y = ai->st.ball->cy[0] - ai->st.self->cy[0];
                pivot_right_speed(40);
                usleep(1150000);
                
                all_stop();
                ai->st.state = 111;
                mx = 0.0;
                my = 0.0;

                }
            }
          }else if (fabs(ai->st.ball->cy[0] - ai->st.self->cy[0]) >20){
            
            drive_speed(30);
            c = 0;
          }
  }else{
    fprintf(stderr, "Something is wrong\n");
  }
}


int side_checker(struct RoboAI *ai){

  if (ai->st.ball->cx[0] > ai->st.self->cx[0]){
    // left of camera
    return 1;
  }else{
    // right of camera
    return 0 ;
  }
}
//h