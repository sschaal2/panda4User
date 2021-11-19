/*============================================================================
==============================================================================
                      
                              initUserGraphics.c
 
==============================================================================
Remarks:

         Functions needed for user graphics
         simulation

============================================================================*/

#include "SL.h"
#include "SL_user.h"
#include "SL_man.h"
#include "SL_common.h"
#include "string.h"

// openGL includes
#include "GL/freeglut.h"
#include "SL_openGL.h"
#include "SL_userGraphics.h"
#include "utility_macros.h"

// global variables

// local variables
static void displayFTVector(void *b);
static void drawPoseCoordinateFrame(void *b);

/*****************************************************************************
******************************************************************************
Function Name	: initUserGraphics
Date		: June 1999
   
Remarks:

      allows adding new graphics functions to openGL interface

******************************************************************************
Paramters:  (i/o = input/output)

  none   

*****************************************************************************/
int
initUserGraphics(void)

{

  switchCometDisplay(TRUE,500);

  addToUserGraphics("displayFTVector","Displays a F/T vector at pos x",displayFTVector,
                    3*N_CART*sizeof(float));
  addToUserGraphics("drawPoseFrame","Displays coordinate frame of pose",drawPoseCoordinateFrame,
                    (N_CART+N_QUAT+1)*sizeof(float));

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  displayFTVector
 \date  April. 2012

 \remarks

 at a given world position, displays a Force/Torque vector (also in world
 coordinates)

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]      b      : the general array of bytes

 ******************************************************************************/
static void
displayFTVector(void *b)
{
  int i,j,n;
  double f[N_CART+1];
  double t[N_CART+1];
  double x[N_CART+1];  
  double fscale_N=0.025;
  double fscale_Nm=0.15;
  double s[N_CART+1];
  double e[N_CART+1];
  double arrow_width = 0.01;
  float  data[3*N_CART+1];

  // assign the ball position from b array
  memcpy(&(data[1]),b,3*N_CART*sizeof(float));

  // rotate the force torque info to world coordinates
  for (i=1; i<=N_CART; ++i) {
    f[i] = data[i];
    t[i] = data[i+N_CART];
    x[i] = data[i+2*N_CART];
  }

  glPushMatrix();
    
  // move to the force torque sensor
  glTranslated(x[_X_],x[_Y_],x[_Z_]);

  // the start and end point of the force vector
  s[_X_] =  0.0;
  s[_Y_] =  0.0;
  s[_Z_] =  0.0;

  e[_X_] = s[_X_] + f[_X_]*fscale_N;
  e[_Y_] = s[_Y_] + f[_Y_]*fscale_N;
  e[_Z_] = s[_Z_] + f[_Z_]*fscale_N;


  glColor4f (0.8,1.0,1.0,1.0);      
  drawArrow(s,e,arrow_width);

  e[_X_] = s[_X_] + t[_X_]*fscale_Nm;
  e[_Y_] = s[_Y_] + t[_Y_]*fscale_Nm;
  e[_Z_] = s[_Z_] + t[_Z_]*fscale_Nm;

    
  glColor4f (0.8,0.4,0.0,1.0);
  drawArrow(s,e,arrow_width);

  glPopMatrix();

}

/*!*****************************************************************************
*******************************************************************************
\note  drawPoseCoordinateFrame
\date  March 2013
   
\remarks 

draws the local coordinate specified by a pose vector [position, quaternion]

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]      b      : the general array of bytes

******************************************************************************/
static void
drawPoseCoordinateFrame(void *b)
{

  int     i,j;
  float   data[N_CART+N_QUAT+1+1];  
  double  v[N_CART+1+1];
  double  r[N_CART+1+1];
  double  s[N_CART+1+1];
  double  arrow_width = 0.005;
  double  length;
  double  x[N_CART+1];
  SL_quat q;
  MY_MATRIX(A,1,N_CART+1,1,N_CART+1);
  

  // assign the pose coordinates to local variables
  memcpy(&(data[1]),b,(1+N_CART+N_QUAT)*sizeof(float));
  length  = data[1];
  x[_X_]  = data[2];
  x[_Y_]  = data[3];
  x[_Z_]  = data[4];
  q.q[_Q0_] = data[5];
  q.q[_Q1_] = data[6];
  q.q[_Q2_] = data[7];
  q.q[_Q3_] = data[8];

  // need the rotation matrix from quaternion: will be in first 3x3 of homogenious
  // transformation matrix
  quatToRotMat(&q,A);
  A[1][4] = x[_X_];
  A[2][4] = x[_Y_];
  A[3][4] = x[_Z_];
  A[4][4] = 1.0;
  
  // draw the coordinate systems
  glPushMatrix();
  glLineWidth(2.0);

  v[_X_] = length;
  v[_Y_] = v[_Z_] = 0.0;
  v[_Z_+1] = 1.0;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  for (i=1; i<=N_CART; ++i)
    s[i] = A[i][4];

  glColor4f (1.0,0.0,0.0,0.0);
  drawArrow(s,r,arrow_width);

  v[_X_] = length+0.1;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  glRasterPos3f(r[_X_],r[_Y_],r[_Z_]);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,'X');  

  v[_Y_] = length;
  v[_X_] = v[_Z_] = 0.0;
  v[_Z_+1] = 1.0;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  for (i=1; i<=N_CART; ++i)
    s[i] = A[i][4];

  glColor4f (0.0,1.0,0.0,0.0);
  drawArrow(s,r,arrow_width);
  
  v[_Y_] = length+0.1;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  glRasterPos3f(r[_X_],r[_Y_],r[_Z_]);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,'Y');  
  
  v[_Z_] = length;
  v[_Y_] = v[_X_] = 0.0;
  v[_Z_+1] = 1.0;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  for (i=1; i<=N_CART; ++i)
    s[i] = A[i][4];

  glColor4f (0.0,0.0,1.0,0.0);
  drawArrow(s,r,arrow_width);

  v[_Z_] = length+0.1;
  mat_vec_mult_size(A,N_CART+1,N_CART+1,v,N_CART+1,r);
  glRasterPos3f(r[_X_],r[_Y_],r[_Z_]);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,'Z');
  
  glLineWidth(1.0);
  glPopMatrix();

}

