#include "leg.h"

void kakiJuara(){
  float t;
  int index = 0;

  // 🟢 Air phase (foot up and forward)
  for (t = 0; t <= 1; t += delta_t) {
    //this for I
    //x = bezierQuadratic(xZeroIJ, xOne, xTwoIJ, t);
    xI = bezierQuadratic(xZero, xOne, xTwo, t);
    yI = bezierQuadratic(yZero, yOne, yTwo, t);

    //this for J
    xJ = bezierQuadratic(xZero, xOne, xTwo, t);
    yJ = bezierQuadratic(yZero, yOne, yTwo, t);

    //this for K
    xK = bezierQuadratic(xTwo, xOne, xZero, t);
    yK = bezierQuadratic(yTwo, yOne, yZero, t);

    //this for L
    xL = bezierQuadratic(xTwo, xOne, xZero, t);
    yL = bezierQuadratic(yTwo, yOne, yZero, t);

    ServoAngles angles_I = IK_ThetaAngle(xI, yI);
    ServoAngles angles_J = IK_ThetaAngle(xJ, yJ);
    ServoAngles angles_K = IK_ThetaAngle(xK, yK);
    ServoAngles angles_L = IK_ThetaAngle180(xL, yL); 

    float I_LA = angles_I.left;
    float I_RA = angles_I.right;

    float J_LA = angles_J.left;
    float J_RA = angles_J.right;

    float K_LA = angles_K.left;
    float K_RA = angles_K.right;

    float L_LA = angles_L.left;
    float L_RA = angles_L.right;

    index++;
    
    SKL.write(K_LA);
    SKR.write(K_RA);

    SIL.write(I_LA);
    SIR.write(I_RA);

    SJL.write(J_LA);
    SJR.write(J_RA);

    SLL.write(L_LA);
    SLR.write(L_RA);

    IK_ThetaAngle(xI, yI);
    IK_ThetaAngle(xJ, yJ);
    IK_ThetaAngle(xK, yK);
    IK_ThetaAngle180(xL, yL);
  }

  // 🔵 Ground phase (foot back and flat)
  for (t = 0; t <= 1; t += delta_t) {

    //this for i
    // x = xTwoIJ - t * (step_length + IJOFFSET);
    xI = xZero + t * step_length;
    yI = ground_offset;


    //tbis for k
    xK = xZero + t * step_length;
    yK = ground_offset;


    //this for L
    xL = xZero + t * step_length;
    yL = ground_offset;

    ServoAngles angles_I = IK_ThetaAngle(xI, yI);
    ServoAngles angles_J = IK_ThetaAngle(xJ, yJ);
    ServoAngles angles_K = IK_ThetaAngle(xK, yK);
    ServoAngles angles_L = IK_ThetaAngle180(xL, yL);  

    float I_LA = angles_I.left;
    float I_RA = angles_I.right;

    float J_LA = angles_J.left;
    float J_RA = angles_J.right;

    float K_LA = angles_K.left;
    float K_RA = angles_K.right;

    float L_LA = angles_L.left;
    float L_RA = angles_L.right;
   
    index++;

    IK_ThetaAngle(xI, yI);
    IK_ThetaAngle(xJ, yJ);
    IK_ThetaAngle(xK, yK);
    IK_ThetaAngle180(xL, yL);
  }
}
