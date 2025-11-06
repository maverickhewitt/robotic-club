void walkPatternLow() {
  float t;

  for (t = 0; t <= 1; t += delta_tfast) {
    //Drag (ground phase) for SIL/SIR and SKL/SKR
    float xDrag = xTwoIJLOW - t * (step_length + IJOFFSET2);
    float yDrag = ground_offset + GROUNDOFF;

    float xOpDrag = xZero + t * step_length;
    float yOpDrag = ground_offset + GROUNDOFF;

    ServoAngles angle_i = IK_ThetaAngle(xDrag, yDrag);
    ServoAngles angle_k = IK_ThetaAngle(xOpDrag, yOpDrag);

    SIL.write(angle_i.left + err[0][0]);
    SIR.write(angle_i.right + err[0][1]);
    SKL.write(angle_k.left + err[2][0]);
    SKR.write(angle_k.right + err[2][1]);

    // --- Swing (forward) for SJL/SJR and SLL/SLR ---
    float xSwing = bezierQuadratic(xZeroIJLOW, xOne, xTwoIJLOW, t);
    float ySwing = bezierQuadratic(yZeroLow, yOneLow, yTwoLow, t);

    float xOpSwing = bezierQuadratic(xTwo, xOne, xZero, t);
    float yOpSwing = bezierQuadratic(yTwoLow, yOneLow, yZeroLow, t);

    ServoAngles angle_j = IK_ThetaAngle(xSwing, ySwing);
    ServoAngles angle_l = IK_ThetaAngle180(xOpSwing, yOpSwing);

    SJL.write(angle_j.left + err[1][0]);
    SJR.write(angle_j.right + err[1][1]);
    SLL.write(angle_l.left + err[3][0]);
    SLR.write(angle_l.right + err[3][1]);

    delay(delayStep);
  }

  for (t = 0; t <= 1; t += delta_tfast) {
    //Swing (forward) for SIL/SIR and SKL/SKR ---
    float xSwing = bezierQuadratic(xZeroIJLOW, xOne, xTwoIJLOW, t);
    float ySwing = bezierQuadratic(yZeroLow, yOneLow, yTwoLow, t);

    float xOpSwing = bezierQuadratic(xTwo, xOne, xZero, t);
    float yOpSwing = bezierQuadratic(yTwoLow, yOneLow, yZeroLow, t);

    ServoAngles angle_i = IK_ThetaAngle(xSwing, ySwing);
    ServoAngles angle_k = IK_ThetaAngle(xOpSwing, yOpSwing);

    SIL.write(angle_i.left + err[0][0]);
    SIR.write(angle_i.right + err[0][1]);
    SKL.write(angle_k.left + err[2][0]);
    SKR.write(angle_k.right + err[2][1]);

    //Drag (ground phase) for SJL/SJR and SLL/SLR 
    float xDrag = xTwoIJLOW - t * (step_length + IJOFFSET2);
    float yDrag = ground_offset + GROUNDOFF;

    float xOpDrag = xZero + t * step_length;
    float yOpDrag = ground_offset + GROUNDOFF;

    ServoAngles angle_j = IK_ThetaAngle(xDrag, yDrag);
    ServoAngles angle_l = IK_ThetaAngle180(xOpDrag, yOpDrag);

    SJL.write(angle_j.left + err[1][0]);
    SJR.write(angle_j.right + err[1][1]);
    SLL.write(angle_l.left + err[3][0]);
    SLR.write(angle_l.right + err[3][1]);

    delay(delayStep);
  }
}

void walkPatternGood() {
  float t;

  for (t = 0; t <= 1; t += delta_t) {
    //Drag (ground phase) for SIL/SIR and SKL/SKR
    float xDrag = xTwoIJ - t * (step_length + IJOFFSET);
    float yDrag = ground_offset;

    float xOpDrag = xZero + t * step_length;
    float yOpDrag = ground_offset;

    ServoAngles angle_i = IK_ThetaAngle(xDrag, yDrag);
    ServoAngles angle_k = IK_ThetaAngle(xOpDrag, yOpDrag);

    SIL.write(angle_i.left + err[0][0]);
    SIR.write(angle_i.right + err[0][1]);
    SKL.write(angle_k.left + err[2][0]);
    SKR.write(angle_k.right + err[2][1]);

    // --- Swing (forward) for SJL/SJR and SLL/SLR ---
    float xSwing = bezierQuadratic(xZeroIJ, xOne, xTwoIJ, t);
    float ySwing = bezierQuadratic(yZero, yOne, yTwo, t);

    float xOpSwing = bezierQuadratic(xTwo, xOne, xZero, t);
    float yOpSwing = bezierQuadratic(yTwo, yOne, yZero, t);

    ServoAngles angle_j = IK_ThetaAngle(xSwing, ySwing);
    ServoAngles angle_l = IK_ThetaAngle180(xOpSwing, yOpSwing);

    SJL.write(angle_j.left + err[1][0]);
    SJR.write(angle_j.right + err[1][1]);
    SLL.write(angle_l.left + err[3][0]);
    SLR.write(angle_l.right + err[3][1]);

    delay(delayStep);
  }

  for (t = 0; t <= 1; t += delta_t) {
    //Swing (forward) for SIL/SIR and SKL/SKR ---
    float xSwing = bezierQuadratic(xZeroIJ, xOne, xTwoIJ, t);
    float ySwing = bezierQuadratic(yZero, yOne, yTwo, t);

    float xOpSwing = bezierQuadratic(xTwo, xOne, xZero, t);
    float yOpSwing = bezierQuadratic(yTwo, yOne, yZero, t);

    ServoAngles angle_i = IK_ThetaAngle(xSwing, ySwing);
    ServoAngles angle_k = IK_ThetaAngle(xOpSwing, yOpSwing);

    SIL.write(angle_i.left + err[0][0]);
    SIR.write(angle_i.right + err[0][1]);
    SKL.write(angle_k.left + err[2][0]);
    SKR.write(angle_k.right + err[2][1]);

    //Drag (ground phase) for SJL/SJR and SLL/SLR 
    float xDrag = xTwoIJ - t * (step_length + IJOFFSET);
    float yDrag = ground_offset;

    float xOpDrag = xZero + t * step_length;
    float yOpDrag = ground_offset;

    ServoAngles angle_j = IK_ThetaAngle(xDrag, yDrag);
    ServoAngles angle_l = IK_ThetaAngle180(xOpDrag, yOpDrag);

    SJL.write(angle_j.left + err[1][0]);
    SJR.write(angle_j.right + err[1][1]);
    SLL.write(angle_l.left + err[3][0]);
    SLR.write(angle_l.right + err[3][1]);

    delay(delayStep);
  }
}

void walkPattern() {
  float t;

  for (t = 0; t <= 1; t += delta_tf) {
    //Swing (forward) for SIL/SIR and SKL/SKR ---
    float xSwing = bezierQuadratic(xZeroIJ, xOne, xTwoIJ, t);
    float ySwing = bezierQuadratic(yZeroIK, yOneIK, yTwoIK, t);

    float xOpSwing = bezierQuadratic(xTwo, xOne, xZero, t);
    float yOpSwing = bezierQuadratic(yTwoIK, yOneIK, yZeroIK, t);

    ServoAngles angle_i = IK_ThetaAngle(xSwing, ySwing);
    ServoAngles angle_k = IK_ThetaAngle(xOpSwing, yOpSwing);

    SIL.write(angle_i.left + err[0][0]);
    SIR.write(angle_i.right + err[0][1]);
    SKL.write(angle_k.left + err[2][0]);
    SKR.write(angle_k.right + err[2][1]);

    //Drag (ground phase) for SJL/SJR and SLL/SLR 
    float xDrag = xTwoIJ - t * (step_length + IJOFFSET);
    float yDrag = ground_offset;

    float xOpDrag = xZero + t * step_length;
    float yOpDrag = ground_offset;

    ServoAngles angle_j = IK_ThetaAngle(xDrag, yDrag);
    ServoAngles angle_l = IK_ThetaAngle180(xOpDrag, yOpDrag);

    SJL.write(angle_j.left + err[1][0]);
    SJR.write(angle_j.right + err[1][1]);
    SLL.write(angle_l.left + err[3][0]);
    SLR.write(angle_l.right + err[3][1]);

    delay(delayStep);
  }

  for (t = 0; t <= 1; t += delta_tb) {
    //Drag (ground phase) for SIL/SIR and SKL/SKR
    float xDrag = xTwoIJ - t * (step_length + IJOFFSET);
    float yDrag = ground_offset + IKHEIGHTOFFSET;

    float xOpDrag = xZero + t * step_length;
    float yOpDrag = ground_offset + IKHEIGHTOFFSET;

    ServoAngles angle_i = IK_ThetaAngle(xDrag, yDrag);
    ServoAngles angle_k = IK_ThetaAngle(xOpDrag, yOpDrag);

    SIL.write(angle_i.left + err[0][0]);
    SIR.write(angle_i.right + err[0][1]);
    SKL.write(angle_k.left + err[2][0]);
    SKR.write(angle_k.right + err[2][1]);

    // --- Swing (forward) for SJL/SJR and SLL/SLR ---
    float xSwing = bezierQuadratic(xZeroIJ, xOne, xTwoIJ, t);
    float ySwing = bezierQuadratic(yZero, yOne, yTwo, t);

    float xOpSwing = bezierQuadratic(xTwo, xOne, xZero, t);
    float yOpSwing = bezierQuadratic(yTwo, yOne, yZero, t);

    ServoAngles angle_j = IK_ThetaAngle(xSwing, ySwing);
    ServoAngles angle_l = IK_ThetaAngle180(xOpSwing, yOpSwing);

    SJL.write(angle_j.left + err[1][0]);
    SJR.write(angle_j.right + err[1][1]);
    SLL.write(angle_l.left + err[3][0]);
    SLR.write(angle_l.right + err[3][1]);

    delay(delayStep);
  }
}

void leg_l_test() {

  float t;
  int index = 0;

  // 🟢 Air phase (foot up and forward)
  for (t = 0; t <= 1; t += delta_t) {
    x = bezierQuadratic(xTwo, xOne, xZero, t);
    y = bezierQuadratic(yTwo, yOne, yZero, t);

    ServoAngles angles = IK_ThetaAngle180(x, y); 
    float LeftServoAngle = angles.left;
    float RightServoAngle = angles.right;

    SLL.write(LeftServoAngle + err[3][0]);
    SLR.write(RightServoAngle + err[3][1]);

    IK_ThetaAngle180(x, y);

    delay(40);
  }

  // 🔵 Ground phase (foot back and flat)
  for (t = 0; t <= 1; t += delta_t) {
    x = xZero + t * step_length;
    y = ground_offset;

    Serial.print("Ground phase - x: ");
    Serial.print(x);
    Serial.print(", y: ");
    Serial.println(y);
    ServoAngles angles = IK_ThetaAngle180(x, y);
    float LeftServoAngle = angles.left;
    float RightServoAngle = angles.right;

    SLL.write(LeftServoAngle + err[3][0]);
    SLR.write(RightServoAngle + err[3][1]);

    IK_ThetaAngle180(x, y);

    delay(60);
  }
}

void leg_k_test() {
  float t;
  int index = 0;

  // 🟢 Air phase (foot up and forward)
  for (t = 0; t <= 1; t += delta_t) {
    x = bezierQuadratic(xTwo, xOne, xZero, t);
    y = bezierQuadratic(yTwo, yOne, yZero, t);

    ServoAngles angles = IK_ThetaAngle(x, y);
    float LeftServoAngle = angles.left;
    float RightServoAngle = angles.right;

    SKL.write(LeftServoAngle + err[2][0]);
    SKR.write(RightServoAngle + err[2][1]);

    IK_ThetaAngle(x, y);
    delay(30);
    
  }

  // 🔵 Ground phase (foot back and flat)
  for (t = 0; t <= 1; t += delta_t) {
    x = xZero + t * step_length;
    y = ground_offset;

    ServoAngles angles = IK_ThetaAngle(x, y);
    float LeftServoAngle = angles.left;
    float RightServoAngle = angles.right;

    SKL.write(LeftServoAngle + err[2][0]);
    SKR.write(RightServoAngle + err[2][1]);

    IK_ThetaAngle(x, y);

    delay(100);
  }
}

void leg_i_test() { 
  float t;

  // 🟢 Air phase (foot up and forward)
  for (t = 0; t <= 1; t += delta_t) {
    x = bezierQuadratic(xZero, xOne, xTwo, t);
    y = bezierQuadratic(yZero, yOne, yTwo, t);

    ServoAngles angles = IK_ThetaAngle(x, y);

    float LeftServoAngle = angles.left;
    float RightServoAngle = angles.right;

    SIL.write(LeftServoAngle + err[0][0]);
    SIR.write(RightServoAngle + err[0][1]);

    SJL.write(LeftServoAngle + err[1][0]);
    SJR.write(RightServoAngle + err[1][0]);

    IK_ThetaAngle(x, y);

    delay(40);
  }

  // 🔵 Ground phase (foot back and flat)
  for (t = 0; t <= 1; t += delta_t) {
    x = xTwo - t * (step_length);
    y = ground_offset;

    ServoAngles angles = IK_ThetaAngle(x, y);
    float LeftServoAngle = angles.left;
    float RightServoAngle = angles.right;

    SIL.write(LeftServoAngle + err[0][0]);
    SIR.write(RightServoAngle + err[0][1]);

    SJL.write(LeftServoAngle + err[1][0]);
    SJR.write(RightServoAngle + err[1][1]);

    IK_ThetaAngle(x, y);

    delay(60);
  }
}

void kakiJuara(){
  float t;
  int index = 0;

  // 🟢 Air phase (foot up and forward)
  for (t = 0; t <= 1; t += delta_tf) {
    //this for I
    //x = bezierQuadratic(xZeroIJ, xOne, xTwoIJ, t);
    xI = bezierQuadratic(xZero, xOne, xTwo, t);
    yI = bezierQuadratic(yZero, yOne, yTwo, t);

    //this for J
    xJ = bezierQuadratic(xZero, xOne, xTwo, t);
    yJ = bezierQuadratic(yZero, yOne, yTwo, t);

    // //this for K
    xK = bezierQuadratic(xTwo, xOne, xZero, t);
    yK = bezierQuadratic(yTwo, yOne, yZero, t);

    // //this for L
    xL = bezierQuadratic(xTwo, xOne, xOne, t);
    yL = bezierQuadratic(yTwo, yOne, yOne, t);

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
    delay(15);
  }

  // 🔵 Ground phase (foot back and flat)
  for (float t = 1; t >= 0; t -= delta_tb) {
    xI = xZero + t * step_length;
    yI = ground_offset;

    xJ = xZero + t * step_length;
    yJ = ground_offset;

    float tMirror = 1.0 - t; 

    xK = xZero + tMirror * step_length;
    yK = ground_offset;

    xL = xZero + tMirror * step_length;
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

    SKL.write(K_LA);
    SKR.write(K_RA);

    SIL.write(I_LA);
    SIR.write(I_RA);

    SJL.write(J_LA);
    SJR.write(J_RA);

    SLL.write(L_LA);
    SLR.write(L_RA);

    delay(15);
  }
}

void walkCycle(){
  float t;
  int index = 0;

  // 🟢 Air phase (foot up and forward)
  for (t = 0; t <= 1; t += delta_tf) {
    //this for I
    //x = bezierQuadratic(xZeroIJ, xOne, xTwoIJ, t);
    xI = bezierQuadratic(xZero, xOne, xTwo, t);
    yI = bezierQuadratic(yZero, yOne, yTwo, t);

    //this for J
    xJ = bezierQuadratic(xZero, xOne, xTwo, t);
    yJ = bezierQuadratic(yZero, yOne, yTwo, t);

    // //this for K
    xK = bezierQuadratic(xTwo, xOne, xZero, t);
    yK = bezierQuadratic(yTwo, yOne, yZero, t);

    // //this for L
    xL = bezierQuadratic(xTwo, xOne, xOne, t);
    yL = bezierQuadratic(yTwo, yOne, yOne, t);

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
    delay(25);
  }

  // 🔵 Ground phase (foot back and flat)
  for (float t = 1, t2 = 0; t >= 0 && t2 <= 1; t -= delta_tb, t2 += delta_tb) {
    xI = xZero + t * step_length;
    yI = ground_offset;

    xJ = xZero + t * step_length;
    yJ = ground_offset;

    xK = xZero + t2 * step_length;
    yK = ground_offset;

    xL = xZero + t2 * step_length;
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

    delay(25);
  }
}



void jalan() {
  float t;

  // ======== 🟢 PHASE 1: I & K swing forward, J & L drag backward ========
  for (t = 0; t <= 1; t += delta_tf) {
    // ---- Swing Phase (I & K) ----
    xI = bezierQuadratic(xZero, xOne, xTwo, t);
    yI = bezierQuadratic(yZero, yOne, yTwo, t);

    xK = bezierQuadratic(xTwo, xOne, xZero, t);
    yK = bezierQuadratic(yTwo, yOne, yZero, t);

    // ---- Drag Phase (J & L) ----
    float tMirror = 1.0 - t; 
    xJ = xZero + t * step_length;
    yJ = ground_offset;

    xL = xZero + tMirror * step_length;
    yL = ground_offset;

    ServoAngles angles_I = IK_ThetaAngle(xI, yI);
    ServoAngles angles_K = IK_ThetaAngle(xK, yK);
    ServoAngles angles_J = IK_ThetaAngle(xJ, yJ);
    ServoAngles angles_L = IK_ThetaAngle180(xL, yL);

    SIL.write(angles_I.left);  SIR.write(angles_I.right);
    SKL.write(angles_K.left);  SKR.write(angles_K.right);
    SJL.write(angles_J.left);  SJR.write(angles_J.right);
    SLL.write(angles_L.left);  SLR.write(angles_L.right);

    delay(30);
  }

  // ======== 🔵 PHASE 2: J & L swing forward, I & K drag backward ========
  for (t = 0; t <= 1; t += delta_tf) {
    // ---- Swing Phase (J & L) ----
    xJ = bezierQuadratic(xZero, xOne, xTwo, t);
    yJ = bezierQuadratic(yZero, yOne, yTwo, t);

    xL = bezierQuadratic(xTwo, xOne, xZero, t);
    yL = bezierQuadratic(yTwo, yOne, yZero, t);

    // ---- Drag Phase (I & K) ----
    float tMirror = 1.0 - t; 
    xI = xZero  * step_length;
    yI = ground_offset;

    xK = xZero + tMirror * step_length;
    yK = ground_offset;

    // ---- Compute angles ----
    ServoAngles angles_I = IK_ThetaAngle(xI, yI);
    ServoAngles angles_K = IK_ThetaAngle(xK, yK);
    ServoAngles angles_J = IK_ThetaAngle(xJ, yJ);
    ServoAngles angles_L = IK_ThetaAngle180(xL, yL);

    // ---- Write to servos ----
    SIL.write(angles_I.left);  SIR.write(angles_I.right);
    SKL.write(angles_K.left);  SKR.write(angles_K.right);
    SJL.write(angles_J.left);  SJR.write(angles_J.right);
    SLL.write(angles_L.left);  SLR.write(angles_L.right);

    delay(15);
  }
}
