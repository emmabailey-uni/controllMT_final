// Change direction of the motor based on input 
void changeDIR(float RightCMD, float LeftCMD){
    if (RightCMD < 0){
      digitalWrite(DIRR,HIGH);
    }
    else{
      digitalWrite(DIRR,LOW);
    }
    if (LeftCMD < 0){
      digitalWrite(DIRL,HIGH);
    }
    else{
      digitalWrite(DIRL,LOW);
    }

  }

// Model based controller - Bogdan [step response 0.2 sec, variables calculated using matlab]
void ModelController(float Vd, float Wd, float w_l_real, float w_r_real){
  
    // Declair variables [r,u,y,r1,u1,y1]
    float refr, refl;
    float inputr, inputl;
    float outputr, outputl;
    static float refr_1, refl_1;
    static float inputr_1, inputl_1;
    static float outputr_1, outputl_1;

    // Init static variables
    if(ctr_Flag == 0){
      refr_1 = 0;
      refl_1 = 0;
      inputr_1 = 0;
      inputl_1 = 0;
      outputr_1 = 0;
      outputl_1 = 0;
      
      ctr_Flag = 1;
    }

    // 1: Calc ref and update current output [r,y]
    refl = (Vd - (b*Wd)/2.0)/r; 
    refr = (Vd + (b*Wd)/2.0)/r;
    outputr = w_r_real;
    outputl = w_l_real;

    // 2: Calc input [u]
    inputr = inputr_1 + a_*refr + b_*refr_1 - c_*outputr - d_*outputr_1;
    inputl = inputl_1 + a_*refl + b_*refl_1 - c_*outputl - d_*outputl_1;

    // 3: Write output
    
    changeDIR(inputr,inputl);

    analogWrite(PWMR,fabs(inputr));
    analogWrite(PWML,fabs(inputl));

    // 4: Update [r1,u1,y1]
    refr_1 = refr;
    refl_1 = refl;
    inputr_1 = inputr;
    inputl_1 = inputl;
    outputr_1 = outputr;
    outputl_1 = outputl;
  }
