// Read sensors and update sensor values
float readSensors(float* left, float* mid, float* right){
  int i;
  float sumr=0, suml=0, summ=0;
  float left_[10];
  float right_[10];
  float mid_[10];

  for (i=0; i<10; i++){
    float valR = analogRead(sensorpinR);
    right_[i] = 57.33*(pow(valR, -0.9998)) - 0.0282;
    sumr = sumr + right_[i];
 
    
    float valM = analogRead(sensorpinM);
    mid_[i] = 265.5*(pow(valM, -1.305)) + 0.01485;
    summ = summ + mid_[i];
    
    float valL = analogRead(sensorpinL);
    left_[i] = 198.9*(pow(valL, -1.265)) + 0.01932;
    suml = suml + left_[i];
  }

  *left = suml/10;
  *mid = summ/10;
  *right = sumr/10;
}
