/* This file is part of the Razor AHRS Firmware */

// Y accelerations array 
const int numYaccels = 4;
float y_accels[numYaccels];
int indexY = 0;
int yZeros = 0;

boolean y_accels_Zero = false;
boolean y_accels_One = false;
boolean y_accels_Two = false;
boolean y_accels_Three = false;



float S = 0.00390625;
float time = 0.02;

float initialSpeedX = 0;
float finalSpeedX = 0;

float initialSpeedY = 0;
float finalSpeedY = 0;

float initialSpeedZ = 0;
float finalSpeedZ = 0;


float finalPosX, initialPosX;
float finalPosY, initialPosY;
float finalPosZ, initialPosZ;


float initial_Y_accel = 0;
float final_Y_accel = 0;
boolean set_initial_Y_toZero = false;
boolean set_final_Y_toZero = false;

float q[4],  rx, ry, rz, gx, gy, gz, cx, cy, cz, qxc[4], qconj[4], trueacc[4];


 float qprev_w = 0;
   float qprev_x = 0;
    float qprev_y = 0;
     float qprev_z = 0;
     
     
    float _yaw,_pitch, _roll;

  float xgravityTest;


// Output angles: yaw, pitch, roll
void output_angles()
{
  
//  Serial.print("#A-");  Serial.print('=');
//  Serial.print(accel[0]); Serial.print(",");
//  Serial.print(accel[1]); Serial.print(",");
//  Serial.print(accel[2]); Serial.println();

  

//    Serial.print("#YPR=");
//    Serial.print(TO_DEG(yaw)); Serial.print(",");
//    Serial.print(TO_DEG(pitch)); Serial.print(",");
//    Serial.print(TO_DEG(roll)); Serial.println();
    
    //char raw_or_calibrated;
    


  
  
  
  
   
   
//   Serial.print(qw); Serial.print(",");
//  Serial.print(qx); Serial.print(",");
//  Serial.print(qy); Serial.print(",");
//  Serial.print(qz); Serial.println();



   rx = accel[0] * S;
  ry = accel[1] * S;
  rz = accel[2] * S;
  
  

  compensate_sensor_errors();
  
//  float c1 = cos(roll/2);
//   float s1 = sin(roll/2);
//   float c2 = cos(pitch/2);
//   float s2 = sin(pitch/2);
//   float c3 = cos(yaw/2);
//   float s3 = sin(yaw/2);
      // bank = roll
      // heading = yaw
      // attitude = pitch
     float c1 = cos(yaw/2);
     float c2 = cos(pitch/2);
     float c3 = cos(roll/2);
     float s1 = sin(yaw/2);
     float s2 = sin(pitch/2);
     float s3 = sin(roll/2);


 q[0] = c1*c2*c3 - s1*s2*s3; // w
 q[1] = s1*s2*c3 + c1*c2*s3; // x
 q[2] = s1*c2*c3 + c1*s2*s3; // y
 q[3] = c1*s2*c3 - s1*c2*s3; // z
    
 _yaw = atan2(2*q[2]*q[0]-2*q[1]*q[3], 1-2*(q[2]*q[2]) - 2*(q[3]*q[3]));
 _pitch = asin(2*q[1]*q[2] + 2*q[3]*q[0]);
 _roll = atan2(2*q[1]*q[0] - 2*q[2]*q[3], 1 - 2*(q[1]*q[1]) - 2*(q[3]*q[3]));
 
//    Serial.print("calculated yaw");
//    Serial.print(TO_DEG(_yaw));
//   Serial.print("calculated pitch"); 
//    Serial.print(TO_DEG(_pitch));
//    Serial.print("calculated roll"); 
//    Serial.print(TO_DEG(_roll));
//    Serial.println();
    
  
//    Serial.print("#RAW-"); Serial.print('=');
//  Serial.print(rx); Serial.print(",");
//  Serial.print(ry); Serial.print(",");
//  Serial.print(rz); Serial.println();
  
//  Serial.print("#Q-");  Serial.print('=');
//  Serial.print(q[0]); Serial.print(",");
//  Serial.print(q[1]); Serial.print(",");
//  Serial.print(q[2]); Serial.print(",");
//  Serial.print(q[3]); Serial.println();


 //gravity
  gx = 2 * (q[1] * q[3] - q[0] * q[2]);
  gy = 2 * (q[0] * q[1] + q[2] * q[3]);
  gz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
  
  
//  Serial.print("#G-");  Serial.print('=');
//  Serial.print(gx); Serial.print(",");
//  Serial.print(gy); Serial.print(",");
//  Serial.print(gz); Serial.println();
  
  
//  xgravityTest = rx * sin(pitch);
//  Serial.print("test");
//  Serial.print(xgravityTest); Serial.println();

  //dymanic acceleration: accel values - gravity
//  cx = gx - rx;
//  cy = gy - ry;
//  cz = gz - rz;
  cy = ry;
  cx = rx;
 
  
//   Serial.print("#A-");  Serial.print('=');
//  Serial.print(cx); Serial.print(",");
//  Serial.print(cy); Serial.print(",");
//  Serial.print(cz); Serial.println();


  // Rotate the acceleration ---------------------------------------------------------------------------------------------------
   //double integration
  //find the quaternion product of the previous quaternion vector and the current dynamic acceleration vector
//  qxc[0] = (qprev_w*0) - (qprev_x*cx) - (qprev_y*cy) - (qprev_z*cz);
//  qxc[1] = (qprev_w*cx) + (qprev_x*0) + (qprev_y*cz) - (qprev_z*cy);
//  qxc[2] = (qprev_w*cy) - (qprev_x*cz) + (qprev_y*0) + (qprev_z*cx);
//  qxc[3] = (qprev_w*cz) + (qprev_x*cy) - (qprev_y*cx) + (qprev_z*0);
//
//  //conjugate the previous quaternion vector
//  qconj[0] = qprev_w;
//  qconj[1] = -1*qprev_x;
//  qconj[2] = -1*qprev_y;
//  qconj[3] = -1*qprev_z;
//
//  //find the quaternion product of qxc and the conjugate of the previous quaternion vector
//  trueacc[0] = (qxc[0]*qconj[0]) - (qxc[1]*qconj[1]) - (qxc[2]*qconj[2]) - (qxc[3]*qconj[3]);
//  trueacc[1] = (qxc[0]*qconj[1]) + (qxc[1]*qconj[0]) + (qxc[2]*qconj[3]) - (qxc[3]*qconj[2]);
//  trueacc[2] = (qxc[0]*qconj[2]) - (qxc[1]*qconj[3]) + (qxc[2]*qconj[0]) + (qxc[3]*qconj[1]);
//  trueacc[3] = (qxc[0]*qconj[3]) + (qxc[1]*qconj[2]) - (qxc[2]*qconj[1]) + (qxc[3]*qconj[0]);
//
//  //save the current quaternions in the previous quaternion area for the next calculation
//  qprev_w = q[0];
//  qprev_x = q[1];
//  qprev_y = q[2];
//  qprev_z = q[3];
  
  
  
  // Start calculating speed 
  
  // X speed 
  if (finalSpeedX * cx < 0) {
//      Serial.print("<---->");
//      Serial.print("\t");
      cx = cx;
    } else if (cx <= 0 && cx >= -0.4 || cx >= 0 && cx <= 0.04){
//      Serial.print("Set to 0");
//      Serial.println();
      cx= 0;
    }
    
    finalSpeedX = initialSpeedX + (cx*9.8)*time;
    
    // Calculate distance
    finalPosX = initialPosX + initialSpeedX * time + 0.5 * cx * time*time;
    
    
    initialSpeedX = finalSpeedX;
    initialPosX = finalPosX;
    
//    Serial.print("#acceleration");  Serial.print('=');
//      Serial.print(cx);
//      Serial.println();
//    
//    Serial.print("#speedX");  Serial.print('=');
//    Serial.print(finalSpeedX); 
//    Serial.print("PosX");
//    Serial.print(finalPosX); 
//    Serial.println();
    
    //Y speed ---------------------------------------------------------------------------------------------------------
    
    // If accel is to low means that the object is not moving so accel will be set to 0
    if (finalSpeedY * cy < 0) {
     // Serial.print("<---->");
    //  Serial.print("\t");
      cy = cy;
    } else if (cy <= 0 && cy >= -0.02 || cy >= 0 && cy <= 0.02){
//      Serial.print("Set to 0");
//      Serial.println();
      cy= 0;
    }
    
    // Calculate Speed
    finalSpeedY = initialSpeedY + (cy*9.8)*time;
    
    
  if ( indexY < 3 ) {
    
      y_accels[indexY] = cy;
      indexY = indexY + 1;
    
  }
  
    // if the array is full
    else  {
      
      y_accels[0] = y_accels[1];
      y_accels[1] = y_accels[2];
      y_accels[2] = y_accels[3];
      y_accels[3] = cy;
      
//      for (int y = 0; y <= 3; y++) {
//        Serial.println(y_accels[y]);
//        if ( y_accels[y] <= 0 && y_accels[y] >= -0.03 || y_accels[y] >= 0 && y_accels[y] <= 0.03) {
//          yZeros++;
//          
//        }
//        
//        if (y == 3 && yZeros == 4 ) {
//            Serial.println("now");
//            yZeros = 0;
//            finalSpeedY = 0;
//          } else {
//            yZeros = 0;
//            
//        }
//      }
  
      if (  y_accels[0] <= 0 && y_accels[0] >= -0.02 || y_accels[0] >= 0 && y_accels[0] <= 0.02) {
        y_accels_Zero = true;
        Serial.println("[0]");
      }
      if (  y_accels[1] <= 0 && y_accels[1] >= -0.02 || y_accels[1] >= 0 && y_accels[1] <= 0.02) {
        y_accels_One = true;
        Serial.println("[1]");
      }
      if (  y_accels[2] <= 0 && y_accels[2] >= -0.02 || y_accels[2] >= 0 && y_accels[2] <= 0.02) {
        y_accels_Two = true;
        Serial.println("[2]");
      }
      if (  y_accels[3] <= 0 && y_accels[3] >= -0.02 || y_accels[3] >= 0 && y_accels[3] <= 0.02) {
        y_accels_Three = true;
        Serial.println("[3]");
      }
      
      if ( y_accels_Zero == true && y_accels_One == true && y_accels_Two == true && y_accels_Three == true){
        Serial.println("now");
        finalSpeedY = 0;
         y_accels_Zero = false;
         y_accels_One = false;
         y_accels_Two = false;
         y_accels_Three = false;
      } else {
        y_accels_Zero = false;
         y_accels_One = false;
         y_accels_Two = false;
         y_accels_Three = false;
      }
    }
    
    
      

    
    
   
    
    // Calculate position
    finalPosY = initialPosY + initialSpeedY * time + 0.5 * cy * time*time;
    
    initial_Y_accel = final_Y_accel;
     
    initialSpeedY = finalSpeedY;
    
    initialPosY = finalPosY;
    
    Serial.print("#speedY");  Serial.print('=');
    Serial.print(finalSpeedY);
      Serial.print("#acceleration");  Serial.print('=');
      Serial.print(cy);


    Serial.print("positionY");
    Serial.print(finalPosY);
    Serial.println();
    
   
   
    
    // Z speed
    if (finalSpeedZ * cz < 0) {
//      Serial.print("<---->");
//      Serial.print("\t");
      cz = cz;
    } else if (cz <= 0 && cz >= -0.08 || cz >= 0 && cz <= 0.08){
      cz= 0;
    } 
    
    finalSpeedZ = initialSpeedZ + (cz*9.8)*time;
    
    if (finalSpeedZ >= initialSpeedZ && finalSpeedZ < 0 && initialSpeedZ < 0 ) {
      finalSpeedZ = 0;
    }
    
    
    
    
    // Calculate distance
    finalPosZ = initialPosZ + initialSpeedZ * time + 0.5 * cz * time*time;
    
    
    initialSpeedZ = finalSpeedZ;
    initialPosZ = finalPosZ;
    
   // z -->  finalPos = initialPos + initialSpeed * time - 0.5 * 9.8 * time*time
    // x -->  finalPos = initialPos + initialSpeed * time + 0.5 * cx * time*time
    
    initialSpeedZ = finalSpeedZ;
//    
//    Serial.print("#speedZ");  Serial.print('=');
//    Serial.print(finalSpeedZ);
//   Serial.print("positionZ");  
//   Serial.print(finalPosZ);
//    Serial.println();
    
}



void output_sensors_text(char raw_or_calibrated)
{
  Serial.print("#A-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(accel[0]); Serial.print(",");
  Serial.print(accel[1]); Serial.print(",");
  Serial.print(accel[2]); Serial.println();

  Serial.print("#M-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(magnetom[0]); Serial.print(",");
  Serial.print(magnetom[1]); Serial.print(",");
  Serial.print(magnetom[2]); Serial.println();

  Serial.print("#G-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(gyro[0]); Serial.print(",");
  Serial.print(gyro[1]); Serial.print(",");
  Serial.print(gyro[2]); Serial.println();
}



