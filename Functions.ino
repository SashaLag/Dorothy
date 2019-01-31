void Pan_Flow() {
  if (xscanDirection == 0) {
    for (xpos=Min_Pan,xPan=0; xpos<=Max_Pan; xpos+=2.5,++xPan) {
      PAN.write(xpos);   //go to position in variable 'pos'
      delay(5);
      Tilt_Flow();
      yscanDirection = ~yscanDirection;
    }
  }
  else {
    for (xpos=Max_Pan,xPan=Iter_Pan-1;xpos>=Min_Pan;xpos-=2.5,--xPan) {
      PAN.write(xpos);   //go to position in variable 'pos'
      delay(5);
      Tilt_Flow();
      yscanDirection = ~yscanDirection;
    }
  }
  xscanDirection = ~xscanDirection;
}


void Tilt_Flow() {
  if (yscanDirection == 0) {
    for (ypos = Min_Tilt; ypos <= Max_Tilt; ypos += 5) {
      TILT.write(ypos);   //go to position in variable 'pos'
      delay(5);
      lidar_acquisition();
    }
  }
  else {
    for (ypos = Max_Tilt; ypos >= Min_Tilt; ypos -= 5) {
      TILT.write(ypos);   //go to position in variable 'pos'
      delay(5);
      lidar_acquisition();
    }
  }
}


void lidar_acquisition() {
  uint16_t radius = tfmini.getDistance();
  if (radius < LiDAR_Radius[xPan]) LiDAR_Radius[xPan] = radius;
  
  float azimuth = (-90 + Max_Pan + xpos) * deg2rad;
  float elevation = (45 + ypos) * deg2rad;
  double x = radius * sin(elevation) * cos(azimuth);
  double y = radius * sin(elevation) * sin(azimuth);
  double z = radius * cos(elevation);
  Serial.println(String(x,5)+" "+String(y,5)+" "+String(-z,5));

  delay(10);  //Lidar Acquisition Delay. No Movement can be done
}


int ChooseDirection() {
  for (int i=0; i<Iter_Pan; ++i) {
    if(LiDAR_Radius[i] < 100) {
       ++search;
       return 0;
    }
    else return 1;
  }
}


void ChangeDirection() {
      LEFT.drive(120);
      RIGHT.drive(-120);
      delay(280);
      brake(LEFT, RIGHT);
}


void STOP() {
  brake(LEFT,RIGHT);
}
