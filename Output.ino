void kirim(unsigned int dat)
{
  unsigned int data[3];

  data[0] = (dat % 1000) / 100;
  data[1] = (dat % 100) / 10;
  data[2] = (dat % 10);
  for (int i = 0; i < 3; i++) {
    Serial.write(data[i] + 48);
  }
}

// Check raw data
// If negatif raw data is converted to ganjil and if positive raw data is converted to genap
int cek(int a)
{
  if (a < 0) {
    if (a % 2 == 0)a = a + 1;
  } else {
    if (a % 2 == 1)a = a + 1;
  }
  return abs(a);
}

void output_full_sensors()
{
  int ax, ay, az, mx, my, mz, r, p, y, gix, giy, giz;
  extern short i,j,k,l;
  ax = round((accel[0] * 0.0039 + 10) * 10);
  ay = round((accel[1] * 0.0039 + 10) * 10);
  az = round((accel[2] * 0.0039 + 10) * 10);

  p = round(TO_DEG(pitch) + 180);
  y = round(TO_DEG(yaw) + 180);
  r = round(TO_DEG(roll) + 180);

  gx = round(cek(gyro[0] / 14.375 / 10)); // Biar 3 digit
  gy = round(cek(gyro[1] / 14.375 / 10));
  gz = round(cek(gyro[2] / 14.375 / 10));

  mx = round(magnetom[0]);
  my = round(magnetom[1]);
  mz = round(magnetom[2]);

  if (ax > 600)
  {
    ax = 600;
  }
  if (ay > 600)
  {
    ay = 600;
  }
  if (az > 600)
  {
    az = 600;
  }

  //Serial.println();
  //kirim(212); Serial.print(" ");
  //Serial.print("A");

  Serial.print("\r");
  Serial.print("016");
  Serial.print(" ");
  kirim(ax); Serial.print(" ");
  kirim(ay); Serial.print(" ");
  kirim(az); Serial.print(" ");
  kirim(gx); Serial.print(" ");
  kirim(gy); Serial.print(" ");
  kirim(gz); Serial.print(" ");
  kirim(r); Serial.print(" ");
  kirim(p); Serial.print(" ");
  kirim(y); Serial.print(" ");
  Serial.print("000");
  Serial.print(" ");
  Serial.print("000");
  Serial.print(" ");
  Serial.print("000");
  Serial.print(" ");
  Serial.print("000");
  Serial.print(" ");
  kirim(altitude); Serial.print(" ");
  kirim(temperature);
  
}


void output_sensors()
{

}



