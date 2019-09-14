/******************************************************************************
   Sun location
******************************************************************************/
#define DEG_TO_RAD(x)  ((x)*pi/180.0)
#define RAD_TO_DEG(x)  ((x)*180/pi)

/******************************************************************************
    Calculates the expected location of the sun, based on the configured location and actual time.
    by:     Christian Oekermann <christian@oekermann.com>

    Parameters:
      time: in UTC
      sunPosition: Helios object
      longtitude: in degrees
      latitude: in degrees
      p_azi: pointer to double: Azimuth
      p_elev: pointer to double: Elevation
    Postcondition:
      location pointed to by p_azi: azimuth in degrees [0..360)
      location pointed to by p_elev: elevation in degrees [-90..90)
*/
void calculateSunLocation(DateTime time, Helios sunPosition,
                          double longtitude, double latitude, double* p_azi, double* p_elev) {
  sunPosition.calcSunPos(time.year(), time.month(), time.day(),
                         time.hour(), time.minute(), time.second(),
                         longtitude, latitude);
  *p_azi = sunPosition.dAzimuth;
  *p_elev = sunPosition.dElevation;
}

/******************************************************************************
    Calculates the expected location pair from alt azimuth to stable Equatorial coordinate system
    ref: https://de.wikipedia.org/wiki/Astronomische_Koordinatensysteme#Horizontale_(a,_h)_%E2%86%92_kartesische_Koordinaten_%E2%86%92_ruhende_%C3%A4quatoriale_Koordinaten_(%CF%84,_%CE%B4)

    Status:   unit test of this gave incorrect results!

    by:     Gregory Fung <gwfung@gmail.com>

    Parameters:
      latitude: latitude of observer location
      p_delta_DEG: pointer to output double: Declination in degrees [-90..90°)
    output:
      targetPositionX: equatorial rotation in degrees [0..360)
      targetPositionY: omega in degrees [-90..90)
*/
void convertToStableEquatorialAxes_DEAD_CODE(double latitude, double azimuth, double elevation, 
  double * p_tau_DEG, double * p_delta_DEG) {
  /*
    ϕ \phi  = geographische Breite (latitude)
    a  a  = Azimut
    h  h  = Höhenwinkel (elevation)
    τ  \tau   = Stundenwinkel (hour angle)
    δ  \delta   = Deklination (declination)

    δ = arcsin ⁡ ( sin ⁡ ϕ ⋅ sin ⁡ h − cos ⁡ ϕ ⋅ cos ⁡ h ⋅ cos ⁡ a )
    τ = arctan ⁡ ( sin ⁡ a / (sin ⁡ ϕ ⋅ cos ⁡ a + cos ⁡ ϕ ⋅ tan ⁡ h ))
  */

  double lat_RAD = DEG_TO_RAD(latitude);
  double azi_RAD = DEG_TO_RAD(azimuth);
  double elev_RAD = DEG_TO_RAD(elevation);

  double sin_phi = sin(lat_RAD);
  double cos_phi = cos(lat_RAD);
  double sin_a = sin(azi_RAD);
  double cos_a = cos(azi_RAD);
  double sin_h = sin(elev_RAD);
  double cos_h = cos(elev_RAD);
  double tan_h = tan(elev_RAD); //TODO handle + or - 90° case?

  double sin_delta = sin_phi * sin_h - cos_phi * cos_h * cos_a;
  double delta_RAD = asin(sin_delta);
  *p_delta_DEG = RAD_TO_DEG(delta_RAD);

  double tau_denominator = sin_phi * cos_a + cos_phi * tan_h;
  double tan_tau = sin_a / tau_denominator;
  double tau_RAD = atan(tan_tau);
  //atran only returns results in quadrants I and IV.   Adjust for results in quadrant II and III
  if (tau_denominator < 0) {      //left half plane
    //quadrant II: correct from quadrant IV; quadrant III: correct from quadrant I
    tau_RAD += pi;        //same correction for both
  }
  *p_tau_DEG = RAD_TO_DEG(tau_RAD);

  /* math debug
    Serial.print(",| lat_RAD ");
    Serial.print(lat_RAD);
    Serial.print(", azi_RAD ");
    Serial.print(azi_RAD);
    Serial.print(", elev_RAD ");
    Serial.print(elev_RAD);

    Serial.print(", sin_phi ");
    Serial.print(sin_phi);
    Serial.print(", cos_phi ");
    Serial.print(cos_phi);
  */
  Serial.print(",| sin_a ");
  Serial.print(sin_a);
  Serial.print(", cos_a ");
  Serial.print(cos_a);
  Serial.print(", sin_h ");
  Serial.print(sin_h);
  Serial.print(", cos_h ");
  Serial.print(cos_h);
  Serial.print(", tan_h ");
  Serial.print(tan_h);
  Serial.print(",| tau_denominator ");
  Serial.print(tau_denominator);
  Serial.print(", tan_tau ");
  Serial.print(tan_tau);
  Serial.print(", sin_delta ");
  Serial.print(sin_delta);


  Serial.print(",| Equa: tau ");
  Serial.print(*p_tau_DEG);
  Serial.print("deg,| delta ");
  Serial.print(*p_delta_DEG);
  Serial.print("deg");
}

/******************************************************************************
    Calculates the expected location pair from alt azimuth to stable Equatorial coordinate system
    ref: http://star-www.st-and.ac.uk/~fv/webnotes/chapter7.htm
    (not implementing the step from hour angle to RA)
    by:     Gregory Fung <gwfung@gmail.com>

    Parameters:
      latitude: latitude of observer location
      azimuth: azimuth in degrees [0..360)
      elevation: elevation in degrees [-90..90)
      p_tau_DEG: pointer to output double: Stundenwinkel (hour angle) in degrees [0..360°)
      p_delta_DEG: pointer to output double: Declination in degrees [-90..90°)
    output:
      targetPositionX: equatorial rotation in degrees [-180..180)
      targetPositionY: omega in degrees [-90..90)
*/
void convertToStableEquatorialAxes(double latitude, double azimuth, double elevation,
                                   double * p_tau_DEG, double * p_delta_DEG) {

  /*
    azimuth A
    altitude a (elevation)
    latitude φ
    Local Hour Angle H (also known as τ tau)
    declination δ
    Given φ phi, a and A, what are H (τ) and δ delta?

    sin(δ) = sin(a)sin(φ) + cos(a) cos(φ) cos(A)
    sin(H) = - sin(A) cos(a) / cos(δ)
    cos(H) = { sin(a) - sin(δ) sin(φ) } / cos(δ) cos(φ)
    use both cos and sin H to map the angle to the correct quadrant.
  */

  double lat_RAD = DEG_TO_RAD(latitude);
  double elev_RAD = DEG_TO_RAD(elevation);
  double azi_RAD = DEG_TO_RAD(azimuth);

  double sin_a = sin(elev_RAD);
  double cos_a = cos(elev_RAD);
  double sin_phi = sin(lat_RAD);
  double cos_phi = cos(lat_RAD);
  double sin_A = sin(azi_RAD);
  double cos_A = cos(azi_RAD);

  double sin_delta = sin_a * sin_phi + cos_a * cos_phi * cos_A;
  double delta_RAD = asin(sin_delta);     //output range [-90..90°], quad I and IV
  double cos_delta = cos(delta_RAD);
  *p_delta_DEG = RAD_TO_DEG(delta_RAD);
  *p_delta_DEG = correctArcsineQuadrant(cos_delta, *p_delta_DEG);

  double sin_H = -1 * sin_A * cos_a / cos_delta;
  double cos_H_numerator = (sin_a - sin_delta * sin_phi);
  double cos_H = cos_H_numerator / (cos_delta * cos_phi);
  *p_tau_DEG = RAD_TO_DEG(asin(sin_H));   //output range [-90..90°], quad I and IV
  *p_tau_DEG = correctArcsineQuadrant(cos_H, *p_tau_DEG);

  /* math debug
    Serial.print(",| sin_a ");
    Serial.print(sin_a);
    Serial.print(", cos_a ");
    Serial.print(cos_a);
    Serial.print(", sin_phi ");
    Serial.print(sin_phi);
    Serial.print(", cos_phi ");
    Serial.print(cos_phi);
    Serial.print(", sin_A ");
    Serial.print(sin_A);
    Serial.print(", cos_A ");
    Serial.print(cos_A);

    Serial.print(",| sin_delta ");
    Serial.print(sin_delta);
    Serial.print(", cos_delta ");
    Serial.print(cos_delta);
    Serial.print(", delta_RAD ");
    Serial.print(delta_RAD);
    Serial.print(", sin_H ");
    Serial.print(sin_H);
    Serial.print(", cos_H_num ");
    Serial.print(cos_H_numerator);
    Serial.print(", cos_H ");
    Serial.print(cos_H);
  */

#ifdef SERIAL_DEBUG_SUN_CALC
  Serial.print(",| in fixed Equatorial: delta ");
  Serial.print(*p_delta_DEG);
  Serial.print("deg, H tau ");
  Serial.print(*p_tau_DEG);
  Serial.print("deg");
#endif
}

double correctArcsineQuadrant(double cos_angle, double angle) {
  double result;
  // Arcsin gives unclear result for left or right half plane.
  // Determine from cos if we should be in the left half plane.
  result = angle;
  if (cos_angle < 0) {
    //mirror quad I to quad II, mirror quad IV to quad III
    result = 180.0 - angle;     // angle complement works perfectly.  eg: -45°: 180 - -45 = 225°
  }
  if (result < -180) {        // should never be, but this code doesn't hurt
    result += 360.0;
  } else if (result > 180) {
    result -= 360.0;
  }
  return result;
}
