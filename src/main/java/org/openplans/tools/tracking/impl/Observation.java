package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;

import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Map;

import org.opengis.referencing.operation.TransformException;
import org.openplans.tools.tracking.impl.util.GeoUtils;


import com.google.common.base.Preconditions;
import com.google.common.collect.Maps;
import com.vividsolutions.jts.geom.Coordinate;

public class Observation {

  private final String vehicleId;
  private final Date timestamp;
  private final Coordinate obsCoords;
  private final Coordinate obsPoint;
  private final Vector projPoint;

  private final Double velocity;

  private final Double heading;
  private final Double accuracy;
  private Observation prevObs;
  private final int recordNumber;

  /*
   * This map is how we keep track of the previous records for each vehicle.
   */
  private static Map<String, Observation> vehiclesToRecords = Maps
      .newConcurrentMap();

  private static final SimpleDateFormat sdf = new SimpleDateFormat(
      "yyyy-MM-dd hh:mm:ss");

  private Observation(String vehicleId, Date timestamp,
    Coordinate obsCoords, Coordinate obsPoint, Double velocity, Double heading,
    Double accuracy, Observation prevObs, int recordNumber) {
    this.recordNumber = recordNumber;
    this.vehicleId = vehicleId;
    this.timestamp = timestamp;
    this.obsCoords = obsCoords;
    this.obsPoint = obsPoint;
    this.velocity = velocity;
    this.heading = heading;
    this.accuracy = accuracy;
    this.projPoint = VectorFactory.getDefault().createVector2D(obsPoint.x,
        obsPoint.y);
    this.prevObs = prevObs;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (obj == null) {
      return false;
    }
    if (getClass() != obj.getClass()) {
      return false;
    }
    final Observation other = (Observation) obj;
    if (accuracy == null) {
      if (other.accuracy != null) {
        return false;
      }
    } else if (!accuracy.equals(other.accuracy)) {
      return false;
    }
    if (heading == null) {
      if (other.heading != null) {
        return false;
      }
    } else if (!heading.equals(other.heading)) {
      return false;
    }
    if (obsCoords == null) {
      if (other.obsCoords != null) {
        return false;
      }
    } else if (!obsCoords.equals(other.obsCoords)) {
      return false;
    }
    if (obsPoint == null) {
      if (other.obsPoint != null) {
        return false;
      }
    } else if (!obsPoint.equals(other.obsPoint)) {
      return false;
    }
    if (projPoint == null) {
      if (other.projPoint != null) {
        return false;
      }
    } else if (!projPoint.equals(other.projPoint)) {
      return false;
    }
    if (timestamp == null) {
      if (other.timestamp != null) {
        return false;
      }
    } else if (!timestamp.equals(other.timestamp)) {
      return false;
    }
    if (vehicleId == null) {
      if (other.vehicleId != null) {
        return false;
      }
    } else if (!vehicleId.equals(other.vehicleId)) {
      return false;
    }
    if (velocity == null) {
      if (other.velocity != null) {
        return false;
      }
    } else if (!velocity.equals(other.velocity)) {
      return false;
    }
    return true;
  }

  public Double getAccuracy() {
    return accuracy;
  }

  public Double getHeading() {
    return heading;
  }

  public Coordinate getObsCoords() {
    return obsCoords;
  }

  public Coordinate getObsPoint() {
    return obsPoint;
  }

  public Observation getPreviousObservation() {
    return prevObs;
  }

  public Vector getProjectedPoint() {
    return projPoint;
  }

  public Date getTimestamp() {
    return timestamp;
  }

  public String getVehicleId() {
    return vehicleId;
  }

  public Double getVelocity() {
    return velocity;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((accuracy == null) ? 0 : accuracy.hashCode());
    result = prime * result + ((heading == null) ? 0 : heading.hashCode());
    result = prime * result + ((obsCoords == null) ? 0 : obsCoords.hashCode());
    result = prime * result + ((obsPoint == null) ? 0 : obsPoint.hashCode());
    result = prime * result + ((projPoint == null) ? 0 : projPoint.hashCode());
    result = prime * result + ((timestamp == null) ? 0 : timestamp.hashCode());
    result = prime * result + ((vehicleId == null) ? 0 : vehicleId.hashCode());
    result = prime * result + ((velocity == null) ? 0 : velocity.hashCode());
    return result;
  }

  private void reset() {
    this.prevObs = null;
  }

  public static void clearRecordData() {
    vehiclesToRecords.clear();
  }

  public static synchronized Observation createObservation(String vehicleId, Date time,
    Coordinate obsCoords, Double velocity, Double heading, Double accuracy) throws TimeOrderException {
    final Coordinate obsPoint = GeoUtils.convertToEuclidean(obsCoords);

    final Observation prevObs = vehiclesToRecords.get(vehicleId);

    /*
     * do this so we don't potentially hold on to every record in memory
     */
    final int recordNumber;
    if (prevObs != null) {
      recordNumber = prevObs.getRecordNumber() + 1;
      prevObs.reset();

      /*
       * We check for out-of-time-order records.
       */
      if (time.getTime() <= prevObs.getTimestamp().getTime())
        throw new TimeOrderException();
      
    } else {
      recordNumber = 1;
    }

    final Observation obs = new Observation(vehicleId, time,
        obsCoords, obsPoint, velocity, heading, accuracy, prevObs, recordNumber);

    vehiclesToRecords.put(vehicleId, obs);
    
    return obs;
  }
  
  public static synchronized Observation createObservation(
    String vehicleId, String timestamp, String latStr, String lonStr,
    String velocity, String heading, String accuracy)
      throws NumberFormatException, ParseException, TransformException, TimeOrderException {
    
    final double lat = Double.parseDouble(latStr);
    final double lon = Double.parseDouble(lonStr);
    final Coordinate obsCoords = new Coordinate(lon, lat);
    final Double velocityd = velocity != null ? Double.parseDouble(velocity) : null;
    final Double headingd = heading != null ? Double.parseDouble(heading) : null;
    final Double accuracyd = accuracy != null ? Double.parseDouble(accuracy) : null;
    final Date time = sdf.parse(timestamp);

    return createObservation(vehicleId, time, obsCoords, velocityd, headingd, accuracyd);
  }

  @Override
  public String toString() {
    return "Observation [vehicleId=" + vehicleId + ", timestamp=" + timestamp
        + ", obsCoords=" + obsCoords + ", obsPoint=" + obsPoint
        + ", velocity=" + velocity + ", heading="
        + heading + ", accuracy=" + accuracy + ", prevObs=" + prevObs + "]";
  }

  public Vector getProjPoint() {
    return projPoint;
  }

  public Observation getPrevObs() {
    return prevObs;
  }

  public int getRecordNumber() {
    return recordNumber;
  }

  public static Map<String, Observation> getVehiclesToRecords() {
    return vehiclesToRecords;
  }

  public static SimpleDateFormat getSdf() {
    return sdf;
  }

  public static void remove(String name) {
    vehiclesToRecords.remove(name);
  }


}
