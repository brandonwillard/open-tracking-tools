package org.opentrackingtools.model.impl;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;

import java.util.Date;

import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.ProjectedCoordinate;

import com.google.common.collect.ComparisonChain;
import com.vividsolutions.jts.geom.Coordinate;

public class SimpleGpsObservation implements GpsObservation {

  private final String sourceId;
  private final Date timestamp;
  private final Coordinate coordsLatLon;
  private final Double velocity;
  private final Double heading;
  private final Double accuracy;
  private final int recordNumber;
  private final Vector projPoint;
  private GpsObservation prevObs;
  private final ProjectedCoordinate coordsProjected;

  public SimpleGpsObservation(String sourceId, Date timestamp,
    Coordinate coordsLatLon, Double velocity, Double heading,
    Double accuracy, int recordNumber, GpsObservation prevObs, 
    ProjectedCoordinate coordsProjected) {
    this.sourceId = sourceId;
    this.timestamp = timestamp;
    this.coordsLatLon = coordsLatLon;
    this.velocity = velocity;
    this.heading = heading;
    this.accuracy = accuracy;
    this.recordNumber = recordNumber;
    this.projPoint = VectorFactory.getDefault().createVector2D(
                      coordsProjected.x, coordsProjected.y);
    this.prevObs = prevObs;
    this.coordsProjected = coordsProjected;
  }

  @Override
  public Double getAccuracy() {
    return accuracy;
  }

  @Override
  public Double getHeading() {
    return heading;
  }

  @Override
  public Coordinate getObsCoordsLatLon() {
    return coordsLatLon;
  }

  @Override
  public Date getTimestamp() {
    return timestamp;
  }

  @Override
  public Double getVelocity() {
    return velocity;
  }

  @Override
  public ProjectedCoordinate getObsProjected() {
    return this.coordsProjected;
  }

  @Override
  public GpsObservation getPreviousObservation() {
    return this.prevObs;
  }

  @Override
  public Vector getProjectedPoint() {
    return this.projPoint;
  }

  @Override
  public int getRecordNumber() {
    return this.recordNumber;
  }

  @Override
  public String getSourceId() {
    return sourceId;
  }

  @Override
  public int compareTo(GpsObservation o) {
    return ComparisonChain.start()
        .compare(this.timestamp, o.getTimestamp())
        .compare(this.sourceId, o.getSourceId()).result();
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
    final SimpleGpsObservation other = (SimpleGpsObservation) obj;
    if (timestamp == null) {
      if (other.timestamp != null) {
        return false;
      }
    } else if (!timestamp.equals(other.timestamp)) {
      return false;
    }
    if (sourceId == null) {
      if (other.sourceId != null) {
        return false;
      }
    } else if (!sourceId.equals(other.sourceId)) {
      return false;
    }
    return true;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime
            * result
            + ((timestamp == null) ? 0 : timestamp
                .hashCode());
    result =
        prime
            * result
            + ((sourceId == null) ? 0 : sourceId
                .hashCode());
    return result;
  }
  
  @Override
  public String toString() {
    StringBuilder builder = new StringBuilder();
    builder.append("SimpleObservation [sourceId=").append(sourceId)
        .append(", timestamp=").append(timestamp)
        .append(", coordsLatLon=").append(coordsLatLon)
        .append(", recordNumber=").append(recordNumber)
        .append(", coordsProjected=").append(coordsProjected)
        .append("]");
    return builder.toString();
  }

  public void reset() {
    this.prevObs = null;
  }

}
