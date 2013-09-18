package org.opentrackingtools.model;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;

import java.util.Date;

import org.apache.commons.lang3.builder.ToStringBuilder;
import org.apache.commons.lang3.builder.ToStringStyle;

import com.google.common.collect.ComparisonChain;
import com.vividsolutions.jts.geom.Coordinate;

public class GpsObservation implements Comparable<GpsObservation> {

  protected Double fixQuality;
  protected final Coordinate coordsLatLon;
  protected final ProjectedCoordinate coordsProjected;
  protected final Double heading;
  protected GpsObservation prevObs;
  protected final Vector projPoint;
  protected final int recordNumber;
  protected final String sourceId;
  protected final Date timestamp;
  protected final Double velocity;

  public GpsObservation(String sourceId, Date timestamp,
    Coordinate coordsLatLon, Double velocity, Double heading,
    Double accuracy, int recordNumber, GpsObservation prevObs,
    ProjectedCoordinate coordsProjected) {
    this.sourceId = sourceId;
    this.timestamp = timestamp;
    this.coordsLatLon = coordsLatLon;
    this.velocity = velocity;
    this.heading = heading;
    this.fixQuality = accuracy;
    this.recordNumber = recordNumber;
    this.projPoint =
        VectorFactory.getDefault().createVector2D(coordsProjected.x,
            coordsProjected.y);
    this.prevObs = prevObs;
    this.coordsProjected = coordsProjected;
  }

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
    if (this.getClass() != obj.getClass()) {
      return false;
    }
    final GpsObservation other = (GpsObservation) obj;
    if (this.timestamp == null) {
      if (other.timestamp != null) {
        return false;
      }
    } else if (!this.timestamp.equals(other.timestamp)) {
      return false;
    }
    if (this.sourceId == null) {
      if (other.sourceId != null) {
        return false;
      }
    } else if (!this.sourceId.equals(other.sourceId)) {
      return false;
    }
    return true;
  }

  public Double getFixQuality() {
    return this.fixQuality;
  }

  public Double getHeading() {
    return this.heading;
  }

  public Coordinate getObsCoordsLatLon() {
    return this.coordsLatLon;
  }

  public ProjectedCoordinate getObsProjected() {
    return this.coordsProjected;
  }

  public GpsObservation getPreviousObservation() {
    return this.prevObs;
  }

  public Vector getProjectedPoint() {
    return this.projPoint;
  }

  public int getRecordNumber() {
    return this.recordNumber;
  }

  public String getSourceId() {
    return this.sourceId;
  }

  public Date getTimestamp() {
    return this.timestamp;
  }

  public Double getVelocity() {
    return this.velocity;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime
            * result
            + ((this.timestamp == null) ? 0 : this.timestamp
                .hashCode());
    result =
        prime
            * result
            + ((this.sourceId == null) ? 0 : this.sourceId.hashCode());
    return result;
  }

  public void reset() {
    this.prevObs = null;
  }

  @Override
  public String toString() {
    final ToStringBuilder builder =
        new ToStringBuilder(this, ToStringStyle.SHORT_PREFIX_STYLE);
    builder.append("recordNum", this.recordNumber);
    builder.append("sourceId", this.sourceId);
    builder.append("coords", this.coordsLatLon);
    return builder.toString();
  }

}
