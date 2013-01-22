package org.opentrackingtools.impl;

import java.util.Date;

import com.vividsolutions.jts.geom.Coordinate;

public class ObservationData {

  private final String vehicleId;
  private final Date timestamp;
  private final Coordinate coordsLatLon;
  private final Double velocity;
  private final Double heading;
  private final Double accuracy;

  public ObservationData(String vehicleId, Date timestamp,
    Coordinate coordsLatLon, Double velocity,
    Double heading, Double accuracy) {
    this.vehicleId = vehicleId;
    this.timestamp = timestamp;
    this.coordsLatLon = coordsLatLon;
    this.velocity = velocity;
    this.heading = heading;
    this.accuracy = accuracy;
  }

  public Double getAccuracy() {
    return accuracy;
  }

  public Double getHeading() {
    return heading;
  }

  public Coordinate getObsCoordsLatLon() {
    return coordsLatLon;
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

}
