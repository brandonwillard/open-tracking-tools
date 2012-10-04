package org.openplans.tools.tracking.impl;

import java.util.Date;

import com.vividsolutions.jts.geom.Coordinate;

public class ObservationData {

  private String vehicleId;
  private Date timestamp;
  private Coordinate coordsLatLon;
  private Double velocity;
  private Double heading;
  private Double accuracy;

  public String getVehicleId() {
    return vehicleId;
  }

  public Date getTimestamp() {
    return timestamp;
  }

  public Coordinate getObsCoordsLatLon() {
    return coordsLatLon;
  }

  public Double getVelocity() {
    return velocity;
  }

  public Double getHeading() {
    return heading;
  }

  public Double getAccuracy() {
    return accuracy;
  }

  public ObservationData(String vehicleId, Date timestamp,
    Coordinate coordsLatLon, Double velocity, Double heading,
    Double accuracy) {
    this.vehicleId = vehicleId;
    this.timestamp = timestamp;
    this.coordsLatLon = coordsLatLon;
    this.velocity = velocity;
    this.heading = heading;
    this.accuracy = accuracy;
  }

}
