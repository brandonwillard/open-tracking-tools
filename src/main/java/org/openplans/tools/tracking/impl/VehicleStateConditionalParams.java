package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.matrix.Vector;

public class VehicleStateConditionalParams {

  private final PathEdge edge;
  private final Vector location;
  private final double distance;

  public VehicleStateConditionalParams(PathEdge edge, Vector location) {
    this.edge = edge;
    this.location = location;
    this.distance = 0d;
  }

  public VehicleStateConditionalParams(PathEdge edge,
    Vector location, double dist) {
    this.edge = edge;
    this.location = location;
    this.distance = dist;
  }

  public VehicleStateConditionalParams(Vector location) {
    this.edge = PathEdge.getEmptyPathEdge();
    this.location = location;
    this.distance = 0d;
  }

  public double getDistanceToCurrentEdge() {
    return this.distance;
  }

  public Vector getLocation() {
    return this.location;
  }

  public PathEdge getPathEdge() {
    return this.edge;
  }

}
