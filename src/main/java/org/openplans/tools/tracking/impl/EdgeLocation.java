package org.openplans.tools.tracking.impl;

import java.util.Collections;
import java.util.List;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;

import com.google.common.base.Preconditions;
import com.vividsolutions.jts.geom.Coordinate;

import gov.sandia.cognition.math.matrix.Vector;

public class EdgeLocation {

  private final Coordinate coordLocation;
  private final Vector location;
  private final List<InferredEdge> path;
  
  public EdgeLocation(InferredEdge edgeInfo, Coordinate coordLocation,
    Vector location, List<InferredEdge> path) {
    Preconditions.checkNotNull(coordLocation);
    Preconditions.checkNotNull(location);
    Preconditions.checkNotNull(path);
    this.coordLocation = coordLocation;
    this.location = location;
    this.path = path;
  }

  public EdgeLocation(Observation observation) {
    this.coordLocation = observation.getObsPoint();
    this.location = observation.getProjectedPoint();
    this.path = Collections.emptyList();
  }
  
  public EdgeLocation(Observation observation, List<InferredEdge> path) {
    Preconditions.checkNotNull(path);
    this.coordLocation = observation.getObsPoint();
    this.location = observation.getProjectedPoint();
    this.path = path;
  }

  public Vector getLocation() {
    return location;
  }

  public Coordinate getCoordLocation() {
    return coordLocation;
  }

  public List<InferredEdge> getPath() {
    return path;
  }
  
  
}
