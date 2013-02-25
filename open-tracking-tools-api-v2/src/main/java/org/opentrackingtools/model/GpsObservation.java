package org.opentrackingtools.model;

import gov.sandia.cognition.math.matrix.Vector;

import java.util.Date;

import com.vividsolutions.jts.geom.Coordinate;

public interface GpsObservation extends Comparable<GpsObservation> {

  public Double getAccuracy();

  public Double getHeading();

  public Coordinate getObsCoordsLatLon();

  public ProjectedCoordinate getObsProjected();

  public GpsObservation getPreviousObservation();

  public Vector getProjectedPoint();

  public int getRecordNumber();

  public Date getTimestamp();

  public String getSourceId();

  public Double getVelocity();

}