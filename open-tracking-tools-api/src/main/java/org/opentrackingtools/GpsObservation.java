package org.opentrackingtools;

import gov.sandia.cognition.math.matrix.Vector;

import java.util.Date;

import org.opentrackingtools.util.geom.ProjectedCoordinate;

import com.vividsolutions.jts.geom.Coordinate;

public interface GpsObservation extends Comparable<GpsObservation> {

  public abstract Double getAccuracy();

  public abstract Double getHeading();

  public abstract Coordinate getObsCoordsLatLon();

  public abstract ProjectedCoordinate getObsPoint();

  public abstract GpsObservation getPreviousObservation();

  public abstract Vector getProjectedPoint();

  public abstract int getRecordNumber();

  public abstract Date getTimestamp();

  public abstract String getSourceId();

  public abstract Double getVelocity();

}