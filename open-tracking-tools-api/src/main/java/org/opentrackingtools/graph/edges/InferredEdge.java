package org.opentrackingtools.graph.edges;

import java.util.Collection;
import java.util.List;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import org.opentrackingtools.graph.InferenceGraph;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import com.vividsolutions.jts.linearref.LengthLocationMap;
import com.vividsolutions.jts.linearref.LocationIndexedLine;

public interface InferredEdge extends Comparable<InferredEdge> {

  public abstract Coordinate getCenterPointCoord();

  public abstract Coordinate getCoordOnEdge(Vector obsPoint);

  public abstract Vector getEndPoint();

  public abstract Geometry getGeometry();

  public abstract Double getLength();

  public abstract LengthIndexedLine getLengthIndexedLine();

  public abstract LengthLocationMap getLengthLocationMap();

  public abstract LocationIndexedLine getLocationIndexedLine();

  /**
   * Get the snapped location in projected/euclidean coordinates for the given
   * obsPoint (in lat/lon).
   * 
   * @param obsPoint
   * @return
   */
  public abstract Vector getPointOnEdge(Coordinate obsPoint);

  public abstract Vector getStartPoint();

  public abstract boolean isNullEdge();

  public abstract boolean hasReverse();

  public abstract String getEdgeId();
  
  public abstract Object getBackingEdge();

  public abstract void update(MultivariateGaussian stateBelief);

}