package org.opentrackingtools.graph.edges.impl;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.bayesian.conjugate.UnivariateGaussianMeanVarianceBayesianEstimator;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.NormalInverseGammaDistribution;

import javax.annotation.Nonnull;

import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.edges.InferredEdge;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import com.vividsolutions.jts.linearref.LengthLocationMap;
import com.vividsolutions.jts.linearref.LinearLocation;
import com.vividsolutions.jts.linearref.LocationIndexedLine;

public class SimpleInferredEdge implements InferredEdge {

  protected final Integer edgeId;
  protected final Vector endPoint;
  protected final Vector startPoint;
  protected final Object backingEdge;
  protected final Geometry geometry;
  protected final Boolean hasReverse;
  protected final LengthIndexedLine lengthIndexedLine;
  protected final LocationIndexedLine locationIndexedLine;

  /*
   * This is the empty edge, which stands for free movement
   */
  protected final static SimpleInferredEdge emptyEdge =
      new SimpleInferredEdge();

  protected SimpleInferredEdge() {
    this.edgeId = null;
    this.endPoint = null;
    this.startPoint = null;
    this.backingEdge = null;
    this.geometry = null;
    this.hasReverse = null;
    this.lengthIndexedLine = null;
    this.locationIndexedLine = null;
  }

  public static SimpleInferredEdge getInferredEdge(@Nonnull Geometry geom,
    @Nonnull Object backingEdge, @Nonnull Integer edgeId, 
    @Nonnull InferenceGraph graph) {
    return new SimpleInferredEdge(geom, backingEdge, edgeId, graph);
  }
  
  protected SimpleInferredEdge(@Nonnull Geometry geom, @Nonnull Object backingEdge,
    @Nonnull Integer edgeId, @Nonnull InferenceGraph graph) {

    this.edgeId = edgeId;
    this.backingEdge = backingEdge;
        
    this.geometry = geom;
    
    this.hasReverse = graph.edgeHasReverse(geom);

    final Coordinate startPointCoord = geometry.getCoordinates()[0];

    this.startPoint =
        VectorFactory.getDefault().createVector2D(
            startPointCoord.x, startPointCoord.y);

    final Coordinate endPointCoord =
        geometry.getCoordinates()[geometry.getNumPoints() - 1];
            
    this.endPoint =
        VectorFactory.getDefault().createVector2D(
            endPointCoord.x, endPointCoord.y);
    
    this.lengthIndexedLine = new LengthIndexedLine(this.geometry);
    
    this.locationIndexedLine = new LocationIndexedLine(this.geometry);
  }

  @Override
  public int compareTo(InferredEdge o) {
    return this.getGeometry().compareTo(o.getGeometry());
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
    final SimpleInferredEdge other = (SimpleInferredEdge) obj;
    if (geometry == null) {
      if (other.geometry != null) {
        return false;
      }
    } else if (!geometry.equalsExact(other.geometry)) {
      return false;
    }
    return true;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.edges.impl.InferredEdge#getCenterPointCoord()
   */
  @Override
  public Coordinate getCenterPointCoord() {
    return this.geometry.getCentroid().getCoordinate();
  }

  @Override
  public Object getBackingEdge() {
    return this.backingEdge;
  }

  @Override
  public String getEdgeId() {
    return String.valueOf(edgeId);
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.edges.impl.InferredEdge#getEndPoint()
   */
  @Override
  public Vector getEndPoint() {
    return this.endPoint;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.edges.impl.InferredEdge#getGeometry()
   */
  @Override
  public Geometry getGeometry() {
    return geometry;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.edges.impl.InferredEdge#getLength()
   */
  @Override
  public Double getLength() {
    if (geometry == null) {
      return null;
    }
    return geometry.getLength();
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.edges.impl.InferredEdge#getStartPoint()
   */
  @Override
  public Vector getStartPoint() {
    return startPoint;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime
            * result
            + ((geometry == null) ? 0 : geometry 
                .hashCode());
    return result;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.edges.impl.InferredEdge#isEmptyEdge()
   */
  @Override
  public boolean isNullEdge() {
    return this == emptyEdge;
  }

  @Override
  public String toString() {
    if (this == emptyEdge)
      return "InferredEdge [null]";
    else
      return "InferredEdge [edgeId=" + edgeId + ", length="
          + getLength() + "]";
  }

  public static SimpleInferredEdge getNullEdge() {
    return SimpleInferredEdge.emptyEdge;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.edges.impl.InferredEdge#hasReverse()
   */
  @Override
  public boolean hasReverse() {
    return this.hasReverse;
  }

  @Override
  public void update(MultivariateGaussian stateBelief) {
  }

  @Override
  public LocationIndexedLine getLocationIndexedLine() {
    return this.locationIndexedLine;
  }

  @Override
  public LengthIndexedLine getLengthIndexedLine() {
    return this.lengthIndexedLine;
  }
  
}