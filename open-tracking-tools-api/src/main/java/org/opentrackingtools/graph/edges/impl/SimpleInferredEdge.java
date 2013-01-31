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

  private final Integer edgeId;
//  private final Vertex startVertex;
//  private final Vertex endVertex;
  private final Vector endPoint;
  private final Vector startPoint;
  
  private final NormalInverseGammaDistribution velocityPrecisionDist;
  private final UnivariateGaussianMeanVarianceBayesianEstimator velocityEstimator;

  private final Object backingEdge;

  private final Geometry geometry;
  private final LocationIndexedLine locationIndexedLine;
  private final LengthIndexedLine lengthIndexedLine;
  private final LengthLocationMap lengthLocationMap;
  private final Boolean hasReverse;

  /*
   * This is the empty edge, which stands for free movement
   */
  private final static SimpleInferredEdge emptyEdge =
      new SimpleInferredEdge();

  private SimpleInferredEdge() {
    this.edgeId = null;
    this.endPoint = null;
    this.startPoint = null;
    this.velocityEstimator = null;
    this.velocityPrecisionDist = null;
//    this.startVertex = null;
//    this.endVertex = null;

    this.backingEdge = null;

    this.geometry = null;
    this.locationIndexedLine = null;
    this.lengthIndexedLine = null;
    this.lengthLocationMap = null;
    this.hasReverse = null;
  }

  public static SimpleInferredEdge getInferredEdge(@Nonnull Geometry geom,
    @Nonnull Object backingEdge, @Nonnull Integer edgeId, 
    @Nonnull InferenceGraph graph) {
    return new SimpleInferredEdge(geom, backingEdge, edgeId, graph);
  }
  
  private SimpleInferredEdge(@Nonnull Geometry geom, @Nonnull Object backingEdge,
    @Nonnull Integer edgeId, @Nonnull InferenceGraph graph) {

    this.edgeId = edgeId;
    this.backingEdge = backingEdge;
        
    this.geometry = geom;
    
    this.hasReverse = graph.edgeHasReverse(geom);

    this.locationIndexedLine =
        new LocationIndexedLine(geometry);
    this.lengthIndexedLine =
        new LengthIndexedLine(geometry);
    this.lengthLocationMap =
        new LengthLocationMap(geometry);

//    this.startVertex = backingEdge.getFromVertex();
//    this.endVertex = backingEdge.getToVertex();

    final Coordinate startPointCoord =
        this.locationIndexedLine
            .extractPoint(this.locationIndexedLine
                .getStartIndex());

    this.startPoint =
        VectorFactory.getDefault().createVector2D(
            startPointCoord.x, startPointCoord.y);

    final Coordinate endPointCoord =
        this.locationIndexedLine
            .extractPoint(this.locationIndexedLine
                .getEndIndex());
    this.endPoint =
        VectorFactory.getDefault().createVector2D(
            endPointCoord.x, endPointCoord.y);

    this.velocityPrecisionDist =
        // ~4.4 m/s, std. dev ~ 30 m/s, Gamma with exp. value = 30 m/s
        // TODO perhaps variance of velocity should be in m/s^2. yeah...
        new NormalInverseGammaDistribution(4.4d,
            1d / Math.pow(30d, 2d),
            1d / Math.pow(30d, 2d) + 1d, Math.pow(30d, 2d));
    this.velocityEstimator =
        new UnivariateGaussianMeanVarianceBayesianEstimator(
            velocityPrecisionDist);
  }

  @Override
  public int compareTo(InferredEdge o) {
//    final CompareToBuilder comparator =
//        new CompareToBuilder();
//    comparator.append(this.endVertex.getLabel(),
//        o.endVertex.getLabel());
//    comparator.append(this.startVertex.getLabel(),
//        o.startVertex.getLabel());
//    return comparator.toComparison();
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
//    if (endVertex == null) {
//      if (other.endVertex != null) {
//        return false;
//      }
//    } else if (!endVertex.equals(other.endVertex)) {
//      return false;
//    }
//    if (startVertex == null) {
//      if (other.startVertex != null) {
//        return false;
//      }
//    } else if (!startVertex.equals(other.startVertex)) {
//      return false;
//    }
    return true;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.edges.impl.InferredEdge#getCenterPointCoord()
   */
  @Override
  public Coordinate getCenterPointCoord() {
    return this.geometry.getCentroid().getCoordinate();
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.edges.impl.InferredEdge#getCoordOnEdge(gov.sandia.cognition.math.matrix.Vector)
   */
  @Override
  public Coordinate getCoordOnEdge(Vector obsPoint) {
    if (this.isNullEdge())
      return null;
    final Coordinate revObsPoint =
        new Coordinate(obsPoint.getElement(1),
            obsPoint.getElement(0));
    final LinearLocation here =
        locationIndexedLine.project(revObsPoint);
    final Coordinate pointOnLine =
        locationIndexedLine.extractPoint(here);
    final Coordinate revOnLine =
        new Coordinate(pointOnLine.y, pointOnLine.x);
    return revOnLine;
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

//  public Vertex getEndVertex() {
//    return endVertex;
//  }

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
   * @see org.opentrackingtools.graph.edges.impl.InferredEdge#getLengthIndexedLine()
   */
  @Override
  public LengthIndexedLine getLengthIndexedLine() {
    return lengthIndexedLine;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.edges.impl.InferredEdge#getLengthLocationMap()
   */
  @Override
  public LengthLocationMap getLengthLocationMap() {
    return lengthLocationMap;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.edges.impl.InferredEdge#getLocationIndexedLine()
   */
  @Override
  public LocationIndexedLine getLocationIndexedLine() {
    return locationIndexedLine;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.edges.impl.InferredEdge#getPointOnEdge(com.vividsolutions.jts.geom.Coordinate)
   */
  @Override
  public Vector getPointOnEdge(Coordinate obsPoint) {
    if (this.isNullEdge())
      return null;
    final LinearLocation here =
        locationIndexedLine.project(obsPoint);
    final Coordinate pointOnLine =
        locationIndexedLine.extractPoint(here);
    return VectorFactory.getDefault().createVector2D(
        pointOnLine.x, pointOnLine.y);
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.edges.impl.InferredEdge#getStartPoint()
   */
  @Override
  public Vector getStartPoint() {
    return startPoint;
  }

//  public Vertex getStartVertex() {
//    return startVertex;
//  }

  public UnivariateGaussianMeanVarianceBayesianEstimator
      getVelocityEstimator() {
    return velocityEstimator;
  }

  public NormalInverseGammaDistribution
      getVelocityPrecisionDist() {
    return velocityPrecisionDist;
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
//    result =
//        prime
//            * result
//            + ((endVertex == null) ? 0 : endVertex
//                .hashCode());
//    result =
//        prime
//            * result
//            + ((startVertex == null) ? 0 : startVertex
//                .hashCode());
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
    final double velocity =
        stateBelief.getMean().getElement(1);
    this.getVelocityEstimator()
        .update(this.getVelocityPrecisionDist(), Math.abs(velocity));
  }
  
}