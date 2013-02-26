package org.opentrackingtools.edges;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;

import javax.annotation.Nonnull;

import org.opentrackingtools.graph.InferenceGraph;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

public class InferredEdge implements Comparable<InferredEdge> {

  protected final Integer edgeId;
  protected final Vector endPoint;
  protected final Vector startPoint;
  protected final Object backingEdge;
  protected final Geometry geometry;
  protected final Boolean hasReverse;

  /*
   * This is the empty edge, which stands for free movement
   */
  protected final static InferredEdge emptyEdge =
      new InferredEdge();

  protected InferredEdge() {
    this.edgeId = null;
    this.endPoint = null;
    this.startPoint = null;
    this.backingEdge = null;
    this.geometry = null;
    this.hasReverse = null;
  }

  public static InferredEdge getInferredEdge(@Nonnull Geometry geom,
    @Nonnull Object backingEdge, @Nonnull Integer edgeId, 
    @Nonnull InferenceGraph graph) {
    return new InferredEdge(geom, backingEdge, edgeId, graph);
  }
  
  protected InferredEdge(@Nonnull Geometry geom, @Nonnull Object backingEdge,
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
    final InferredEdge other = (InferredEdge) obj;
    if (geometry == null) {
      if (other.geometry != null) {
        return false;
      }
    } else if (!geometry.equalsExact(other.geometry)) {
      return false;
    }
    return true;
  }

  public Coordinate getCenterPointCoord() {
    return this.geometry.getCentroid().getCoordinate();
  }

  public Object getBackingEdge() {
    return this.backingEdge;
  }

  public String getEdgeId() {
    return String.valueOf(edgeId);
  }

  public Vector getEndPoint() {
    return this.endPoint;
  }

  public Geometry getGeometry() {
    return geometry;
  }

  public Double getLength() {
    if (geometry == null) {
      return null;
    }
    return geometry.getLength();
  }

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

  public static InferredEdge getNullEdge() {
    return InferredEdge.emptyEdge;
  }

  public boolean hasReverse() {
    return this.hasReverse;
  }

}