package org.opentrackingtools.graph;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;

import javax.annotation.Nonnull;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

public class InferenceGraphEdge implements
    Comparable<InferenceGraphEdge> {

  /*
   * This is the empty edge, which stands for free movement
   */
  protected final static InferenceGraphEdge emptyEdge =
      new InferenceGraphEdge();

  public static InferenceGraphEdge getInferredEdge(
    @Nonnull Geometry geom, @Nonnull Object backingEdge,
    @Nonnull Integer edgeId, @Nonnull InferenceGraph graph) {
    return new InferenceGraphEdge(geom, backingEdge, edgeId, graph);
  }

  public static InferenceGraphEdge getNullEdge() {
    return InferenceGraphEdge.emptyEdge;
  }

  protected final Object backingEdge;
  protected final Integer edgeId;
  protected final Vector endPoint;

  protected final Geometry geometry;

  protected final Boolean hasReverse;

  protected final Vector startPoint;

  public InferenceGraphEdge() {
    this.edgeId = null;
    this.endPoint = null;
    this.startPoint = null;
    this.backingEdge = null;
    this.geometry = null;
    this.hasReverse = null;
  }

  protected InferenceGraphEdge(@Nonnull Geometry geom,
    @Nonnull Object backingEdge, @Nonnull Integer edgeId,
    @Nonnull InferenceGraph graph) {

    this.edgeId = edgeId;
    this.backingEdge = backingEdge;

    this.geometry = geom;

    this.hasReverse = graph.edgeHasReverse(geom);

    final Coordinate startPointCoord =
        this.geometry.getCoordinates()[0];

    this.startPoint =
        VectorFactory.getDefault().createVector2D(startPointCoord.x,
            startPointCoord.y);

    final Coordinate endPointCoord =
        this.geometry.getCoordinates()[this.geometry.getNumPoints() - 1];

    this.endPoint =
        VectorFactory.getDefault().createVector2D(endPointCoord.x,
            endPointCoord.y);

  }

  @Override
  public int compareTo(InferenceGraphEdge o) {
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
    if (this.getClass() != obj.getClass()) {
      return false;
    }
    final InferenceGraphEdge other = (InferenceGraphEdge) obj;
    if (this.geometry == null) {
      if (other.geometry != null) {
        return false;
      }
    } else if (!this.geometry.equalsExact(other.geometry)) {
      return false;
    }
    return true;
  }

  public Object getBackingEdge() {
    return this.backingEdge;
  }

  public Coordinate getCenterPointCoord() {
    return this.geometry.getCentroid().getCoordinate();
  }

  public String getEdgeId() {
    return String.valueOf(this.edgeId);
  }

  public Vector getEndPoint() {
    return this.endPoint;
  }

  public Geometry getGeometry() {
    return this.geometry;
  }

  public Double getLength() {
    if (this.geometry == null) {
      return null;
    }
    return this.geometry.getLength();
  }

  public Vector getStartPoint() {
    return this.startPoint;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime
            * result
            + ((this.geometry == null) ? 0 : this.geometry.hashCode());
    return result;
  }

  public boolean hasReverse() {
    return this.hasReverse;
  }

  public boolean isNullEdge() {
    return this == InferenceGraphEdge.emptyEdge;
  }

  @Override
  public String toString() {
    if (this == InferenceGraphEdge.emptyEdge) {
      return "InferredEdge [null]";
    } else {
      return "InferredEdge [edgeId=" + this.edgeId + ", length="
          + this.getLength() + "]";
    }
  }

}