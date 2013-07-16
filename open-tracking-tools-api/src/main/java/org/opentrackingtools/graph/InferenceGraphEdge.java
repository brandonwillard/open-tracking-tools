package org.opentrackingtools.graph;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;

import java.util.Collections;
import java.util.List;

import javax.annotation.Nonnull;

import org.apache.commons.lang3.builder.ToStringBuilder;
import org.opentrackingtools.util.GeoUtils;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.linearref.LengthLocationMap;
import com.vividsolutions.jts.linearref.LocationIndexedLine;

public class InferenceGraphEdge implements
    Comparable<InferenceGraphEdge> {

  /*
   * This is the empty edge, which stands for free movement
   */
  public final static InferenceGraphEdge nullGraphEdge =
      new InferenceGraphEdge();

  protected final Object backingEdge;
  protected final Integer edgeId;
  protected final Geometry geometry;
  protected final Boolean hasReverse;
  
  protected List<InferenceGraphSegment> graphSegments;
  protected LengthLocationMap lengthLocationMap = null;
  protected LocationIndexedLine locationIndexedLine;

  protected InferenceGraphEdge() {
    this.locationIndexedLine = null;
    this.graphSegments = null;
    this.edgeId = null;
    this.backingEdge = null;
    this.geometry = null;
    this.hasReverse = null;
  }

  public InferenceGraphEdge(@Nonnull Geometry geom,
    @Nonnull Object backingEdge, @Nonnull Integer edgeId,
    @Nonnull InferenceGraph graph) {

    this.edgeId = Preconditions.checkNotNull(edgeId);
    this.backingEdge = Preconditions.checkNotNull(backingEdge);

    this.geometry = Preconditions.checkNotNull(geom);

    this.hasReverse = graph.edgeHasReverse(geom);

    final Coordinate startPointCoord =
        this.geometry.getCoordinates()[0];

    final Coordinate endPointCoord =
        this.geometry.getCoordinates()[this.geometry.getNumPoints() - 1];

    this.locationIndexedLine = new LocationIndexedLine(this.geometry);
    this.graphSegments = Lists.newArrayList();
    final List<LineSegment> segments =
        GeoUtils.getSubLineSegments((LineString) this.geometry);
    InferenceGraphSegment nextSegment = null;
    for (final LineSegment segment : Lists.reverse(segments)) {
      final InferenceGraphSegment infSegment =
          new InferenceGraphSegment(segment, this, nextSegment);
      nextSegment = infSegment;
      this.graphSegments.add(infSegment);
    }
    Collections.reverse(this.graphSegments);
  }

  public InferenceGraphEdge(InferenceGraphEdge infEdge) {
    this.backingEdge = infEdge.backingEdge;
    this.edgeId = infEdge.edgeId;
    this.geometry = infEdge.geometry;
    this.hasReverse = infEdge.hasReverse;
    this.graphSegments = infEdge.graphSegments;
    this.lengthLocationMap = infEdge.lengthLocationMap;
    this.locationIndexedLine = infEdge.locationIndexedLine;
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

  public Geometry getGeometry() {
    return this.geometry;
  }

  public Double getLength() {
    if (this.geometry == null) {
      return null;
    }
    return this.geometry.getLength();
  }

  public LengthLocationMap getLengthLocationMap() {
    if (this.lengthLocationMap == null) {
      this.lengthLocationMap = new LengthLocationMap(this.geometry);
    }
    return this.lengthLocationMap;
  }

  public LocationIndexedLine getLocationIndexedLine() {
    return this.locationIndexedLine;
  }

  public List<InferenceGraphSegment> getSegments() {
    return this.graphSegments;
  }

  public List<InferenceGraphSegment> getSegments(double upToLength) {
    return this.getSegments(0d, upToLength);
  }

  /**
   * Get segments with distances that overlap the given interval (inclusive).
   * 
   * @param lengthStart
   * @param lengthEnd
   * @return
   */
  public List<InferenceGraphSegment> getSegments(double lengthStart,
    double lengthEnd) {
    Preconditions.checkState(lengthStart <= lengthEnd);
    final List<InferenceGraphSegment> results = Lists.newArrayList();
    for (final InferenceGraphSegment segment : this.graphSegments) {
      if (segment.startDistance >= lengthStart) {
        results.add(segment);
      }
      if (segment.startDistance + segment.line.getLength() > lengthEnd) {
        break;
      }
    }
    return results;
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
    return this.equals(InferenceGraphEdge.nullGraphEdge);
  }

  @Override
  public String toString() {
    final ToStringBuilder builder = new ToStringBuilder(this);
    builder.append("edgeId", this.getEdgeId());
    builder.append("length", this.getLength());
    return builder.toString();
  }

}