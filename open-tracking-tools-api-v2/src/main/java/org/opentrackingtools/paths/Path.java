package org.opentrackingtools.paths;

import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Collections;
import java.util.List;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.linearref.LengthIndexedLine;

/**
 * Inferred paths are collections of PathEdges that track the distance traveled
 * and the direction (by sign)
 * 
 * @author bwillard
 * 
 */
public class Path extends AbstractCloneableSerializable implements
    Comparable<Path> {

  private static final long serialVersionUID = -113041668509555507L;
  
  public List<String> edgeIds = null;
  
  protected List<? extends PathEdge> edges = null;

  protected Geometry geometry = null;

  protected Boolean isBackward = null;

  protected Double totalPathDistance = null;
  
  public final static Path nullPath = new Path();

  protected Path() {
    this.edges = Collections.singletonList(PathEdge.nullPathEdge);
    this.totalPathDistance = null;
    this.isBackward = null;
    this.geometry = null;
  }

  public Path(List<? extends PathEdge> edges, Boolean isBackward) {
    Preconditions.checkArgument(edges.size() > 0);
    Preconditions.checkArgument(!Iterables.getFirst(edges, null).isNullEdge());
    Preconditions.checkState(Iterables.getFirst(edges, null).getDistToStartOfEdge() == 0d);
    
    this.edges = edges;
    this.isBackward = isBackward;
    this.edgeIds = Lists.newArrayList();

    PathEdge lastEdge = null;
    final List<Coordinate> coords = Lists.newArrayList();
    for (final PathEdge edge : edges) {

      if (!edge.isNullEdge()) {
        if (isBackward) {
          Preconditions.checkArgument(lastEdge == null
              || lastEdge.getInferenceGraphEdge().getStartPoint()
                  .equals(edge.getInferenceGraphEdge().getEndPoint()));
        } else {
          Preconditions.checkArgument(lastEdge == null
              || lastEdge.getInferenceGraphEdge().getEndPoint()
                  .equals(edge.getInferenceGraphEdge().getStartPoint()));

        }

        final Geometry geom = edge.getGeometry();
        if (geom.getLength() > 1e-4) {
          final Coordinate[] theseCoords =
              isBackward ? geom.reverse().getCoordinates() : geom
                  .getCoordinates();
          final int startIdx = coords.size() == 0 ? 0 : 1;
          for (int i = startIdx; i < theseCoords.length; i++) {
            if (i == 0
                || !theseCoords[i]
                    .equals(coords.get(coords.size() - 1))) {
              coords.add(theseCoords[i]);
            }
          }
          this.edgeIds.add(edge.getInferenceGraphEdge().getEdgeId());
        }
      }

      lastEdge = edge;
    }

    if (edges.size() > 1) {
      this.geometry =
          JTSFactoryFinder.getGeometryFactory().createLineString(
              coords.toArray(new Coordinate[coords.size()]));
    } else {
      final Geometry edgeGeom =
          Iterables.getOnlyElement(edges).getGeometry();
      this.geometry = isBackward ? edgeGeom.reverse() : edgeGeom;
    }

    final double direction = isBackward ? -1d : 1d;
    this.totalPathDistance = direction * this.geometry.getLength();
  }

  /**
   * Produce a path starting at this edge.
   * 
   * @param edge
   */
  public Path(PathEdge edge) {
    Preconditions.checkArgument(!edge.isNullEdge());
    Preconditions.checkArgument(edge.getDistToStartOfEdge() == null
        || edge.getDistToStartOfEdge() == 0d);
    this.isBackward = edge.isBackward();
    this.edges = Collections.singletonList(edge);
    this.totalPathDistance =
        (this.isBackward == Boolean.TRUE ? -1d : 1d)
            * edge.getInferenceGraphEdge().getLength();
    this.edgeIds = Lists.newArrayList(edge.getInferenceGraphEdge().getEdgeId());
    this.geometry =
        (this.isBackward == Boolean.TRUE) ? edge.getGeometry()
            .reverse() : edge.getGeometry();
  }

  public double clampToPath(final double distance) {
    final double dir = this.isBackward() ? -1d : 1d;
    final LengthIndexedLine lil =
        new LengthIndexedLine(this.getGeometry());
    final double clampedIndex = dir * lil.clampIndex(dir * distance);
    return clampedIndex;
  }

  @Override
  public int compareTo(Path o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(this.edges.toArray(), o.getPathEdges()
        .toArray());
    return comparator.toComparison();
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
    final Path other = (Path) obj;
    if (this.edges == null) {
      if (other.edges != null) {
        return false;
      }
    } else if (!this.edges.equals(other.edges)) {
      return false;
    }
    return true;
  }

  public PathEdge
      getEdgeForDistance(double distance, boolean clamp) {
    final double direction = Math.signum(this.totalPathDistance);
    if (direction * distance - Math.abs(this.totalPathDistance) > MotionStateEstimatorPredictor
        .getEdgeLengthErrorTolerance()) {
      return clamp ? Iterables.getLast(this.edges) : null;
    } else if (direction * distance < 0d) {
      return clamp ? Iterables.getFirst(this.edges, null) : null;
    }

    for (final PathEdge edge : Lists.reverse(this.edges)) {
      if (edge.isOnEdge(distance)) {
        return edge;
      }
    }

    assert Preconditions.checkNotNull(null);

    return null;
  }

  public List<String> getEdgeIds() {
    return this.edgeIds;
  }

  public Geometry getGeometry() {
    return this.geometry;
  }

  public List<? extends PathEdge> getPathEdges() {
    return this.edges;
  }

  public Path getPathTo(PathEdge edge) {

    final List<PathEdge> newEdges = Lists.newArrayList();
    for (final PathEdge edge1 : this.getPathEdges()) {
      newEdges.add(edge1);
      if (edge1.equals(edge)) {
        break;
      }
    }

    final Path newPath = new Path(newEdges, this.isBackward);

    return newPath;
  }

  public Double getTotalPathDistance() {
    return this.totalPathDistance;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime * result
            + ((this.edges == null) ? 0 : this.edges.hashCode());
    return result;
  }

  public Boolean isBackward() {
    return this.isBackward;
  }

  public boolean isNullPath() {
    return this.equals(nullPath);
  }

  public boolean isOnPath(double distance) {

    Preconditions.checkState(!this.isNullPath());

    final double direction = Math.signum(this.totalPathDistance);
    final double overTheEndDist =
        direction * distance - Math.abs(this.totalPathDistance);
    if (overTheEndDist > MotionStateEstimatorPredictor
        .getEdgeLengthErrorTolerance()) {
      return false;
    } else if (direction * distance < -MotionStateEstimatorPredictor
        .getEdgeLengthErrorTolerance()) {
      return false;
    }

    return true;
  }

  @Override
  public String toString() {
    if (this.isNullPath()) {
      return "Path [null path]";
    } else {
      return "Path [edges=" + this.edgeIds
          + ", totalPathDistance=" + this.totalPathDistance + "]";
    }
  }

}
