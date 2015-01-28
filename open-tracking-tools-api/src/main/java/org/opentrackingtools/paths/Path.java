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
import com.vividsolutions.jts.geom.CoordinateArrays;
import com.vividsolutions.jts.geom.CoordinateList;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineSegment;
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

  public static final CoordinateArrays.BidirectionalComparator biDirComp =
      new CoordinateArrays.BidirectionalComparator();

  public final static Path nullPath = new Path();

  private static final long serialVersionUID = -113041668509555507L;

  public List<String> edgeIds = null;

  protected List<? extends PathEdge> edges = null;

  protected Geometry geometry = null;

  protected Boolean isBackward = null;

  protected Double totalPathDistance = null;

  protected Path() {
    this.edges = Collections.singletonList(PathEdge.nullPathEdge);
    this.totalPathDistance = null;
    this.isBackward = null;
    this.geometry = null;
  }

  public Path(List<? extends PathEdge> edges, Boolean isBackward) {
    Preconditions.checkArgument(edges.size() > 0);
    Preconditions.checkArgument(!Iterables.getFirst(edges, null)
        .isNullEdge());
    Preconditions.checkState(Iterables.getFirst(edges, null)
        .getDistToStartOfEdge() == 0d);

    this.edges = edges;
    this.isBackward = isBackward;
    this.edgeIds = Lists.newArrayList();

    PathEdge lastEdge = null;
    final CoordinateList coords = new CoordinateList();
    for (final PathEdge edge : edges) {

      if (lastEdge != null && !edge.equals(lastEdge)) {
        Preconditions.checkArgument(lastEdge == null
            || lastEdge.line.p1.equals(edge.line.p0));

      }

      final LineSegment geom = edge.getLine();
      if (geom.getLength() > 1e-4) {
        coords.add(geom.p0, false);
        coords.add(geom.p1, false);
        if (!edge.equals(lastEdge)) {
          this.edgeIds.add(edge.getInferenceGraphSegment().getEdgeId());
        }
      }

      lastEdge = edge;
    }

    this.geometry =
        JTSFactoryFinder.getGeometryFactory().createLineString(
            coords.toCoordinateArray());

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
            * edge.getLength();
    this.edgeIds =
        Lists.newArrayList(edge.getInferenceGraphSegment().getEdgeId());
    this.geometry =
        edge.getLine().toGeometry(
            JTSFactoryFinder.getGeometryFactory());

    if (this.isBackward == Boolean.TRUE) {
      this.geometry = this.geometry.reverse();
    }
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

  public PathEdge getEdgeForDistance(double distance, boolean clamp) {
    final double direction = Math.signum(this.totalPathDistance);
    final double distDiff = direction * distance - Math.abs(this.totalPathDistance);
    if (clamp) {
      if (distDiff > MotionStateEstimatorPredictor.getEdgeLengthErrorTolerance()
          || Math.abs(distDiff) <= 1e-5) {
        return Iterables.getLast(this.edges);
      } else if (direction * distance < 0d) {
        return Iterables.getFirst(this.edges, null);
      }
    } 

//    if (Math.abs(distDiff) <= 1e-5) {
//      return Iterables.getLast(this.edges);
//    }

    for (final PathEdge edge : Lists.reverse(this.edges)) {
      if (edge.isOnEdge(distance)) {
        return edge;
      }
    }

    //    assert Preconditions.checkNotNull(null);

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

  public Path getPathFrom(PathEdge edge) {

    final List<PathEdge> newEdges = Lists.newArrayList();
    double distance = 0d;
    for (final PathEdge edge1 : this.getPathEdges()) {
      if (edge1.getDistToStartOfEdge() >= edge.getDistToStartOfEdge()) {
        newEdges.add(new PathEdge(edge1.getInferenceGraphSegment(), distance, edge1
            .isBackward()));
        distance += edge1.getLength();
      }

    }

    final Path newPath = new Path(newEdges, this.isBackward);

    return newPath;
  }

  public Path getPathTo(double distance) {

    final List<PathEdge> newEdges = Lists.newArrayList();
    final double direction = this.isBackward ? -1d : 1d;
    for (final PathEdge edge : this.getPathEdges()) {
      newEdges.add(edge);
      if (direction * distance <= Math.abs(edge.distToStartOfEdge)
          + edge.getLength()) {
        break;
      }
    }

    final Path newPath = new Path(newEdges, this.isBackward);

    return newPath;
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
    return this.equals(Path.nullPath);
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
      return "Path [edges=" + this.edgeIds + ", totalPathDistance="
          + this.totalPathDistance + "]";
    }
  }

}
