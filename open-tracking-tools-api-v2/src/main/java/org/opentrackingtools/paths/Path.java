package org.opentrackingtools.paths;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.opentrackingtools.distributions.BayesianEstimableDistribution;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.estimators.RecursiveBayesianEstimatorPredictor;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.PathUtils.PathEdgeProjection;
import org.opentrackingtools.util.model.WrappedWeightedValue;

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableList;
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
  protected ImmutableList<? extends PathEdge<?>> edges;
  protected Double totalPathDistance;

  public List<String> edgeIds = Lists.newArrayList();

  /*
   * Note: single edges are considered forward
   */
  protected Boolean isBackward = null;

  protected Geometry geometry;

  protected Path() {
    this.edges = null;
    this.totalPathDistance = null;
    this.isBackward = null;
    this.geometry = null;
  }

  protected Path(ImmutableList<? extends PathEdge<?>> edges, Boolean isBackward) {
    Preconditions.checkArgument(edges.size() > 0);
    Preconditions.checkState(Iterables.getFirst(edges, null)
        .getDistToStartOfEdge() == 0d);
    this.edges = edges;
    this.isBackward = isBackward;

    PathEdge<?> lastEdge = null;
    //    double absTotalDistance = 0d;
    final List<Coordinate> coords = Lists.newArrayList();
    for (final PathEdge<?> edge : edges) {

      if (!edge.isNullEdge()) {
        if (isBackward) {
          Preconditions.checkArgument(lastEdge == null
              || lastEdge.getInferredEdge().getStartPoint()
                  .equals(edge.getInferredEdge().getEndPoint()));
        } else {
          Preconditions.checkArgument(lastEdge == null
              || lastEdge.getInferredEdge().getEndPoint()
                  .equals(edge.getInferredEdge().getStartPoint()));

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
                    .equals(coords.get(coords.size() - 1)))
              coords.add(theseCoords[i]);
          }
          edgeIds.add(edge.getInferredEdge().getEdgeId());
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
  protected Path(PathEdge<?> edge) {
    // TODO FIXME remove specific PathEdge type.
    this.edges = ImmutableList.of(edge);
    Preconditions.checkArgument(edge.getDistToStartOfEdge() == null
        || edge.getDistToStartOfEdge() == 0d);

    this.isBackward = edge.isBackward();
    if (edge.getInferredEdge().isNullEdge())
      this.totalPathDistance = null;
    else
      this.totalPathDistance =
          (this.isBackward == Boolean.TRUE ? -1d : 1d)
              * edge.getInferredEdge().getLength();
    this.edgeIds.add(edge.getInferredEdge().getEdgeId());
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
    if (getClass() != obj.getClass()) {
      return false;
    }
    final Path other = (Path) obj;
    if (edges == null) {
      if (other.edges != null) {
        return false;
      }
    } else if (!edges.equals(other.edges)) {
      return false;
    }
    return true;
  }

  public PathEdge<?> getEdgeForDistance(double distance, boolean clamp) {
    final double direction = Math.signum(totalPathDistance);
    if (direction * distance - Math.abs(totalPathDistance) > MotionStateEstimatorPredictor
        .getEdgeLengthErrorTolerance()) {
      return clamp ? Iterables.getLast(edges) : null;
    } else if (direction * distance < 0d) {
      return clamp ? Iterables.getFirst(edges, null) : null;
    }

    for (final PathEdge<?> edge : edges.reverse()) {
      if (edge.isOnEdge(distance))
        return edge;
    }

    assert Preconditions.checkNotNull(null);

    return null;
  }

  public List<String> getEdgeIds() {
    return edgeIds;
  }

  public ImmutableList<? extends PathEdge<?>> getPathEdges() {
    return edges;
  }

  public Geometry getGeometry() {
    return geometry;
  }

  public Boolean isBackward() {
    return isBackward;
  }



  public Double getTotalPathDistance() {
    return totalPathDistance;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime * result + ((edges == null) ? 0 : edges.hashCode());
    return result;
  }

  public boolean isNullPath() {
    return this.edges.isEmpty();
  }

  public boolean isOnPath(double distance) {

    Preconditions.checkState(!isNullPath());

    final double direction = Math.signum(totalPathDistance);
    final double overTheEndDist =
        direction * distance - Math.abs(totalPathDistance);
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
    if (this.isNullPath())
      return "SimpleInferredPath [null path]";
    else
      return "SimpleInferredPath [edges=" + edgeIds
          + ", totalPathDistance=" + totalPathDistance + "]";
  }

  public Path getPathTo(PathEdge<?> edge) {

    final List<PathEdge<?>> newEdges = Lists.newArrayList();
    for (PathEdge<?> edge1 : this.getPathEdges()) {
      newEdges.add(edge1);
      if (edge1.equals(edge)) {
        break;
      }
    }

    Path newPath =
        new Path(ImmutableList.copyOf(newEdges), this.isBackward);

    return newPath;
  }

}
