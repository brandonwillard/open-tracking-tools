package org.opentrackingtools.paths;

import java.util.ArrayList;
import java.util.Random;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.util.AbstractCloneableSerializable;
import gov.sandia.cognition.util.ObjectUtil;

import javax.annotation.Nonnull;

import org.apache.commons.lang3.builder.CompareToBuilder;
import org.opentrackingtools.distributions.BayesianEstimableDistribution;
import org.opentrackingtools.estimators.AbstractRoadTrackingFilter;
import org.opentrackingtools.estimators.RecursiveBayesianEstimatorPredictor;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.util.PathUtils;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;

public class PathState extends AbstractCloneableSerializable
    implements Comparable<PathState>, Cloneable, 
      BayesianEstimableDistribution<PathState, PathState> {

  private static final long serialVersionUID = 2846671162796173049L;
  private Vector globalState;
  private Vector groundState;
  private Vector localState;
  private Vector rawState;
  protected Path path;
  protected PathEdge edge;

  public Path getPath() {
    return path;
  }

  public boolean isOnRoad() {
    return !this.path.isNullPath();
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((path == null) ? 0 : path.hashCode());
    result =
        prime
            * result
            + ((getGlobalState() == null) ? 0 : getGlobalState()
                .hashCode());
    result =
        prime
            * result
            + ((getRawState() == null) ? 0 : getRawState().hashCode());
    return result;
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
    final PathState other = (PathState) obj;
    if (path == null) {
      if (other.path != null) {
        return false;
      }
    } else if (!path.equals(other.path)) {
      return false;
    }

    if (getGlobalState() == null) {
      if (other.getGlobalState() != null) {
        return false;
      }
    } else if (!getGlobalState().equals((other.getGlobalState()))) {
      return false;
    }

    if (getRawState() == null) {
      if (other.getRawState() != null) {
        return false;
      }
    } else if (!getRawState().equals((other.getRawState()))) {
      return false;
    }
    return true;
  }

  protected PathState(Path path, Vector state) {

    this.path = path;
    this.rawState = state.clone();
    this.globalState = state.clone();
    /*
     * Now make sure the result is on this path.
     */
    if (!path.isNullPath()) {
      this.globalState.setElement(0,
          path.clampToPath(this.globalState.getElement(0)));
    }
  }

  @Override
  public PathState clone() {
    final PathState clone = (PathState) super.clone();
    clone.path = ObjectUtil.cloneSmart(this.path);
    /*
     * The edge should always refer to the edge in this path,
     * so if the path gets cloned and that clone makes a clone
     * of the path edges, then we need to find the new edge.
     */
    clone.edge = null;
    clone.rawState = ObjectUtil.cloneSmart(this.rawState);
    clone.localState = ObjectUtil.cloneSmart(this.localState);
    clone.globalState = ObjectUtil.cloneSmart(this.globalState);
    clone.groundState = ObjectUtil.cloneSmart(this.groundState);
    return clone;
  }

  @Override
  public int compareTo(PathState o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(this.path, o.getPath());
    comparator.append(this.globalState, o.getGlobalState());
    comparator.append(this.rawState, o.getRawState());
    return comparator.toComparison();
  }

  public PathEdge getEdge() {

    if (!isOnRoad())
      return Iterables.getOnlyElement(this.path.getPathEdges());

    if (edge == null) {
      Preconditions.checkState(!path.isNullPath());
      this.edge =
          path.getEdgeForDistance(globalState.getElement(0), false);
    }
    return Preconditions.checkNotNull(this.edge);
  }

  public Vector getGlobalState() {
    return globalState;
  }

  public Vector getGroundState() {
    if (!this.isOnRoad())
      return this.globalState;

    if (this.groundState == null) {
      this.groundState =
          PathUtils.getGroundStateFromRoad(this.globalState,
              this.getEdge(), true);
    }

    return this.groundState;
  }

  public Vector getLocalState() {
    if (this.localState != null)
      return this.localState;
    if (this.path.isNullPath()) {
      this.localState = this.globalState;
    } else {
      this.localState =
          this.getEdge().getCheckedStateOnEdge(
              this.globalState,
              AbstractRoadTrackingFilter
                  .getEdgeLengthErrorTolerance(), true);
    }
    return this.localState;
  }

  public Vector getRawState() {
    return rawState;
  }

  @Override
  public String toString() {
    final StringBuilder builder = new StringBuilder();
    builder.append("PathState [path=").append(path)
        .append(", state=").append(globalState).append("]");
    return builder.toString();
  }

  public static PathState getPathState(@Nonnull Path path,
    @Nonnull Vector state) {
    Preconditions.checkArgument(!path.isNullPath()
        || state.getDimensionality() == 4);

    final Vector adjState =
        PathUtils.checkAndGetConvertedState(state, path);

    return new PathState(path, adjState);
  }

  public PathState getTruncatedPathState() {

    if (!this.isOnRoad())
      return this;

    final Path newPath = this.path.getPathTo(this.getEdge());

    return new PathState(newPath, this.rawState);
  }

  public Vector minus(PathState currentState) {
    return PathUtils.stateDiff(currentState, this, false);
  }

  @Override
  public ProbabilityFunction<PathState> getProbabilityFunction() {
  }

  @Override
  public PathState sample(Random random) {
  }

  @Override
  public ArrayList<? extends PathState> sample(Random random,
    int numSamples) {
  }

  @Override
  public RecursiveBayesianEstimatorPredictor<PathState, PathState>
      getBayesianEstimatorPredictor() {
  }

}
