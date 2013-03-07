package org.opentrackingtools.paths;

import java.text.NumberFormat;
import java.util.Iterator;

import gov.sandia.cognition.math.matrix.AbstractVector;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorEntry;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.util.ObjectUtil;

import org.apache.commons.lang3.builder.CompareToBuilder;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.util.PathUtils;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;

public class PathState 
    extends AbstractVector implements Comparable<PathState>, Cloneable {

  private static final long serialVersionUID = 2846671162796173049L;

  protected PathEdge<?> edge;

  private Vector globalState;

  private Vector groundState;

  private Vector localState;

  protected Path path;

  private Vector rawState;

  public PathState(PathEdge<?> pathEdge, Vector stateSample) {
    this.edge = pathEdge;
    this.globalState = stateSample;
  }

  public PathState(Path path, Vector state) {
    
    Preconditions.checkArgument(!path.isNullPath()
        || state.getDimensionality() == 4);

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

  public PathState(PathState pathState) {
    this.edge = pathState.edge;
    this.globalState = pathState.globalState;
    this.groundState = pathState.groundState;
    this.localState = pathState.localState;
    this.path = pathState.path;
    this.rawState = pathState.rawState;
  }

  @Override
  public double angle(Vector other) {
    return globalState.angle(other);
  }

  @Override
  public void assertDimensionalityEquals(int otherDimensionality) {
    globalState.assertDimensionalityEquals(otherDimensionality);
  }

  @Override
  public void assertSameDimensionality(Vector other) {
    globalState.assertSameDimensionality(other);
  }

  @Override
  public boolean checkSameDimensionality(Vector other) {
    return globalState.checkSameDimensionality(other);
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

  @Override
  public void convertFromVector(Vector parameters) {
    globalState.convertFromVector(parameters);
  }

  @Override
  public Vector convertToVector() {
    return globalState.convertToVector();
  }

  @Override
  public double cosine(Vector other) {
    return globalState.cosine(other);
  }

  @Override
  public double dotProduct(Vector other) {
    return globalState.dotProduct(other);
  }

  @Override
  public Vector dotTimes(Vector other) {
    return globalState.dotTimes(other);
  }

  @Override
  public void dotTimesEquals(Vector other) {
    groundState = null;
    globalState.dotTimesEquals(other);
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

  @Override
  public boolean equals(Vector other, double effectiveZero) {
    return globalState.equals(other, effectiveZero);
  }

  @Override
  public double euclideanDistance(Vector other) {
    return globalState.euclideanDistance(other);
  }

  @Override
  public double euclideanDistanceSquared(Vector other) {
    return globalState.euclideanDistanceSquared(other);
  }

  @Override
  public int getDimensionality() {
    return globalState.getDimensionality();
  }

  public PathEdge<?> getEdge() {

    if (!isOnRoad())
      return Iterables.getOnlyElement(this.path.getPathEdges());

    if (edge == null) {
      Preconditions.checkState(!path.isNullPath());
      this.edge =
          path.getEdgeForDistance(globalState.getElement(0), false);
    }
    return Preconditions.checkNotNull(this.edge);
  }

  @Override
  public double getElement(int index) {
    return globalState.getElement(index);
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
              MotionStateEstimatorPredictor
                  .getEdgeLengthErrorTolerance(), true);
    }
    return this.localState;
  }

  public Path getPath() {
    return path;
  }

  public Vector getRawState() {
    return rawState;
  }

  public PathState getTruncatedPathState() {

    if (!this.isOnRoad())
      return this;

    final Path newPath = this.path.getPathTo(this.getEdge());

    return new PathState(newPath, this.rawState);
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

  public boolean isOnRoad() {
    return !this.path.isNullPath();
  }

  @Override
  public boolean isUnitVector() {
    return globalState.isUnitVector();
  }

  @Override
  public boolean isUnitVector(double tolerance) {
    return globalState.isUnitVector(tolerance);
  }

  @Override
  public boolean isZero() {
    return globalState.isZero();
  }

  @Override
  public boolean isZero(double effectiveZero) {
    return globalState.isZero(effectiveZero);
  }

  @Override
  public Iterator<VectorEntry> iterator() {
    return globalState.iterator();
  }

  public Vector minus(PathState currentState) {
    return PathUtils.stateDiff(currentState, this, false);
  }

  @Override
  public Vector minus(Vector other) {
    return globalState.minus(other);
  }

  @Override
  public void minusEquals(Vector other) {
    groundState = null;
    globalState.minusEquals(other);
  }

  @Override
  public Vector negative() {
    return globalState.negative();
  }

  @Override
  public void negativeEquals() {
    groundState = null;
    globalState.negativeEquals();
  }

  @Override
  public double norm(double power) {
    return globalState.norm(power);
  }

  @Override
  public double norm1() {
    return globalState.norm1();
  }

  @Override
  public double norm2() {
    return globalState.norm2();
  }
  @Override
  public double norm2Squared() {
    return globalState.norm2Squared();
  }
  @Override
  public double normInfinity() {
    return globalState.normInfinity();
  }
  @Override
  public Matrix outerProduct(Vector other) {
    return globalState.outerProduct(other);
  }
  @Override
  public Vector plus(Vector other) {
    return globalState.plus(other);
  }
  @Override
  public void plusEquals(Vector other) {
    groundState = null;
    globalState.plusEquals(other);
  }
  @Override
  public Vector scale(double scaleFactor) {
    return globalState.scale(scaleFactor);
  }

  @Override
  public Vector scaledMinus(double scaleFactor, Vector other) {
    groundState = null;
    return globalState.scaledMinus(scaleFactor, other);
  }

  @Override
  public void scaledMinusEquals(double scaleFactor, Vector other) {
    groundState = null;
    globalState.scaledMinusEquals(scaleFactor, other);
  }

  @Override
  public Vector scaledPlus(double scaleFactor, Vector other) {
    groundState = null;
    return globalState.scaledPlus(scaleFactor, other);
  }

  @Override
  public void scaledPlusEquals(double scaleFactor, Vector other) {
    groundState = null;
    globalState.scaledPlusEquals(scaleFactor, other);
  }

  @Override
  public void scaleEquals(double scaleFactor) {
    groundState = null;
    globalState.scaleEquals(scaleFactor);
  }

  @Override
  public void setElement(int index, double value) {
    globalState.setElement(index, value);
  }

  @Override
  public Vector stack(Vector other) {
    return globalState.stack(other);
  }

  @Override
  public Vector subVector(int minIndex, int maxIndex) {
    return globalState.subVector(minIndex, maxIndex);
  }

  @Override
  public double sum() {
    return globalState.sum();
  }

  @Override
  public Vector times(Matrix matrix) {
    return globalState.times(matrix);
  }

  @Override
  public double[] toArray() {
    return globalState.toArray();
  }

  @Override
  public String toString() {
    final StringBuilder builder = new StringBuilder();
    builder.append("PathState [path=").append(path)
        .append(", state=").append(globalState).append("]");
    return builder.toString();
  }

  @Override
  public String toString(NumberFormat format) {
    return globalState.toString(format);
  }

  @Override
  public String toString(NumberFormat format, String delimiter) {
    return globalState.toString(format, delimiter);
  }

  @Override
  public Vector unitVector() {
    return globalState.unitVector();
  }

  @Override
  public void unitVectorEquals() {
    globalState.unitVectorEquals();
  }

  @Override
  public void zero() {
    globalState.zero();
  }

  public PathState getRelatableState(PathState currentState) {

    final Path path = this.path;
    final Vector adjState;
    if (path.isNullPath()) {
      adjState = currentState.getGroundState();
    } else {

      final PathState startState =
          new PathState(path, VectorFactory.getDefault()
              .createVector2D(0d, 0d));

      final Vector diff = startState.minus(currentState);
      diff.negativeEquals();

      adjState =
          VectorFactory.getDefault().createVector2D(
              path.clampToPath(diff.getElement(0)),
              diff.getElement(1));

      assert (path.isOnPath(adjState.getElement(0)));
      assert !currentState.isOnRoad()
          || Preconditions.checkNotNull(new PathState(path, adjState)
              .minus(currentState)
              .isZero(
                  MotionStateEstimatorPredictor
                      .getEdgeLengthErrorTolerance()) ? Boolean.TRUE
              : null);
    }

    PathState result = new PathState(path, adjState);

    return result;
  }

}
