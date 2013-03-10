package org.opentrackingtools.paths;

import gov.sandia.cognition.math.matrix.AbstractVector;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorEntry;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.util.ObjectUtil;

import java.text.NumberFormat;
import java.util.Iterator;

import org.apache.commons.lang3.builder.CompareToBuilder;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.util.PathUtils;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;

public class PathState extends AbstractVector implements
    Comparable<PathState>, Cloneable {

  private static final long serialVersionUID = 2846671162796173049L;

  protected PathEdge<?> edge;

  private Vector globalState;

  private Vector groundState;

  private Vector localState;

  protected Path path;

  private Vector rawState;

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

  public PathState(PathEdge<?> pathEdge, Vector stateSample) {
    this.edge = pathEdge;
    this.globalState = stateSample;
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
    return this.globalState.angle(other);
  }

  @Override
  public void assertDimensionalityEquals(int otherDimensionality) {
    this.globalState.assertDimensionalityEquals(otherDimensionality);
  }

  @Override
  public void assertSameDimensionality(Vector other) {
    this.globalState.assertSameDimensionality(other);
  }

  @Override
  public boolean checkSameDimensionality(Vector other) {
    return this.globalState.checkSameDimensionality(other);
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
    this.globalState.convertFromVector(parameters);
  }

  @Override
  public Vector convertToVector() {
    return this.globalState.convertToVector();
  }

  @Override
  public double cosine(Vector other) {
    return this.globalState.cosine(other);
  }

  @Override
  public double dotProduct(Vector other) {
    return this.globalState.dotProduct(other);
  }

  @Override
  public Vector dotTimes(Vector other) {
    return this.globalState.dotTimes(other);
  }

  @Override
  public void dotTimesEquals(Vector other) {
    this.groundState = null;
    this.globalState.dotTimesEquals(other);
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
    final PathState other = (PathState) obj;
    if (this.path == null) {
      if (other.path != null) {
        return false;
      }
    } else if (!this.path.equals(other.path)) {
      return false;
    }

    if (this.getGlobalState() == null) {
      if (other.getGlobalState() != null) {
        return false;
      }
    } else if (!this.getGlobalState()
        .equals((other.getGlobalState()))) {
      return false;
    }

    if (this.getRawState() == null) {
      if (other.getRawState() != null) {
        return false;
      }
    } else if (!this.getRawState().equals((other.getRawState()))) {
      return false;
    }
    return true;
  }

  @Override
  public boolean equals(Vector other, double effectiveZero) {
    return this.globalState.equals(other, effectiveZero);
  }

  @Override
  public double euclideanDistance(Vector other) {
    return this.globalState.euclideanDistance(other);
  }

  @Override
  public double euclideanDistanceSquared(Vector other) {
    return this.globalState.euclideanDistanceSquared(other);
  }

  @Override
  public int getDimensionality() {
    return this.globalState.getDimensionality();
  }

  public PathEdge<?> getEdge() {

    if (!this.isOnRoad()) {
      return Iterables.getOnlyElement(this.path.getPathEdges());
    }

    if (this.edge == null) {
      Preconditions.checkState(!this.path.isNullPath());
      this.edge =
          this.path.getEdgeForDistance(
              this.globalState.getElement(0), false);
    }
    return Preconditions.checkNotNull(this.edge);
  }

  @Override
  public double getElement(int index) {
    return this.globalState.getElement(index);
  }

  public Vector getGlobalState() {
    return this.globalState;
  }

  public Vector getGroundState() {
    if (!this.isOnRoad()) {
      return this.globalState;
    }

    if (this.groundState == null) {
      this.groundState =
          PathUtils.getGroundStateFromRoad(this.globalState,
              this.getEdge(), true);
    }

    return this.groundState;
  }

  public Vector getLocalState() {
    if (this.localState != null) {
      return this.localState;
    }
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
    return this.path;
  }

  public Vector getRawState() {
    return this.rawState;
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
              .minus(currentState).isZero(
                  MotionStateEstimatorPredictor
                      .getEdgeLengthErrorTolerance()) ? Boolean.TRUE
              : null);
    }

    final PathState result = new PathState(path, adjState);

    return result;
  }

  public PathState getTruncatedPathState() {

    if (!this.isOnRoad()) {
      return this;
    }

    final Path newPath = this.path.getPathTo(this.getEdge());

    return new PathState(newPath, this.rawState);
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime * result
            + ((this.path == null) ? 0 : this.path.hashCode());
    result =
        prime
            * result
            + ((this.getGlobalState() == null) ? 0 : this
                .getGlobalState().hashCode());
    result =
        prime
            * result
            + ((this.getRawState() == null) ? 0 : this.getRawState()
                .hashCode());
    return result;
  }

  public boolean isOnRoad() {
    return !this.path.isNullPath();
  }

  @Override
  public boolean isUnitVector() {
    return this.globalState.isUnitVector();
  }

  @Override
  public boolean isUnitVector(double tolerance) {
    return this.globalState.isUnitVector(tolerance);
  }

  @Override
  public boolean isZero() {
    return this.globalState.isZero();
  }

  @Override
  public boolean isZero(double effectiveZero) {
    return this.globalState.isZero(effectiveZero);
  }

  @Override
  public Iterator<VectorEntry> iterator() {
    return this.globalState.iterator();
  }

  public Vector minus(PathState currentState) {
    return PathUtils.stateDiff(currentState, this, false);
  }

  @Override
  public Vector minus(Vector other) {
    return this.globalState.minus(other);
  }

  @Override
  public void minusEquals(Vector other) {
    this.groundState = null;
    this.globalState.minusEquals(other);
  }

  @Override
  public Vector negative() {
    return this.globalState.negative();
  }

  @Override
  public void negativeEquals() {
    this.groundState = null;
    this.globalState.negativeEquals();
  }

  @Override
  public double norm(double power) {
    return this.globalState.norm(power);
  }

  @Override
  public double norm1() {
    return this.globalState.norm1();
  }

  @Override
  public double norm2() {
    return this.globalState.norm2();
  }

  @Override
  public double norm2Squared() {
    return this.globalState.norm2Squared();
  }

  @Override
  public double normInfinity() {
    return this.globalState.normInfinity();
  }

  @Override
  public Matrix outerProduct(Vector other) {
    return this.globalState.outerProduct(other);
  }

  @Override
  public Vector plus(Vector other) {
    return this.globalState.plus(other);
  }

  @Override
  public void plusEquals(Vector other) {
    this.groundState = null;
    this.globalState.plusEquals(other);
  }

  @Override
  public Vector scale(double scaleFactor) {
    return this.globalState.scale(scaleFactor);
  }

  @Override
  public Vector scaledMinus(double scaleFactor, Vector other) {
    this.groundState = null;
    return this.globalState.scaledMinus(scaleFactor, other);
  }

  @Override
  public void scaledMinusEquals(double scaleFactor, Vector other) {
    this.groundState = null;
    this.globalState.scaledMinusEquals(scaleFactor, other);
  }

  @Override
  public Vector scaledPlus(double scaleFactor, Vector other) {
    this.groundState = null;
    return this.globalState.scaledPlus(scaleFactor, other);
  }

  @Override
  public void scaledPlusEquals(double scaleFactor, Vector other) {
    this.groundState = null;
    this.globalState.scaledPlusEquals(scaleFactor, other);
  }

  @Override
  public void scaleEquals(double scaleFactor) {
    this.groundState = null;
    this.globalState.scaleEquals(scaleFactor);
  }

  @Override
  public void setElement(int index, double value) {
    this.globalState.setElement(index, value);
  }

  @Override
  public Vector stack(Vector other) {
    return this.globalState.stack(other);
  }

  @Override
  public Vector subVector(int minIndex, int maxIndex) {
    return this.globalState.subVector(minIndex, maxIndex);
  }

  @Override
  public double sum() {
    return this.globalState.sum();
  }

  @Override
  public Vector times(Matrix matrix) {
    return this.globalState.times(matrix);
  }

  @Override
  public double[] toArray() {
    return this.globalState.toArray();
  }

  @Override
  public String toString() {
    final StringBuilder builder = new StringBuilder();
    builder.append("PathState [path=").append(this.path)
        .append(", state=").append(this.globalState).append("]");
    return builder.toString();
  }

  @Override
  public String toString(NumberFormat format) {
    return this.globalState.toString(format);
  }

  @Override
  public String toString(NumberFormat format, String delimiter) {
    return this.globalState.toString(format, delimiter);
  }

  @Override
  public Vector unitVector() {
    return this.globalState.unitVector();
  }

  @Override
  public void unitVectorEquals() {
    this.globalState.unitVectorEquals();
  }

  @Override
  public void zero() {
    this.globalState.zero();
  }

}
