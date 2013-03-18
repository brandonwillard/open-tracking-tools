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
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.util.PathUtils;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;

public class PathState extends AbstractVector implements
    Comparable<PathState>, Cloneable {

  private static final long serialVersionUID = 2846671162796173049L;

  protected Vector motionState = null;
  protected Path path = null;
  protected PathEdge edge = null;
  protected Vector groundState = null;
  protected Vector edgeState = null;

  public PathState(Path path, Vector state) {
    
    Preconditions.checkArgument(path.isNullPath() || state.getDimensionality() == 2);
    Preconditions.checkArgument(!path.isNullPath() || state.getDimensionality() == 4);
    Preconditions.checkArgument(!(state instanceof PathState));
    
    this.motionState = state;
    this.path = path;
    
    /*
     * Now make sure the result is on this path.
     */
    if (!path.isNullPath()) {
      this.motionState.setElement(0,
          path.clampToPath(this.motionState.getElement(0)));
    }
  }

  public PathState(PathState pathState) {
    this.edge = pathState.edge;
    
    this.motionState = pathState.motionState;
    this.groundState = pathState.groundState;
    this.edgeState = pathState.edgeState;
    this.path = pathState.path;
  }

  @Override
  public double angle(Vector other) {
    return this.motionState.angle(other);
  }

  @Override
  public void assertDimensionalityEquals(int otherDimensionality) {
    this.motionState.assertDimensionalityEquals(otherDimensionality);
  }

  @Override
  public void assertSameDimensionality(Vector other) {
    this.motionState.assertSameDimensionality(other);
  }

  @Override
  public boolean checkSameDimensionality(Vector other) {
    return this.motionState.checkSameDimensionality(other);
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
    clone.edgeState = ObjectUtil.cloneSmart(this.edgeState);
    clone.motionState = ObjectUtil.cloneSmart(this.motionState);
    clone.groundState = ObjectUtil.cloneSmart(this.groundState);
    return clone;
  }

  @Override
  public int compareTo(PathState o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(this.path, o.getPath());
    comparator.append(this.motionState, o.getMotionState());
    return comparator.toComparison();
  }

  @Override
  public void convertFromVector(Vector parameters) {
    this.motionState.convertFromVector(parameters);
  }

  @Override
  public Vector convertToVector() {
    return this.motionState.convertToVector();
  }

  @Override
  public double cosine(Vector other) {
    return this.motionState.cosine(other);
  }

  @Override
  public double dotProduct(Vector other) {
    return this.motionState.dotProduct(other);
  }

  @Override
  public Vector dotTimes(Vector other) {
    return this.motionState.dotTimes(other);
  }

  @Override
  public void dotTimesEquals(Vector other) {
    this.groundState = null;
    this.motionState.dotTimesEquals(other);
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

    if (this.getMotionState() == null) {
      if (other.getMotionState() != null) {
        return false;
      }
    } else if (!this.getMotionState()
        .equals((other.getMotionState()))) {
      return false;
    }

    return true;
  }

  @Override
  public boolean equals(Vector other, double effectiveZero) {
    return this.motionState.equals(other, effectiveZero);
  }

  @Override
  public double euclideanDistance(Vector other) {
    return this.motionState.euclideanDistance(other);
  }

  @Override
  public double euclideanDistanceSquared(Vector other) {
    return this.motionState.euclideanDistanceSquared(other);
  }

  @Override
  public int getDimensionality() {
    return this.motionState.getDimensionality();
  }

  public PathEdge getEdge() {

    if (!this.isOnRoad()) {
      return PathEdge.nullPathEdge;
    }

    if (this.edge == null) {
      Preconditions.checkState(!this.path.isNullPath());
      this.edge =
          this.path.getEdgeForDistance(
              this.motionState.getElement(0), false);
    }
    return Preconditions.checkNotNull(this.edge);
  }

  @Override
  public double getElement(int index) {
    return this.motionState.getElement(index);
  }

  public Vector getMotionState() {
    return this.motionState;
  }

  public Vector getGroundState() {
    if (!this.isOnRoad()) {
      return this.motionState;
    }

    if (this.groundState == null) {
      this.groundState =
          PathUtils.getGroundStateFromRoad(this.motionState,
              this.getEdge(), true);
      
    }

    return this.groundState;
  }

  public Vector getEdgeState() {
    if (this.edgeState != null) {
      return this.edgeState;
    }
    if (this.path.isNullPath()) {
      this.edgeState = this.motionState;
    } else {
      this.edgeState =
          this.getEdge().getCheckedStateOnEdge(
              this.motionState,
              MotionStateEstimatorPredictor
                  .getEdgeLengthErrorTolerance(), true);
    }
    return this.edgeState;
  }

  public Path getPath() {
    return this.path;
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

    return new PathState(newPath, this.motionState);
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
            + ((this.getMotionState() == null) ? 0 : this
                .getMotionState().hashCode());
    return result;
  }

  public boolean isOnRoad() {
    return !this.path.isNullPath();
  }

  @Override
  public boolean isUnitVector() {
    return this.motionState.isUnitVector();
  }

  @Override
  public boolean isUnitVector(double tolerance) {
    return this.motionState.isUnitVector(tolerance);
  }

  @Override
  public boolean isZero() {
    return this.motionState.isZero();
  }

  @Override
  public boolean isZero(double effectiveZero) {
    return this.motionState.isZero(effectiveZero);
  }

  @Override
  public Iterator<VectorEntry> iterator() {
    return this.motionState.iterator();
  }

  @Override
  public Vector minus(Vector other) {
    if (other instanceof PathState)
      return PathUtils.stateDiff((PathState)other, this, false);
    else
      return this.motionState.minus(other);
  }

  @Override
  public void minusEquals(Vector other) {
    this.groundState = null;
    this.motionState.minusEquals(other);
  }

  @Override
  public Vector negative() {
    return this.motionState.negative();
  }

  @Override
  public void negativeEquals() {
    this.groundState = null;
    this.motionState.negativeEquals();
  }

  @Override
  public double norm(double power) {
    return this.motionState.norm(power);
  }

  @Override
  public double norm1() {
    return this.motionState.norm1();
  }

  @Override
  public double norm2() {
    return this.motionState.norm2();
  }

  @Override
  public double norm2Squared() {
    return this.motionState.norm2Squared();
  }

  @Override
  public double normInfinity() {
    return this.motionState.normInfinity();
  }

  @Override
  public Matrix outerProduct(Vector other) {
    return this.motionState.outerProduct(other);
  }

  @Override
  public Vector plus(Vector other) {
    return this.motionState.plus(other);
  }

  @Override
  public void plusEquals(Vector other) {
    this.groundState = null;
    this.motionState.plusEquals(other);
  }

  @Override
  public Vector scale(double scaleFactor) {
    return this.motionState.scale(scaleFactor);
  }

  @Override
  public Vector scaledMinus(double scaleFactor, Vector other) {
    this.groundState = null;
    return this.motionState.scaledMinus(scaleFactor, other);
  }

  @Override
  public void scaledMinusEquals(double scaleFactor, Vector other) {
    this.groundState = null;
    this.motionState.scaledMinusEquals(scaleFactor, other);
  }

  @Override
  public Vector scaledPlus(double scaleFactor, Vector other) {
    this.groundState = null;
    return this.motionState.scaledPlus(scaleFactor, other);
  }

  @Override
  public void scaledPlusEquals(double scaleFactor, Vector other) {
    this.groundState = null;
    this.motionState.scaledPlusEquals(scaleFactor, other);
  }

  @Override
  public void scaleEquals(double scaleFactor) {
    this.groundState = null;
    this.motionState.scaleEquals(scaleFactor);
  }

  @Override
  public void setElement(int index, double value) {
    this.motionState.setElement(index, value);
    if (!this.path.isNullPath() && index == 0) {
      this.edgeState = null; 
    } else {
      this.edgeState.setElement(index, value);
    }
    this.groundState = null;
    
  }

  @Override
  public Vector stack(Vector other) {
    return this.motionState.stack(other);
  }

  @Override
  public Vector subVector(int minIndex, int maxIndex) {
    return this.motionState.subVector(minIndex, maxIndex);
  }

  @Override
  public double sum() {
    return this.motionState.sum();
  }

  @Override
  public Vector times(Matrix matrix) {
    return this.motionState.times(matrix);
  }

  @Override
  public double[] toArray() {
    return this.motionState.toArray();
  }

  @Override
  public String toString() {
    final StringBuilder builder = new StringBuilder();
    builder.append("PathState [path=").append(this.path)
        .append(", state=").append(this.motionState).append("]");
    return builder.toString();
  }

  @Override
  public String toString(NumberFormat format) {
    return this.motionState.toString(format);
  }

  @Override
  public String toString(NumberFormat format, String delimiter) {
    return this.motionState.toString(format, delimiter);
  }

  @Override
  public Vector unitVector() {
    return this.motionState.unitVector();
  }

  @Override
  public void unitVectorEquals() {
    this.motionState.unitVectorEquals();
  }

  @Override
  public void zero() {
    this.motionState.zero();
  }

  public PathState convertToPath(Path newPath) {
    final Vector adjState;
    if (newPath.isNullPath()) {
      adjState = this.getGroundState().clone();
    } else {
  
      final PathState startState =
          new PathState(newPath, VectorFactory
              .getDefault().createVector2D(0d, 0d));
  
      final Vector diff = startState.minus(this);
      diff.negativeEquals();
  
      adjState =
          VectorFactory.getDefault().createVector2D(
              newPath.clampToPath(diff.getElement(0)),
              diff.getElement(1));
  
      assert (newPath.isOnPath(adjState.getElement(0)));
      assert !this.isOnRoad() || Preconditions.checkNotNull(
          new PathState(newPath, adjState)
            .minus(this)
            .isZero(
               MotionStateEstimatorPredictor 
                    .getEdgeLengthErrorTolerance())
          ? Boolean.TRUE : null);
    }
  
    PathState result = new PathState(newPath, adjState);
    
    return result;
  }

}
