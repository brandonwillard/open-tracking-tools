package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseVector;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import jj2000.j2k.NotImplementedError;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.statistics.OnOffEdgeTransDirMulti;
import org.openplans.tools.tracking.impl.statistics.StatisticsUtil;
import org.openplans.tools.tracking.impl.statistics.filters.AbstractRoadTrackingFilter;
import org.openplans.tools.tracking.impl.statistics.filters.AdjKalmanFilter;
import org.openplans.tools.tracking.impl.util.OtpGraph;

import com.google.common.annotations.Beta;
import com.google.common.base.Objects;
import com.google.common.base.Preconditions;

/**
 * This class represents the state of a vehicle, which is made up of the
 * vehicles location, whether it is on an edge, which path it took from its
 * previous location on an edge, and the distributions that determine these.
 * 
 * @author bwillard
 * 
 */
public class VehicleState implements
    ComputableDistribution<Observation>, Comparable<VehicleState> {

  public static class PDF extends VehicleState implements
      ProbabilityFunction<Observation> {

    private static final long serialVersionUID = 879217079360170446L;

    public PDF(VehicleState state) {
      super(state);
    }

    @Override
    public PDF clone() {
      return (PDF) super.clone();
    }

    @Override
    public Double evaluate(Observation input) {
      return Math.exp(logEvaluate(input));
    }

    @Override
    public VehicleState.PDF getProbabilityFunction() {
      return this;
    }

    @Beta
    @Override
    public double logEvaluate(Observation input) {
      double logLikelihood = 0d;

//      /*
//       * Edge transitions.
//       * XXX: Very important!  This skips the edges in-between.
//       */
//      final InferredEdge parentEdge = this.getParentState() != null ? 
//          this.getParentState().getInferredEdge() : null;
//      logLikelihood +=
//          this.edgeTransitionDist.logEvaluate(parentEdge,
//              this.getInferredEdge());

      /*
       * Movement.
       * XXX: this does not include the state transition likelihood.
       */
      logLikelihood +=
          this.getMovementFilter().logLikelihood(
              input.getProjectedPoint(), this.belief,
              PathEdge.getEdge(this.getInferredEdge()));

      return logLikelihood;
    }

    @Override
    public Observation sample(Random random) {
      throw new NotImplementedError();
    }

    @Override
    public ArrayList<Observation>
        sample(Random random, int numSamples) {
      throw new NotImplementedError();
    }

  }

  public static class VehicleStateInitialParameters {
    private final Vector obsVariance;
    private final Vector onRoadStateVariance;
    private final Vector offRoadStateVariance;
    private final Vector offTransitionProbs;
    private final Vector onTransitionProbs;
    private final long seed;
    private final int numParticles;
    private final String filterTypeName;

    public VehicleStateInitialParameters(Vector obsVariance,
      Vector onRoadStateVariance, Vector offRoadStateVariance,
      Vector offProbs, Vector onProbs, String filterTypeName, 
      int numParticles, long seed) {
      this.numParticles = numParticles;
      this.obsVariance = obsVariance;
      this.onRoadStateVariance = onRoadStateVariance;
      this.offRoadStateVariance = offRoadStateVariance;
      this.offTransitionProbs = offProbs;
      this.onTransitionProbs = onProbs;
      this.seed = seed;
      this.filterTypeName = filterTypeName;
    }

    public int getNumParticles() {
      return numParticles;
    }

    public Vector getObsVariance() {
      return obsVariance;
    }

    public Vector getOffRoadStateVariance() {
      return offRoadStateVariance;
    }

    public Vector getOffTransitionProbs() {
      return offTransitionProbs;
    }

    public Vector getOnRoadStateVariance() {
      return onRoadStateVariance;
    }

    public Vector getOnTransitionProbs() {
      return onTransitionProbs;
    }

    public long getSeed() {
      return seed;
    }

    public String getFilterTypeName() {
      return filterTypeName;
    }

    @Override
    public int hashCode() {
      final int prime = 31;
      int result = 1;
      result =
          prime
              * result
              + ((filterTypeName == null) ? 0 : filterTypeName
                  .hashCode());
      result = prime * result + numParticles;
      result =
          prime * result
              + StatisticsUtil.hashCodeVector(obsVariance);
      result =
          prime
              * result
              + StatisticsUtil.hashCodeVector(offRoadStateVariance);
      result =
          prime
              * result
              + StatisticsUtil.hashCodeVector(offTransitionProbs);
      result =
          prime
              * result
              + StatisticsUtil.hashCodeVector(onRoadStateVariance);
      result =
          prime
              * result
              + StatisticsUtil.hashCodeVector(onTransitionProbs);
      result = prime * result + (int) (seed ^ (seed >>> 32));
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
      VehicleStateInitialParameters other =
          (VehicleStateInitialParameters) obj;
      if (filterTypeName == null) {
        if (other.filterTypeName != null) {
          return false;
        }
      } else if (!filterTypeName.equals(other.filterTypeName)) {
        return false;
      }
      if (numParticles != other.numParticles) {
        return false;
      }
      if (obsVariance == null) {
        if (other.obsVariance != null) {
          return false;
        }
      } else if (!StatisticsUtil.vectorEquals(obsVariance, other.obsVariance)) {
        return false;
      }
      if (offRoadStateVariance == null) {
        if (other.offRoadStateVariance != null) {
          return false;
        }
      } else if (!StatisticsUtil
          .vectorEquals(offRoadStateVariance, other.offRoadStateVariance)) {
        return false;
      }
      if (offTransitionProbs == null) {
        if (other.offTransitionProbs != null) {
          return false;
        }
      } else if (!StatisticsUtil.vectorEquals(offTransitionProbs, other.offTransitionProbs)) {
        return false;
      }
      if (onRoadStateVariance == null) {
        if (other.onRoadStateVariance != null) {
          return false;
        }
      } else if (!StatisticsUtil
          .vectorEquals(onRoadStateVariance, other.onRoadStateVariance)) {
        return false;
      }
      if (onTransitionProbs == null) {
        if (other.onTransitionProbs != null) {
          return false;
        }
      } else if (!StatisticsUtil.vectorEquals(onTransitionProbs, other.onTransitionProbs)) {
        return false;
      }
      if (seed != other.seed) {
        return false;
      }
      return true;
    }
  }

  private static final long serialVersionUID = 3229140254421801273L;

  /*
   * These members represent the state/parameter samples/sufficient statistics.
   */
  private final AbstractRoadTrackingFilter movementFilter;

  /**
   * This could be the 4D ground-coordinates dist. for free motion, or the 2D
   * road-coordinates, either way the tracking filter will check. Also, this
   * could be the prior or prior predictive distribution.
   */
  protected final MultivariateGaussian belief;

  private final MultivariateGaussian initialBelief;

  /*-
   * Edge transition priors 
   * 1. edge off 
   * 2. edge on 
   * 3. edges transitions to others (one for all)
   * edges
   */
  protected final OnOffEdgeTransDirMulti edgeTransitionDist;
  private final Observation observation;
  private final InferredEdge edge;
  private VehicleState parentState = null;
  private final Double distanceFromPreviousState;

  private final OtpGraph graph;

  // private final int initialHashCode;
  // private final int edgeInitialHashCode;
  // private final int obsInitialHashCode;
  // private final int transInitialHashCode;
  // private final int beliefInitialHashCode;

  private final InferredPath path;

  private int hash = 0;

  public VehicleState(OtpGraph graph, Observation initialObservation,
    InferredEdge inferredEdge,
    AbstractRoadTrackingFilter movementFilter, 
    OnOffEdgeTransDirMulti edgeTransDist, Random rng) {

    Preconditions.checkNotNull(initialObservation);
    Preconditions.checkNotNull(inferredEdge);

    this.movementFilter = movementFilter;

    final double timeDiff;
    if (initialObservation.getPreviousObservation() != null) {
      timeDiff =
          (initialObservation.getTimestamp().getTime() - initialObservation
              .getPreviousObservation().getTimestamp().getTime()) / 1000d;
    } else {
      timeDiff = 30d;
    }
    Preconditions.checkArgument(timeDiff > 0d);
    this.movementFilter.setCurrentTimeDiff(timeDiff);

    if (inferredEdge.isEmptyEdge()) {
      this.belief =
          movementFilter.getGroundFilter()
              .createInitialLearnedObject();
      final Vector xyPoint = initialObservation.getProjectedPoint();

      /*
       * Sample the velocity
       */
      belief.setMean(this.movementFilter.sampleStateBelief(
          belief.getMean(), rng));
      belief.getMean().setElement(0, xyPoint.getElement(0));
      belief.getMean().setElement(2, xyPoint.getElement(1));

    } else {
      /*
       * Find our starting position on this edge
       */
      this.belief =
          movementFilter.getRoadFilter().createInitialLearnedObject();

      final double lengthLocation =
          inferredEdge.getLengthIndexedLine().project(
              initialObservation.getObsPoint());

      /*
       * Sample the velocity
       */
      belief.setMean(this.movementFilter.sampleStateBelief(
          belief.getMean(), rng));
      belief.getMean().setElement(0, lengthLocation);

      AbstractRoadTrackingFilter.normalizeBelief(
          this.belief.getMean(), PathEdge.getEdge(inferredEdge));

      assert Double.compare(
          Math.abs(this.belief.getMean().getElement(0)),
          inferredEdge.getLength()) <= 0;
    }

    this.edge = inferredEdge;
    this.initialBelief = belief.clone();
    this.path = InferredPath.getInferredPath(this.edge);
    this.observation = initialObservation;
    this.graph = graph;
    this.edgeTransitionDist = edgeTransDist;
    this.distanceFromPreviousState = 0d;

    // DEBUG
    // this.initialHashCode = this.hashCode();
    // this.edgeInitialHashCode = this.edge.hashCode();
    // this.transInitialHashCode = this.edgeTransitionDist.hashCode();
    // this.beliefInitialHashCode =
    // Arrays.hashCode(((DenseVector)this.initialBelief.convertToVector()).getArray());
    // this.obsInitialHashCode = this.observation.hashCode();
  }

  public VehicleState(OtpGraph graph, Observation observation,
    AbstractRoadTrackingFilter updatedFilter, MultivariateGaussian belief,
    OnOffEdgeTransDirMulti edgeTransitionDist,
    InferredPath path, VehicleState state) {

    Preconditions
        .checkArgument(!(belief.getInputDimensionality() == 2 && path
            .isEmptyPath()));
    Preconditions.checkNotNull(state);
    Preconditions.checkNotNull(graph);
    Preconditions.checkNotNull(observation);
    Preconditions.checkNotNull(updatedFilter);
    Preconditions.checkNotNull(belief);

    this.observation = observation;
    this.movementFilter = updatedFilter;
    this.belief = belief.clone();
    this.graph = graph;
    this.path = path;
    /*
     * This is the constructor used when creating transition states, so this is
     * where we'll need to reset the distance measures
     */
    if (this.belief.getInputDimensionality() == 2) {
      /*
       * IMPORTANT: the filtering that occurs on edges doesn't mean
       * that the filtered location will end up on the same edge.
       */
      final double distPosition = this.belief.getMean().getElement(0);
      final PathEdge pathEdge =
          path.getEdgeForDistance(distPosition, true);

      assert !path.isEmptyPath() && !pathEdge.isEmptyEdge();

      this.edge = pathEdge.getInferredEdge();
      this.distanceFromPreviousState =
          pathEdge.getDistToStartOfEdge();

      /*
       * We normalized the position relative to the direction of motion.
       */
      this.belief.getMean().setElement(0,
          distPosition - pathEdge.getDistToStartOfEdge());
      AbstractRoadTrackingFilter.normalizeBelief(
          this.belief.getMean(), PathEdge.getEdge(edge));

      assert Double.compare(
          Math.abs(this.belief.getMean().getElement(0)),
          edge.getLength() + 0.9) <= 0;

    } else {

      this.edge = InferredEdge.getEmptyEdge();
      this.distanceFromPreviousState = null;
    }

    /*
     * This has to come after the adjustment 
     */
    this.initialBelief = this.belief.clone();

    this.edgeTransitionDist = edgeTransitionDist;

    this.parentState = state;
    /*
     * Reset the parent's parent state so that we don't keep these objects
     * forever.
     */
    // state.parentState = null;

    final double timeDiff;
    if (observation.getPreviousObservation() != null) {
      timeDiff =
          (observation.getTimestamp().getTime() - observation
              .getPreviousObservation().getTimestamp().getTime()) / 1000d;
    } else {
      timeDiff = 30d;
    }
    this.movementFilter.setCurrentTimeDiff(timeDiff);

    // DEBUG
    // this.initialHashCode = this.hashCode();
    // this.edgeInitialHashCode = this.edge.hashCode();
    // this.transInitialHashCode = this.edgeTransitionDist.hashCode();
    // this.beliefInitialHashCode =
    // Arrays.hashCode(((DenseVector)this.initialBelief.convertToVector()).getArray());
    // this.obsInitialHashCode = this.observation.hashCode();
  }

  public VehicleState(VehicleState other) {
    this.graph = other.graph;
    this.movementFilter = other.movementFilter.clone();
    this.belief = other.belief.clone();
    this.edgeTransitionDist = other.edgeTransitionDist.clone();
    // TODO clone this?
    this.edge = other.edge;
    this.observation = other.observation;
    this.distanceFromPreviousState = other.distanceFromPreviousState;
    this.parentState = other.parentState;
    this.initialBelief = other.initialBelief.clone();
    this.path = other.path;

    // DEBUG
    // this.initialHashCode = this.hashCode();
    // this.edgeInitialHashCode = this.edge.hashCode();
    // this.transInitialHashCode = this.edgeTransitionDist.hashCode();
    // this.beliefInitialHashCode =
    // Arrays.hashCode(((DenseVector)this.initialBelief.convertToVector()).getArray());
    // this.obsInitialHashCode = this.observation.hashCode();
  }

  @Override
  public VehicleState clone() {
    return new VehicleState(this);
  }

  @Override
  public int compareTo(VehicleState arg0) {
    return oneStateCompareTo(this, arg0);
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
    final VehicleState other = (VehicleState) obj;
    final DenseVector thisV =
        (DenseVector) initialBelief.convertToVector();
    final DenseVector otherV =
        (DenseVector) other.initialBelief.convertToVector();
    if (initialBelief == null) {
      if (other.initialBelief != null) {
        return false;
      }
    } else if (!Arrays.equals(thisV.getArray(), otherV.getArray())) {
      return false;
    }
    if (edge == null) {
      if (other.edge != null) {
        return false;
      }
    } else if (!edge.equals(other.edge)) {
      return false;
    }
    if (edgeTransitionDist == null) {
      if (other.edgeTransitionDist != null) {
        return false;
      }
    } else if (!edgeTransitionDist.equals(other.edgeTransitionDist)) {
      return false;
    }
    if (observation == null) {
      if (other.observation != null) {
        return false;
      }
    } else if (!observation.equals(other.observation)) {
      return false;
    }
    return true;
  }

  public MultivariateGaussian getBelief() {
    return belief;
  }

  public Double getDistanceFromPreviousState() {
    return distanceFromPreviousState;
  }

  public InferredEdge getEdge() {
    return edge;
  }

  public OnOffEdgeTransDirMulti getEdgeTransitionDist() {
    return edgeTransitionDist;
  }

  public OtpGraph getGraph() {
    return graph;
  }

  /**
   * Gets the belief in ground coordinates (even if it's really tracking road
   * coordinates).
   * 
   * @return
   */
  public MultivariateGaussian getGroundOnlyBelief() {
    if (belief.getInputDimensionality() == 2) {
      final MultivariateGaussian beliefProj =
          new MultivariateGaussian();
      AbstractRoadTrackingFilter.convertToGroundBelief(beliefProj,
          PathEdge.getEdge(this.edge, 0d, this.path.getIsBackward()));
      return beliefProj;
    } else {
      return belief;
    }
  }

  public InferredEdge getInferredEdge() {
    return this.edge;
  }

  public MultivariateGaussian getInitialBelief() {
    return initialBelief;
  }

  public AdjKalmanFilter getKalmanFilter() {
    return this.belief.getInputDimensionality() == 4 ? this
        .getMovementFilter().getGroundFilter() : this
        .getMovementFilter().getRoadFilter();
  }

  /**
   * Returns ground-coordinate mean location
   * 
   * @return
   */
  public Vector getMeanLocation() {
    Vector v;
    if (belief.getInputDimensionality() == 2) {
      Preconditions.checkArgument(!this.edge.isEmptyEdge());
      final MultivariateGaussian projBelief = belief.clone();
      AbstractRoadTrackingFilter.convertToGroundBelief(projBelief,
          PathEdge.getEdge(this.edge, 0d, this.path.getIsBackward()));
      v = projBelief.getMean();
    } else {
      v = belief.getMean();
    }
    return VectorFactory.getDefault().createVector2D(v.getElement(0),
        v.getElement(2));
  }

  public AbstractRoadTrackingFilter getMovementFilter() {
    return movementFilter;
  }

  public Vector getNonVelocityVector() {
    return getNonVelocityVector(this.belief.getMean());
  }

  public Observation getObservation() {
    return observation;
  }

  public VehicleState getParentState() {
    return parentState;
  }

  public InferredPath getPath() {
    return path;
  }

  @Override
  public VehicleState.PDF getProbabilityFunction() {
    return new VehicleState.PDF(this);
  }

  @Override
  public int hashCode() {
    if (hash != 0)
      return hash;

    final int prime = 31;
    int result = 1;
    final DenseVector thisV =
        (DenseVector) initialBelief.convertToVector();
    result =
        prime
            * result
            + ((initialBelief == null) ? 0 : Arrays.hashCode(thisV
                .getArray()));
    result = prime * result + ((edge == null) ? 0 : edge.hashCode());
    result =
        prime
            * result
            + ((edgeTransitionDist == null) ? 0 : edgeTransitionDist
                .hashCode());
    result =
        prime * result
            + ((observation == null) ? 0 : observation.hashCode());
    hash = result;
    return result;
  }

  @Override
  public Observation sample(Random random) {
    throw new NotImplementedError();
  }

  @Override
  public ArrayList<Observation> sample(Random random, int numSamples) {
    throw new NotImplementedError();
  }

  public void setParentState(VehicleState parentState) {
    this.parentState = parentState;
  }

  @Override
  public String toString() {
    return "VehicleState [movementFilter=" + movementFilter
        + ", belief=" + belief + ", initialBelief=" + initialBelief
        + ", edgeTransitionDist=" + edgeTransitionDist
        + ", observation=" + observation + ", edge=" + edge
        + ", parentState=" + parentState
        + ", distanceFromPreviousState=" + distanceFromPreviousState
        + ", graph=" + graph + ", path=" + path + "]";
  }

  public static Vector getNonVelocityVector(Vector vector) {
    final Vector res;
    if (vector.getDimensionality() == 4)
      res = AbstractRoadTrackingFilter.getOg().times(vector);
    else
      res = AbstractRoadTrackingFilter.getOr().times(vector);
    return res;
  }

  public static long getSerialversionuid() {
    return serialVersionUID;
  }

  private static int
      oneStateCompareTo(VehicleState t, VehicleState o) {
    if (t == o)
      return 0;

    if (t == null) {
      if (o != null)
        return -1;
      else
        return 0;
    } else if (o == null) {
      return 1;
    }

    final DenseVector thisV =
        (DenseVector) t.initialBelief.convertToVector();
    final DenseVector otherV =
        (DenseVector) o.initialBelief.convertToVector();
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(thisV.getArray(), otherV.getArray());
    comparator.append(t.getObservation(), o.getObservation());
    comparator.append(t.edge, o.edge);
    comparator.append(t.edgeTransitionDist, o.edgeTransitionDist);

    return comparator.toComparison();
  }

}
