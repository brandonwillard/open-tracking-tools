package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseVector;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.bayesian.KalmanFilter;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import jj2000.j2k.NotImplementedError;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.statistics.EdgeTransitionDistributions;
import org.openplans.tools.tracking.impl.statistics.StandardRoadTrackingFilter;
import org.openplans.tools.tracking.impl.util.OtpGraph;

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
    ComputableDistribution<VehicleStateConditionalParams>,
    Comparable<VehicleState> {

  public static class InitialParameters {
    private final Vector obsVariance;
    private final Vector onRoadStateVariance;
    private final Vector offRoadStateVariance;
    private final Vector offTransitionProbs;
    private final Vector onTransitionProbs;
    private final long seed;

    public InitialParameters(Vector obsVariance,
      Vector onRoadStateVariance, Vector offRoadStateVariance,
      Vector offProbs, Vector onProbs, long seed) {
      this.obsVariance = obsVariance;
      this.onRoadStateVariance = onRoadStateVariance;
      this.offRoadStateVariance = offRoadStateVariance;
      this.offTransitionProbs = offProbs;
      this.onTransitionProbs = onProbs;
      this.seed = seed;
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
  }

  public static class PDF extends VehicleState implements
      ProbabilityFunction<VehicleStateConditionalParams> {

    private static final long serialVersionUID = 879217079360170446L;

    public PDF(VehicleState state) {
      super(state);
    }

    @Override
    public PDF clone() {
      return (PDF) super.clone();
    }

    @Override
    public Double evaluate(VehicleStateConditionalParams input) {
      return Math.exp(logEvaluate(input));
    }

    @Override
    public VehicleState.PDF getProbabilityFunction() {
      return this;
    }

    @Override
    public double logEvaluate(VehicleStateConditionalParams input) {
      double logLikelihood = 0d;

      final InferredEdge previousEdge = input.getPathEdge()
          .getInferredEdge();
      /*
       * Edge transitions
       */
      logLikelihood += this.edgeTransitionDist.logEvaluate(
          previousEdge, this.getInferredEdge());

      /*
       * Movement likelihood Note: should be predictive for PL
       */
      PathEdge edge;
      if (this.getInferredEdge().isEmptyEdge()) {
        edge = PathEdge.getEmptyPathEdge();
      } else {
        edge = PathEdge.getEdge(
            this.getInferredEdge(), input.getDistanceToCurrentEdge());
      }
      logLikelihood += this.getMovementFilter().logLikelihood(
          input.getLocation(), this.belief, edge);

      return logLikelihood;
    }

    @Override
    public VehicleStateConditionalParams sample(Random random) {
      throw new NotImplementedError();
    }

    @Override
    public ArrayList<VehicleStateConditionalParams> sample(
      Random random, int numSamples) {
      throw new NotImplementedError();
    }

  }

  private static final long serialVersionUID = 3229140254421801273L;

  private static final double gVariance = 50d * 50d / 4d; // meters
  private static final double dVariance = Math.pow(0.05, 2) / 4d; // m/s^2
  private static final double vVariance = Math.pow(0.05, 2) / 4d; // m/s^2

  /*
   * These members represent the state/parameter samples/sufficient statistics.
   */
  private final StandardRoadTrackingFilter movementFilter;

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
  protected final EdgeTransitionDistributions edgeTransitionDist;
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

  public VehicleState(OtpGraph graph,
    Observation initialObservation, InferredEdge inferredEdge,
    InitialParameters parameters) {

    Preconditions.checkNotNull(initialObservation);
    Preconditions.checkNotNull(inferredEdge);
    Preconditions.checkNotNull(parameters);

    this.movementFilter = new StandardRoadTrackingFilter(
        parameters.getObsVariance(),
        parameters.getOffRoadStateVariance(),
        parameters.getOnRoadStateVariance());

    final double timeDiff;
    if (initialObservation.getPreviousObservation() != null) {
      timeDiff = (initialObservation.getTimestamp().getTime() - initialObservation
          .getPreviousObservation().getTimestamp().getTime()) / 1000d;
    } else {
      timeDiff = 30d;
    }
    Preconditions.checkArgument(timeDiff > 0d);
    this.movementFilter.setCurrentTimeDiff(timeDiff);

    if (inferredEdge.isEmptyEdge()) {
      this.belief = movementFilter.getGroundFilter()
          .createInitialLearnedObject();
      final Vector xyPoint = initialObservation.getProjectedPoint();
      belief.setMean(VectorFactory.getDefault().copyArray(
          new double[] { xyPoint.getElement(0), 0d,
              xyPoint.getElement(1), 0d }));

    } else {
      /*
       * Find our starting position on this edge
       */
      this.belief = movementFilter.getRoadFilter()
          .createInitialLearnedObject();

      final Vector loc = inferredEdge
          .getPointOnEdge(initialObservation.getObsPoint());
      belief
          .setMean(VectorFactory.getDefault()
              .copyArray(
                  new double[] {
                      inferredEdge.getStartPoint().euclideanDistance(
                          loc), 0d }));
    }

    this.initialBelief = belief.clone();
    this.edge = inferredEdge;
    this.path = InferredPath.getInferredPath(this.edge);
    this.observation = initialObservation;
    this.graph = graph;
    this.edgeTransitionDist = new EdgeTransitionDistributions(
        this.graph, parameters.getOnTransitionProbs(),
        parameters.getOffTransitionProbs());
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
    StandardRoadTrackingFilter filter, MultivariateGaussian belief,
    EdgeTransitionDistributions edgeTransitionDist, PathEdge edge,
    InferredPath path, VehicleState state) {

    Preconditions
        .checkArgument(!(belief.getInputDimensionality() == 2 && edge.isEmptyEdge()));
    Preconditions.checkNotNull(state);
    Preconditions.checkNotNull(graph);
    Preconditions.checkNotNull(observation);
    Preconditions.checkNotNull(filter);
    Preconditions.checkNotNull(belief);
    Preconditions.checkNotNull(edge);

    this.observation = observation;
    this.movementFilter = filter;
    this.belief = belief.clone();
    this.graph = graph;
    this.path = path;
    /*
     * This is the constructor used when creating transition states, so this is
     * where we'll need to reset the distance measures
     */
    this.distanceFromPreviousState = edge.getDistToStartOfEdge();
    if (this.belief.getInputDimensionality() == 2) {
      this.belief.getMean().setElement(
          0,
          this.belief.getMean().getElement(0)
              - edge.getDistToStartOfEdge());
    }
    /*
     * This has to come after the adjustment 
     */
    this.initialBelief = this.belief.clone();
    
    this.edgeTransitionDist = edgeTransitionDist;
    this.edge = edge.getInferredEdge();

    this.parentState = state;
    /*
     * Reset the parent's parent state so that we don't keep these objects
     * forever.
     */
    // state.parentState = null;

    final double timeDiff;
    if (observation.getPreviousObservation() != null) {
      timeDiff = (observation.getTimestamp().getTime() - observation
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
    final DenseVector thisV = (DenseVector) initialBelief
        .convertToVector();
    final DenseVector otherV = (DenseVector) other.initialBelief
        .convertToVector();
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

  public EdgeTransitionDistributions getEdgeTransitionDist() {
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
      final MultivariateGaussian beliefProj = new MultivariateGaussian();
      StandardRoadTrackingFilter.convertToGroundBelief(
          beliefProj, PathEdge.getEdge(this.edge, 0d));
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

  public KalmanFilter getKalmanFilter() {
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
      StandardRoadTrackingFilter.invertProjection(
          projBelief, PathEdge.getEdge(this.edge, 0d));
      v = projBelief.getMean();
    } else {
      v = belief.getMean();
    }
    return VectorFactory.getDefault().createVector2D(
        v.getElement(0), v.getElement(2));
  }

  public StandardRoadTrackingFilter getMovementFilter() {
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
    final int prime = 31;
    int result = 1;
    final DenseVector thisV = (DenseVector) initialBelief
        .convertToVector();
    result = prime
        * result
        + ((initialBelief == null) ? 0 : Arrays.hashCode(thisV
            .getArray()));
    result = prime * result + ((edge == null) ? 0 : edge.hashCode());
    result = prime
        * result
        + ((edgeTransitionDist == null) ? 0 : edgeTransitionDist
            .hashCode());
    result = prime * result
        + ((observation == null) ? 0 : observation.hashCode());
    return result;
  }

  @Override
  public VehicleStateConditionalParams sample(Random random) {
    throw new NotImplementedError();
  }

  @Override
  public ArrayList<VehicleStateConditionalParams> sample(
    Random random, int numSamples) {
    throw new NotImplementedError();
  }

  public void setParentState(VehicleState parentState) {
    this.parentState = parentState;
  }

  @Override
  public String toString() {
    return Objects.toStringHelper(this).add("belief", belief)
        .add("edge", edge.getEdgeId())
        .addValue(observation.getTimestamp()).toString();
  }

  public static double getDvariance() {
    return dVariance;
  }

  public static double getGvariance() {
    return gVariance;
  }

  public static Vector getNonVelocityVector(Vector vector) {
    final Vector res;
    if (vector.getDimensionality() == 4)
      res = StandardRoadTrackingFilter.getOg().times(vector);
    else
      res = StandardRoadTrackingFilter.getOr().times(vector);
    return res;
  }

  public static long getSerialversionuid() {
    return serialVersionUID;
  }

  public static double getVvariance() {
    return vVariance;
  }

  private static int oneStateCompareTo(VehicleState t, VehicleState o) {
    if (t == o)
      return 0;

    if (t == null) {
      if (o != null)
        return -1;
    } else if (o == null) {
      return 1;
    }

    final DenseVector thisV = (DenseVector) t.initialBelief
        .convertToVector();
    final DenseVector otherV = (DenseVector) o.initialBelief
        .convertToVector();
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(thisV.getArray(), otherV.getArray());
    comparator.append(t.getObservation(), o.getObservation());
    comparator.append(t.edge, o.edge);
    comparator.append(t.edgeTransitionDist, o.edgeTransitionDist);

    return comparator.toComparison();
  }

}
