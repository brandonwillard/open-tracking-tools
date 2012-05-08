package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.bayesian.AbstractKalmanFilter;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import jj2000.j2k.NotImplementedError;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.Iterables;
import com.vividsolutions.jts.geom.Coordinate;

/**
 * This class represents the state of a vehicle, which is made up of the
 * vehicles location, whether it is on an edge, which path it took from its
 * previous location on an edge, and the distributions that determine these.
 * 
 * @author bwillard
 * 
 */
public class VehicleState implements ComputableDistribution<EdgeLocation> {

  public static class PDF extends VehicleState implements
      ProbabilityFunction<EdgeLocation> {

    private static final long serialVersionUID = 879217079360170446L;

    public PDF(VehicleState state) {
      super(state);
    }

    @Override
    public PDF clone() {
      return (PDF) super.clone();
    }

    @Override
    public Double evaluate(EdgeLocation input) {
      return Math.exp(logEvaluate(input));
    }

    @Override
    public VehicleState.PDF getProbabilityFunction() {
      return this;
    }

    @Override
    public double logEvaluate(EdgeLocation input) {
      double logLikelihood = 0d;
      final Vector location = input.getLocation();

      /*
       * Evaluate the transition of edges leading up to the given location (if a
       * path is given).
       */
      InferredEdge prevEdgeInfo = null;
      for (final InferredEdge currentEdgeInfo : input.getPath()) {

        if (prevEdgeInfo == null) {
          prevEdgeInfo = currentEdgeInfo;
          continue;
        }

        /*
         * Edge transitions
         */
        logLikelihood += this.edgeTransitionDist.logEvaluate(prevEdgeInfo,
            currentEdgeInfo);

        /*
         * Movement likelihood Note: should be predictive for PL
         */
        logLikelihood += this.movementBelief.getProbabilityFunction()
            .logEvaluate(location);

      }

      return logLikelihood;
    }

    @Override
    public EdgeLocation sample(Random random) {
      throw new NotImplementedError();
    }

    @Override
    public ArrayList<EdgeLocation> sample(Random random, int numSamples) {
      throw new NotImplementedError();
    }

  }

  private static final long serialVersionUID = 3229140254421801273L;
  private static final double gVariance = 50d; // meters

  private static final double aVariance = 0.5d; // m/s^2
  /*
   * These members represent the state/parameter samples/sufficient statistics.
   */
  private final Standard2DTrackingFilter movementFilter;
  protected final MultivariateGaussian movementBelief;

  /*-
   * Edge transition priors 
   * 1. edge off 
   * 2. edge on 
   * 3. edges transitions to others (one for all)
   * edges
   */
  protected final EdgeTransitionDistributions edgeTransitionDist;
  

  /*
   * Current edge is the last element, naturally.
   */
  private final List<InferredEdge> inferredPath;
  private final Observation observation;

  public VehicleState(Observation initialObservation) {
    Preconditions.checkNotNull(initialObservation);
    final Standard2DTrackingFilter freeMovementFilter = new Standard2DTrackingFilter(
        gVariance, aVariance, 0d, null);
    final MultivariateGaussian freeMovementBelief = freeMovementFilter
        .createInitialLearnedObject();
    final Vector xyPoint = initialObservation.getProjectedPoint();
    freeMovementBelief.setMean(VectorFactory.getDefault().copyArray(
        new double[] { xyPoint.getElement(0), 0d, xyPoint.getElement(1), 0d }));

    this.observation = initialObservation;
    this.movementFilter = freeMovementFilter;
    this.movementBelief = freeMovementBelief;
    this.edgeTransitionDist = new EdgeTransitionDistributions();
    this.inferredPath = ImmutableList.of(InferredGraph.getEmptyEdge());
  }

  public VehicleState(Observation initialObservation, List<InferredEdge> path) {
    Preconditions.checkNotNull(initialObservation);
    Preconditions.checkNotNull(path);
    Preconditions.checkArgument(!path.isEmpty());

    final InferredEdge edge = Iterables.getLast(path);

    /*
     * Get y-axis angle of edge to use in constrained state covariance matrix.
     */
    final Double angle = edge.getAngle();

    final Standard2DTrackingFilter edgeMovementFilter = new Standard2DTrackingFilter(
        gVariance, aVariance, 0d, angle);
    final MultivariateGaussian movementBelief = edgeMovementFilter
        .createInitialLearnedObject();

    /*
     * Set the initial position on the motion model
     */
    final Coordinate xyPoint = edge == InferredGraph.getEmptyEdge()?
        initialObservation.getObsPoint() : 
          edge.getPointOnEdge(initialObservation.getObsPoint());
        
    movementBelief.setMean(VectorFactory.getDefault().copyArray(
        new double[] { xyPoint.x, 0d, xyPoint.y, 0d }));

    this.observation = initialObservation;
    this.movementFilter = edgeMovementFilter;
    this.movementBelief = movementBelief;
    this.edgeTransitionDist = new EdgeTransitionDistributions();
    this.inferredPath = ImmutableList.copyOf(path);
  }

  public VehicleState(VehicleState other) {
    this.movementFilter = other.movementFilter;
    this.movementBelief = other.movementBelief;
    this.edgeTransitionDist = other.edgeTransitionDist;
    this.inferredPath = other.inferredPath;
    this.observation = other.observation;
  }

  public VehicleState(Observation observation,
    Standard2DTrackingFilter filter, MultivariateGaussian belief,
    EdgeTransitionDistributions edgeTransitionDist,
    ImmutableList<InferredEdge> path) {
    this.observation = observation;
    this.movementFilter = filter;
    this.movementBelief = belief;
    this.edgeTransitionDist = edgeTransitionDist;
    this.inferredPath = path;
  }

  @Override
  public VehicleState clone() {
    try {
      return (VehicleState) super.clone();
    } catch (final CloneNotSupportedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    return null;
  }

  public EdgeTransitionDistributions getEdgeTransitionDist() {
    return edgeTransitionDist;
  }

  public List<InferredEdge> getInferredPath() {
    return inferredPath;
  }

  public Coordinate getMeanLocation() {
    return getMeanLocation(this.movementBelief.getMean());
  }

  public MultivariateGaussian getMovementBelief() {
    return movementBelief;
  }

  public Standard2DTrackingFilter getMovementFilter() {
    return movementFilter;
  }

  @Override
  public VehicleState.PDF getProbabilityFunction() {
    return new VehicleState.PDF(this);
  }

  @Override
  public EdgeLocation sample(Random random) {
    throw new NotImplementedError();
  }

  @Override
  public ArrayList<EdgeLocation> sample(Random random, int numSamples) {
    throw new NotImplementedError();
  }

  public static double getAvariance() {
    return aVariance;
  }

  public static double getGvariance() {
    return gVariance;
  }

  /**
   * Pull coordinates from a vector of location & velocity.
   */
  public static Coordinate getMeanLocation(Vector vec) {
    return new Coordinate(vec.getElement(0), vec.getElement(2));
  }

  public static long getSerialversionuid() {
    return serialVersionUID;
  }

  public Observation getObservation() {
    return observation;
  }

  @Override
  public String toString() {
    return "VehicleState [movementFilter=" + movementFilter
        + ", movementBelief=" + movementBelief + ", edgeTransitionDist="
        + edgeTransitionDist + ", inferredPath=" + inferredPath
        + ", observation=" + observation + "]";
  }

}
