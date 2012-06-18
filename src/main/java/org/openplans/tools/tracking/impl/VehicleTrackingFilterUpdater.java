package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.ParticleFilter;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.List;
import java.util.Random;
import java.util.Set;

import jj2000.j2k.NotImplementedError;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;
import org.openplans.tools.tracking.impl.VehicleState.InitialParameters;
import org.opentripplanner.routing.edgetype.StreetEdge;
import org.opentripplanner.routing.graph.Edge;

import com.google.common.collect.ImmutableList;
import com.google.common.collect.Sets;

public class VehicleTrackingFilterUpdater implements
    ParticleFilter.Updater<Observation, VehicleState> {

  private static class UpdaterThreadLocal extends ThreadLocal<Random> {

    long seed;

    public UpdaterThreadLocal(long seed) {
      super();
      this.seed = seed;
    }

    @Override
    public Random get() {
      return super.get();
    }

    public long getSeed() {
      return seed;
    }

    @Override
    protected Random initialValue() {
      final Random rng = new Random();
      if (this.seed == 0l) {
        this.seed = rng.nextLong();
      }
      rng.setSeed(this.seed);

      return rng;
    }

  }

  private static final long serialVersionUID = 2884138088944317656L;
  private final Observation initialObservation;

  private final InferredGraph inferredGraph;

  private final InitialParameters parameters;

  private final ThreadLocal<Random> threadRandom;

  public VehicleTrackingFilterUpdater(Observation obs,
    InferredGraph inferredGraph, InitialParameters parameters) {
    this.initialObservation = obs;
    this.inferredGraph = inferredGraph;
    this.parameters = parameters;
    this.threadRandom = new UpdaterThreadLocal(parameters.getSeed());
  }

  @Override
  public VehicleTrackingFilterUpdater clone() {
    throw new RuntimeException("not implemented");
  }

  /**
   * Evaluate the "point-wise" likelihood, i.e. we don't evaluate a path.
   */
  @Override
  public double computeLogLikelihood(VehicleState particle,
    Observation observation) {
    return particle.getProbabilityFunction().logEvaluate(
        new VehicleStateConditionalParams(PathEdge.getEdge(
            particle.getInferredEdge(), 0d), observation
            .getProjectedPoint(), 0d));
  }

  @Override
  public DataDistribution<VehicleState> createInitialParticles(
    int numParticles) {
    /*
     * Create initial distributions for all snapped edges
     */

//    final List<StreetEdge> initialEdges = inferredGraph.getNearbyEdges(null)
//        .getNarratedGraph().snapToGraph(
//            null, initialObservation.getObsCoords());
    
    final StandardRoadTrackingFilter trackingFilter = new StandardRoadTrackingFilter(parameters.getObsVariance(), 
        parameters.getOffRoadStateVariance(), parameters.getOnRoadStateVariance());
    final MultivariateGaussian initialBelief = trackingFilter.createInitialLearnedObject();
    final Vector xyPoint = initialObservation.getProjectedPoint();
    initialBelief.setMean(VectorFactory.getDefault().copyArray(
        new double[] { xyPoint.getElement(0), 0d,
            xyPoint.getElement(1), 0d }));
    final List<StreetEdge> initialEdges = inferredGraph.getNearbyEdges(initialBelief, trackingFilter);
    
    final DataDistribution<VehicleState> initialDist = new DefaultDataDistribution<VehicleState>(
        numParticles);

    final Set<InferredPathEntry> evaluatedPaths = Sets.newHashSet();
    if (!initialEdges.isEmpty()) {
      for (final Edge nativeEdge : initialEdges) {
        final InferredEdge edge = inferredGraph
            .getInferredEdge(nativeEdge);
        final PathEdge pathEdge = PathEdge.getEdge(edge, 0d);
        final InferredPath path = new InferredPath(
            ImmutableList.of(pathEdge));
        final FilterInformation info = new FilterInformation(
            path, evaluatedPaths, initialDist);
        evaluatedPaths
            .add(new InferredPathEntry(path, null, null, Double.NEGATIVE_INFINITY));
        
        final VehicleState state = new VehicleState(
            this.inferredGraph, initialObservation,
            pathEdge.getInferredEdge(), parameters, info);

        final VehicleStateConditionalParams edgeLoc = new VehicleStateConditionalParams(
            pathEdge, initialObservation.getProjectedPoint());
        final double lik = state.getProbabilityFunction().evaluate(
            edgeLoc);

        initialDist.increment(state, lik);
      }
    }

    /*
     * Free-motion
     */
    final FilterInformation info = new FilterInformation(
        InferredPath.getEmptyPath(), evaluatedPaths, initialDist);
    final VehicleState state = new VehicleState(
        this.inferredGraph, initialObservation,
        InferredGraph.getEmptyEdge(), parameters, info);
    final double lik = state.getProbabilityFunction().evaluate(
        new VehicleStateConditionalParams(initialObservation
            .getProjectedPoint()));
    initialDist.increment(state, lik);

    final DataDistribution<VehicleState> retDist = new DefaultDataDistribution<VehicleState>(
        initialDist.sample(threadRandom.get(), numParticles));

    return retDist;
  }

  public Observation getInitialObservation() {
    return initialObservation;
  }

  public ThreadLocal<Random> getThreadRandom() {
    return threadRandom;
  }

  @Override
  public VehicleState update(VehicleState previousParameter) {
    throw new NotImplementedError();
  }

}