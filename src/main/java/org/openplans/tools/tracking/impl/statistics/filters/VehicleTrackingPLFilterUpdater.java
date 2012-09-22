package org.openplans.tools.tracking.impl.statistics.filters;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.ParticleFilter;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.List;
import java.util.Random;
import java.util.Set;

import javax.annotation.Nonnull;

import org.apache.commons.lang.NotImplementedException;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.InferredPathEntry;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.statistics.DefaultCountedDataDistribution;
import org.openplans.tools.tracking.impl.statistics.OnOffEdgeTransDirMulti;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.routing.edgetype.StreetEdge;
import org.opentripplanner.routing.graph.Edge;

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Sets;

public class VehicleTrackingPLFilterUpdater implements
    ParticleFilter.Updater<Observation, VehicleState> {

  private static class UpdaterThreadLocal extends ThreadLocal<Random> {

    private long seed;

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

    public void setSeed(long nextLong) {
      this.seed = nextLong;
    }

  }

  private static final long serialVersionUID = 2884138088944317656L;
  private final Observation initialObservation;

  private final OtpGraph inferredGraph;

  private final VehicleStateInitialParameters parameters;

  private final ThreadLocal<Random> threadRandom;

  public VehicleTrackingPLFilterUpdater(Observation obs,
    OtpGraph inferredGraph, VehicleStateInitialParameters parameters, 
    @Nonnull Random rng) {
    this.initialObservation = obs;
    this.inferredGraph = inferredGraph;
    this.parameters = parameters;
    
    /*
     * If we passed a Random then we don't want thread local values.
     */
    this.threadRandom = new ThreadLocal<Random>() {
      
      private Random rng;

      @Override
      public Random get() {
        return rng;
      }

      @Override
      public void set(Random value) {
        rng = value;
      }
      
    };
    
    this.threadRandom.set(rng);
    
  }
  
  public VehicleTrackingPLFilterUpdater(Observation obs,
    OtpGraph inferredGraph, VehicleStateInitialParameters parameters) {
    this.initialObservation = obs;
    this.inferredGraph = inferredGraph;
    this.parameters = parameters;
    this.threadRandom = new UpdaterThreadLocal(parameters.getSeed());
  }

  @Override
  public VehicleTrackingPLFilterUpdater clone() {
    throw new NotImplementedException();
  }

  /**
   * Evaluate the "point-wise" likelihood, i.e. we don't evaluate a path.
   */
  @Override
  public double computeLogLikelihood(VehicleState particle,
    Observation observation) {
    throw new NotImplementedException();
    //    return particle.getProbabilityFunction().logEvaluate(
    //        new VehicleStateConditionalParams(PathEdge.getEdge(
    //            particle.getInferredEdge(), 0d), observation
    //            .getProjectedPoint(), 0d));
  }

  @Override
  public DataDistribution<VehicleState> createInitialParticles(
    int numParticles) {

    final StandardRoadTrackingFilter tmpTrackingFilter =
        new StandardRoadTrackingFilter(parameters.getObsVariance(),
            parameters.getOffRoadStateVariance(),
            parameters.getOnRoadStateVariance());
    final MultivariateGaussian tmpInitialBelief =
        tmpTrackingFilter.createInitialLearnedObject();
    final Vector xyPoint = initialObservation.getProjectedPoint();
    tmpInitialBelief.setMean(VectorFactory.getDefault().copyArray(
        new double[] { xyPoint.getElement(0), 0d,
            xyPoint.getElement(1), 0d }));
    final List<StreetEdge> initialEdges =
        inferredGraph.getNearbyEdges(tmpInitialBelief, tmpTrackingFilter);

    final DataDistribution<VehicleState> initialDist =
        new DefaultDataDistribution<VehicleState>(numParticles);

    final Set<InferredPathEntry> evaluatedPaths = Sets.newHashSet();
    if (!initialEdges.isEmpty()) {
      for (final Edge nativeEdge : initialEdges) {
        final InferredEdge edge =
            inferredGraph.getInferredEdge(nativeEdge);
        final PathEdge pathEdge = PathEdge.getEdge(edge, 0d, false);
        final InferredPath path =
            InferredPath.getInferredPath(pathEdge);
        evaluatedPaths.add(new InferredPathEntry(path, null, null,
            null, Double.NEGATIVE_INFINITY));

        final StandardRoadTrackingFilter trackingFilter = new StandardRoadTrackingFilter(
            parameters.getObsVariance(), parameters.getOffRoadStateVariance(), 
            parameters.getOnRoadStateVariance());
        
        final OnOffEdgeTransDirMulti edgeTransDist = new OnOffEdgeTransDirMulti(inferredGraph, 
            parameters.getOnTransitionProbs(), parameters.getOffTransitionProbs());
        
        final VehicleState state =
            new VehicleState(this.inferredGraph, initialObservation,
                pathEdge.getInferredEdge(), trackingFilter, edgeTransDist, 
                this.threadRandom.get());
        
        /*
         * Sample an initial prior for the transition probabilities
         */
        Vector edgeDriorParams = state.getEdgeTransitionDist().getEdgeMotionTransProbPrior()
          .sample(this.threadRandom.get());
        Vector freeDriorParams = state.getEdgeTransitionDist().getFreeMotionTransProbPrior()
          .sample(this.threadRandom.get());
        state.getEdgeTransitionDist().getEdgeMotionTransPrior().setParameters(edgeDriorParams);
        state.getEdgeTransitionDist().getFreeMotionTransPrior().setParameters(freeDriorParams);

        final double lik =
            state.getProbabilityFunction().evaluate(
                initialObservation);

        initialDist.increment(state, lik);
      }
    }

    /*
     * Free-motion
     */
    final StandardRoadTrackingFilter trackingFilter = new StandardRoadTrackingFilter(
        parameters.getObsVariance(), parameters.getOffRoadStateVariance(), 
        parameters.getOnRoadStateVariance());
    
    final OnOffEdgeTransDirMulti edgeTransDist = new OnOffEdgeTransDirMulti(inferredGraph, 
        parameters.getOnTransitionProbs(), parameters.getOffTransitionProbs());
    
    final VehicleState state =
        new VehicleState(this.inferredGraph, initialObservation,
            InferredEdge.getEmptyEdge(), trackingFilter, edgeTransDist, 
            this.threadRandom.get());
    /*
     * Sample an initial prior for the transition probabilities
     */
    Vector edgeDriorParams = state.getEdgeTransitionDist().getEdgeMotionTransProbPrior()
      .sample(this.threadRandom.get());
    Vector freeDriorParams = state.getEdgeTransitionDist().getFreeMotionTransProbPrior()
      .sample(this.threadRandom.get());
    state.getEdgeTransitionDist().getEdgeMotionTransPrior().setParameters(edgeDriorParams);
    state.getEdgeTransitionDist().getFreeMotionTransPrior().setParameters(freeDriorParams);
    
    final double lik =
        state.getProbabilityFunction().evaluate(initialObservation);

    initialDist.increment(state, lik);

    final DataDistribution<VehicleState> retDist =
        new DefaultCountedDataDistribution<VehicleState>(
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
    throw new NotImplementedException();
  }
  
}