package org.opentrackingtools.statistics.filters.vehicles.particle_learning;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.ParticleFilter;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Collection;
import java.util.Random;
import java.util.Set;

import javax.annotation.Nonnull;

import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.impl.InferredPathPrediction;
import org.opentrackingtools.graph.paths.states.impl.SimplePathStateBelief;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleState.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.distributions.impl.DefaultCountedDataDistribution;
import org.opentrackingtools.statistics.distributions.impl.OnOffEdgeTransDirMulti;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;

import com.google.common.collect.Sets;

public abstract class AbstractVTParticleFilterUpdater
    implements
    ParticleFilter.Updater<GpsObservation, VehicleState> {

  private static final long serialVersionUID =
      7567157323292175525L;

  protected GpsObservation initialObservation;

  protected InferenceGraph inferenceGraph;

  protected VehicleStateInitialParameters parameters;

  protected Random random;

  public long seed;

  public AbstractVTParticleFilterUpdater(GpsObservation obs,
    InferenceGraph graph,
    VehicleStateInitialParameters parameters) {
    this.initialObservation = obs;
    this.inferenceGraph = graph;
    this.parameters = parameters;
  }

  @Override
  public AbstractVTParticleFilterUpdater clone() {
    try {
      final AbstractVTParticleFilterUpdater clone =
          (AbstractVTParticleFilterUpdater) super.clone();
      clone.seed = seed;
      clone.inferenceGraph = inferenceGraph;
      clone.initialObservation = initialObservation;
      clone.parameters = clone.parameters.clone();
      clone.random = random;
      return clone;
    } catch (final CloneNotSupportedException e) {
      e.printStackTrace();
    }
    return null;
  }

  @Nonnull
  @Override
  public DataDistribution<VehicleState>
      createInitialParticles(int numParticles) {

    final DataDistribution<VehicleState> retDist =
        new DefaultCountedDataDistribution<VehicleState>();

    for (int i = 0; i < numParticles; i++) {
      final AbstractRoadTrackingFilter<?> tmpTrackingFilter =
          this.createRoadTrackingFilter();
      final MultivariateGaussian tmpInitialBelief =
          tmpTrackingFilter.createInitialLearnedObject();
      final Vector xyPoint =
          initialObservation.getProjectedPoint();
      tmpInitialBelief.setMean(VectorFactory.getDefault()
          .copyArray(
              new double[] { xyPoint.getElement(0), 0d,
                  xyPoint.getElement(1), 0d }));
      final Collection<InferredEdge> initialEdges =
          inferenceGraph.getNearbyEdges(tmpInitialBelief,
              tmpTrackingFilter);

      final DataDistribution<VehicleState> initialDist =
          new DefaultDataDistribution<VehicleState>(
              numParticles);

      final Set<InferredPathPrediction> evaluatedPaths =
          Sets.newHashSet();
      if (!initialEdges.isEmpty()) {
        for (final InferredEdge edge: initialEdges) {
          final PathEdge pathEdge =
              this.inferenceGraph.getPathEdge(edge, 0d, false);
          final InferredPath path =
              this.inferenceGraph.getInferredPath(pathEdge);
          evaluatedPaths.add(new InferredPathPrediction(path,
              null, null, null, Double.NEGATIVE_INFINITY));

          final AbstractRoadTrackingFilter<?> trackingFilter =
              this.createRoadTrackingFilter();

          final OnOffEdgeTransDirMulti edgeTransDist =
              new OnOffEdgeTransDirMulti(inferenceGraph,
                  parameters.getOnTransitionProbs(),
                  parameters.getOffTransitionProbs());

          final MultivariateGaussian initialBelief =
              trackingFilter.getRoadFilter()
                  .createInitialLearnedObject();

          final double lengthLocation =
              edge.getLengthIndexedLine().project(
                  initialObservation.getObsPoint());

          final Vector stateSmpl =
              trackingFilter.sampleStateTransDist(
                  initialBelief.getMean(), this.random);
          initialBelief.setMean(stateSmpl);
          initialBelief.getMean().setElement(0,
              lengthLocation);

          final SimplePathStateBelief simplePathStateBelief =
              SimplePathStateBelief.getPathStateBelief(path,
                  initialBelief);

          final VehicleState state =
              new VehicleState(this.inferenceGraph,
                  initialObservation, trackingFilter,
                  simplePathStateBelief, edgeTransDist, null);

          /*
           * Sample an initial prior for the transition probabilities
           */
          final Vector edgePriorParams =
              state.getEdgeTransitionDist()
                  .getEdgeMotionTransProbPrior()
                  .sample(this.random);
          final Vector freeDriorParams =
              state.getEdgeTransitionDist()
                  .getFreeMotionTransProbPrior()
                  .sample(this.random);
          state.getEdgeTransitionDist()
              .getEdgeMotionTransPrior()
              .setParameters(edgePriorParams);
          state.getEdgeTransitionDist()
              .getFreeMotionTransPrior()
              .setParameters(freeDriorParams);

          final double lik =
              state.getProbabilityFunction().evaluate(
                  initialObservation);

          initialDist.increment(state, lik);
        }
      }

      /*
       * Free-motion
       */
      final AbstractRoadTrackingFilter<?> trackingFilter =
          this.createRoadTrackingFilter();

      final OnOffEdgeTransDirMulti edgeTransDist =
          new OnOffEdgeTransDirMulti(inferenceGraph,
              parameters.getOnTransitionProbs(),
              parameters.getOffTransitionProbs());

      final MultivariateGaussian initialBelief =
          trackingFilter.getGroundFilter()
              .createInitialLearnedObject();

      final Vector stateSmpl =
          trackingFilter.sampleStateTransDist(
              initialBelief.getMean(), this.random);

      initialBelief.setMean(stateSmpl);
      initialBelief.getMean().setElement(0,
          xyPoint.getElement(0));
      initialBelief.getMean().setElement(2,
          xyPoint.getElement(1));

      final SimplePathStateBelief simplePathStateBelief =
          SimplePathStateBelief.getPathStateBelief(
              this.inferenceGraph.getNullPath(), initialBelief);

      final VehicleState state =
          new VehicleState(this.inferenceGraph,
              initialObservation, trackingFilter,
              simplePathStateBelief, edgeTransDist, null);
      /*
       * Sample an initial prior for the transition probabilities
       */
      final Vector edgeDriorParams =
          state.getEdgeTransitionDist()
              .getEdgeMotionTransProbPrior()
              .sample(this.random);
      final Vector freeDriorParams =
          state.getEdgeTransitionDist()
              .getFreeMotionTransProbPrior()
              .sample(this.random);
      state.getEdgeTransitionDist()
          .getEdgeMotionTransPrior()
          .setParameters(edgeDriorParams);
      state.getEdgeTransitionDist()
          .getFreeMotionTransPrior()
          .setParameters(freeDriorParams);

      final double lik =
          state.getProbabilityFunction().evaluate(
              initialObservation);

      initialDist.increment(state, lik);

      retDist.increment(initialDist.sample(this.random));
    }

    return retDist;
  }

  @Nonnull
  abstract protected AbstractRoadTrackingFilter<?>
      createRoadTrackingFilter();

  @Nonnull
  public InferenceGraph getInferredGraph() {
    return inferenceGraph;
  }

  @Nonnull
  public GpsObservation getInitialObservation() {
    return initialObservation;
  }

  @Nonnull
  public VehicleStateInitialParameters getParameters() {
    return parameters;
  }

  public Random getRandom() {
    return random;
  }

  public long getSeed() {
    return seed;
  }

  public void setRandom(Random random) {
    this.random = random;
  }

  public void setSeed(long seed) {
    this.seed = seed;
  }

}
