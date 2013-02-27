package org.opentrackingtools.distributions;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseVector;
import gov.sandia.cognition.statistics.ClosedFormComputableDiscreteDistribution;
import gov.sandia.cognition.statistics.ClosedFormComputableDistribution;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.bayesian.BayesianParameter;
import gov.sandia.cognition.statistics.bayesian.conjugate.MultinomialBayesianEstimator;
import gov.sandia.cognition.statistics.distribution.DirichletDistribution;
import gov.sandia.cognition.statistics.distribution.MultinomialDistribution;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.opentrackingtools.estimators.RecursiveBayesianEstimatorPredictor;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.util.StatisticsUtil;

import com.google.common.base.Preconditions;
import com.google.common.base.Predicate;
import com.google.common.collect.Iterators;
import com.google.common.collect.Lists;

/**
 * Class representing the transition from one edge to another. For now we use
 * three transition types: 1. off-road to off-road/on-road to on-road 2.
 * off-road to on-road 3. on-road to off-road
 * 
 * @author bwillard
 * 
 */
public class OnOffEdgeTransDistribution<E extends InferenceGraphEdge>
    extends AbstractCloneableSerializable implements
    ProbabilityFunction<E>, Comparable<OnOffEdgeTransDistribution<E>> {

  private static final long serialVersionUID = -8329433263373783485L;

  /*
   * Distribution corresponding to free-movement -> free-movement and
   * free-movement -> edge-movement
   */
  protected DirichletDistribution freeMotionTransProbPrior;
  protected MultinomialDistribution freeMotionTransPrior =
      new MultinomialDistribution(2, 1);

  /*
   * Distribution corresponding to edge-movement -> free-movement and
   * edge-movement -> edge-movement
   */
  protected DirichletDistribution edgeMotionTransProbPrior;
  protected MultinomialDistribution edgeMotionTransPrior =
      new MultinomialDistribution(2, 1);
  protected VehicleState<? extends GpsObservation> currentState;

  private final InferenceGraph graph;

  private static final Vector stateOffToOff = VectorFactory
      .getDefault().copyValues(1d, 0d);
  private static final Vector stateOffToOn = VectorFactory
      .getDefault().copyValues(0d, 1d);

  private static final Vector stateOnToOn = VectorFactory
      .getDefault().copyValues(1d, 0d);
  private static final Vector stateOnToOff = VectorFactory
      .getDefault().copyValues(0d, 1d);

  private static final Double zeroTolerance = 1e-6;

  /**
   * Initialize prior from the given hyper prior parameters, using the hyper
   * prior means (i.e. the given parameters).
   * 
   * @param graph
   * @param edgeMotionPriorParams
   * @param freeMotionPriorParams
   * @param rng
   */
  public OnOffEdgeTransDistribution(InferenceGraph graph,
    Vector edgeMotionPriorParams, Vector freeMotionPriorParams) {
    this.graph = graph;
    this.setFreeMotionTransProbPrior(new DirichletDistribution(
        freeMotionPriorParams));
    this.setEdgeMotionTransProbPrior(new DirichletDistribution(
        edgeMotionPriorParams));
    /*
     * Start by setting the means.
     */
    getFreeMotionTransProb().setParameters(
        getFreeMotionTransProbPrior().getMean());
    getEdgeMotionTransProb().setParameters(
        getEdgeMotionTransProbPrior().getMean());
  }

  public OnOffEdgeTransDistribution(InferenceGraph inferenceGraph,
    Vector onTransitionProbs, Vector offTransitionProbs, Random random) {
    this(inferenceGraph, onTransitionProbs, offTransitionProbs);

    /*
     * Sample an initial prior for the transition probabilities
     */
    final Vector edgePriorParams =
        OnOffEdgeTransDistribution.checkedSample(
            this.getEdgeMotionTransProbPrior(), random);
    final Vector freeDriorParams =
        OnOffEdgeTransDistribution.checkedSample(
            this.getFreeMotionTransProbPrior(), random);
    this.getEdgeMotionTransProb().setParameters(edgePriorParams);
    this.getFreeMotionTransProb().setParameters(freeDriorParams);
  }

  @Override
  public OnOffEdgeTransDistribution<E> clone() {
    final OnOffEdgeTransDistribution<E> transDist =
        (OnOffEdgeTransDistribution<E>) super.clone();
    transDist.setEdgeMotionTransPrior(this.getEdgeMotionTransProb()
        .clone());
    transDist.setFreeMotionTransPrior(this.getFreeMotionTransProb()
        .clone());
    transDist.setEdgeMotionTransProbPrior(this
        .getEdgeMotionTransProbPrior().clone());
    transDist.setFreeMotionTransProbPrior(this
        .getFreeMotionTransProbPrior().clone());
    return transDist;
  }

  @Override
  public int compareTo(OnOffEdgeTransDistribution<E> o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(((DenseVector) this.getEdgeMotionTransProb()
        .getParameters()).getArray(), ((DenseVector) o
        .getEdgeMotionTransProb().getParameters()).getArray());
    comparator.append(((DenseVector) this.getFreeMotionTransProb()
        .getParameters()).getArray(), ((DenseVector) o
        .getFreeMotionTransProb().getParameters()).getArray());

    comparator.append(((DenseVector) this
        .getEdgeMotionTransProbPrior().getParameters()).getArray(),
        ((DenseVector) o.getEdgeMotionTransProbPrior()
            .getParameters()).getArray());
    comparator.append(((DenseVector) this
        .getFreeMotionTransProbPrior().getParameters()).getArray(),
        ((DenseVector) o.getFreeMotionTransProbPrior()
            .getParameters()).getArray());

    return comparator.toComparison();
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
    final OnOffEdgeTransDistribution other =
        (OnOffEdgeTransDistribution) obj;
    if (getEdgeMotionTransProb() == null) {
      if (other.getEdgeMotionTransProb() != null) {
        return false;
      }
    } else if (!StatisticsUtil.vectorEquals(getEdgeMotionTransProb()
        .getParameters(), other.getEdgeMotionTransProb()
        .getParameters())) {
      return false;
    }
    if (getEdgeMotionTransProbPrior() == null) {
      if (other.getEdgeMotionTransProbPrior() != null) {
        return false;
      }
    } else if (!StatisticsUtil.vectorEquals(
        getEdgeMotionTransProbPrior().getParameters(), other
            .getEdgeMotionTransProbPrior().getParameters())) {
      return false;
    }
    if (getFreeMotionTransProb() == null) {
      if (other.getFreeMotionTransProb() != null) {
        return false;
      }
    } else if (!StatisticsUtil.vectorEquals(getFreeMotionTransProb()
        .getParameters(), other.getFreeMotionTransProb()
        .getParameters())) {
      return false;
    }
    if (getFreeMotionTransProbPrior() == null) {
      if (other.getFreeMotionTransProbPrior() != null) {
        return false;
      }
    } else if (!StatisticsUtil.vectorEquals(
        getFreeMotionTransProbPrior().getParameters(), other
            .getFreeMotionTransProbPrior().getParameters())) {
      return false;
    }
    return true;
  }

  public double evaluate(InferenceGraphEdge from,
    InferenceGraphEdge to) {
    if (from.isNullEdge()) {
      return getFreeMotionTransProb().getProbabilityFunction()
          .evaluate(getTransitionType(from, to));
    } else {
      return getEdgeMotionTransProb().getProbabilityFunction()
          .evaluate(getTransitionType(from, to));
    }
  }

  public MultinomialDistribution getEdgeMotionTransProb() {
    return edgeMotionTransPrior;
  }

  public DirichletDistribution getEdgeMotionTransProbPrior() {
    return edgeMotionTransProbPrior;
  }

  public MultinomialDistribution getFreeMotionTransProb() {
    return freeMotionTransPrior;
  }

  public DirichletDistribution getFreeMotionTransProbPrior() {
    return freeMotionTransProbPrior;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime
            * result
            + ((getEdgeMotionTransProb() == null) ? 0
                : StatisticsUtil
                    .hashCodeVector(getEdgeMotionTransProb()
                        .getParameters()));
    result =
        prime
            * result
            + ((getEdgeMotionTransProbPrior() == null) ? 0
                : StatisticsUtil
                    .hashCodeVector(getEdgeMotionTransProbPrior()
                        .getParameters()));
    result =
        prime
            * result
            + ((getFreeMotionTransProb() == null) ? 0
                : StatisticsUtil
                    .hashCodeVector(getFreeMotionTransProb()
                        .getParameters()));
    result =
        prime
            * result
            + ((getFreeMotionTransProbPrior() == null) ? 0
                : StatisticsUtil
                    .hashCodeVector(getFreeMotionTransProbPrior()
                        .getParameters()));
    return result;
  }

  @Override
  public double logEvaluate(E to) {

    if (from == null) {

      final double totalProb =
          Math.log(getFreeMotionTransProb().getParameters().sum()
              + getEdgeMotionTransProb().getParameters().sum());
      if (to.isNullEdge()) {
        return LogMath
            .add(
                getFreeMotionTransProb().getProbabilityFunction()
                    .logEvaluate(
                        OnOffEdgeTransDistribution.stateOffToOff),
                getEdgeMotionTransProb().getProbabilityFunction()
                    .logEvaluate(
                        OnOffEdgeTransDistribution.stateOnToOff))
            - totalProb;
      } else {
        return LogMath
            .add(
                getFreeMotionTransProb().getProbabilityFunction()
                    .logEvaluate(
                        OnOffEdgeTransDistribution.stateOffToOn),
                getEdgeMotionTransProb().getProbabilityFunction()
                    .logEvaluate(
                        OnOffEdgeTransDistribution.stateOnToOn))
            - totalProb;
      }
    } else {
      if (from.isNullEdge()) {
        return getFreeMotionTransProb().getProbabilityFunction()
            .logEvaluate(getTransitionType(from, to));
      } else {
        return getEdgeMotionTransProb().getProbabilityFunction()
            .logEvaluate(getTransitionType(from, to));
      }
    }
  }

  /**
   * This method restricts sampling when probabilities have collapsed on one
   * value in the domain. It allows us to emulate a deterministic distribution
   * and eliminate on- or off-roading. Also, it prevents Foundry's gamma sampler
   * from taking forever.
   * 
   * @param dist
   * @param rng
   * @return
   */
  public static Vector checkedSample(
    final ClosedFormComputableDistribution<Vector> dist, Random rng) {

    if (dist instanceof ClosedFormComputableDiscreteDistribution) {
      ClosedFormComputableDiscreteDistribution<Vector> distDiscrete =
          (ClosedFormComputableDiscreteDistribution<Vector>) dist;
      Vector one =
          Iterators.find(distDiscrete.getDomain().iterator(),
              new Predicate<Vector>() {
                @Override
                public boolean apply(Vector input) {
                  return Math.abs(1 - dist.getProbabilityFunction()
                      .evaluate(input)) <= zeroTolerance;
                }
              }, null);

      return one == null ? dist.sample(rng) : one;

    } else if (dist instanceof DirichletDistribution) {
      for (int i = 0; i < dist.getMean().getDimensionality(); i++) {
        final double prob = dist.getMean().getElement(i);
        if (Double.isNaN(prob) || Math.abs(1 - prob) <= zeroTolerance) {
          Vector result =
              VectorFactory.getDefault().createVector(
                  dist.getMean().getDimensionality());
          result.setElement(i, 1d);
          return result;
        }
      }
    }

    return dist.sample(rng);
  }

  public void setEdgeMotionTransPrior(
    MultinomialDistribution edgeMotionTransPrior) {
    this.edgeMotionTransPrior = edgeMotionTransPrior;
  }

  public void setEdgeMotionTransProbPrior(
    DirichletDistribution edgeMotionTransProbPrior) {
    this.edgeMotionTransProbPrior = edgeMotionTransProbPrior;
  }

  public void setFreeMotionTransPrior(
    MultinomialDistribution freeMotionTransPrior) {
    this.freeMotionTransPrior = freeMotionTransPrior;
  }

  public void setFreeMotionTransProbPrior(
    DirichletDistribution freeMotionTransProbPrior) {
    this.freeMotionTransProbPrior = freeMotionTransProbPrior;
  }

  @Override
  public String toString() {
    return "EdgeTransitionDistributions [freeMotionTransPrior="
        + getFreeMotionTransProb().getParameters()
        + ", edgeMotionTransPrior="
        + getEdgeMotionTransProb().getParameters() + "]";
  }

  public static Vector getStateOffToOff() {
    return stateOffToOff;
  }

  public static Vector getStateOffToOn() {
    return stateOffToOn;
  }

  public static Vector getStateOnToOff() {
    return stateOnToOff;
  }

  public static Vector getStateOnToOn() {
    return stateOnToOn;
  }

  public static Vector getTransitionType(InferenceGraphEdge from,
    InferenceGraphEdge to) {
    if (from.isNullEdge()) {
      if (to.isNullEdge()) {
        return stateOffToOff;
      } else {
        return stateOffToOn;
      }
    } else {
      if (!to.isNullEdge()) {
        return stateOnToOn;
      } else {
        return stateOnToOff;
      }
    }
  }

  @Override
  public E sample(Random random) {
    final E inferredEdge =
        Preconditions.checkNotNull(this.);
    Preconditions.checkNotNull(transferEdges);

    if (inferredEdge.isNullEdge()) {
      /*
       * We're currently in free-motion. If there are transfer edges, then
       * sample from those.
       */
      if (transferEdges.isEmpty()) {
        return graph.getNullInferredEdge();
      } else {
        final Vector sample =
            checkedSample(this.getFreeMotionTransProb(), rng);

        if (sample.equals(stateOffToOn)) {
          final List<InferenceGraphEdge> support =
              Lists.newArrayList(transferEdges);
          support.remove(graph.getNullInferredEdge());
          return support.get(rng.nextInt(support.size()));
        } else {
          return graph.getNullInferredEdge();
        }
      }
    } else {
      /*
       * We're on an edge, so sample whether we go off-road, or transfer/stay
       * on.
       * If the empty edge is contained in the support (transferEdges)
       * then we sample for that.
       */
      final Vector sample =
          transferEdges.contains(graph.getNullInferredEdge())
              ? checkedSample(this.getEdgeMotionTransProb(), rng)
              : stateOnToOn;

      if (sample.equals(stateOnToOff) || transferEdges.isEmpty()) {
        return graph.getNullInferredEdge();
      } else {
        final List<InferenceGraphEdge> support =
            Lists.newArrayList(transferEdges);
        support.remove(graph.getNullInferredEdge());
        return support.get(rng.nextInt(support.size()));
      }
    }
  }

  @Override
  public ArrayList<E> sample(Random random, int numSamples) {
    ArrayList<E> samples = Lists.newArrayList();
    for (int i = 0; i < numSamples; i++) {
      samples.add(sample(random));
    }
    return samples;
  }

  public static class OnOffEdgeTransitionEstimatorPredictor<E extends InferenceGraphEdge>
      extends AbstractCloneableSerializable
      implements
      RecursiveBayesianEstimatorPredictor<E, OnOffEdgeTransDistribution<E>> {

    MultinomialBayesianEstimator estimator;

    protected BayesianParameter<E, OnOffEdgeTransDistribution<E>, OnOffEdgeTransDistribution<E>> fromParameter;

    private VehicleState<? extends GpsObservation> currentState;

    public <V extends VehicleState<? extends GpsObservation>> OnOffEdgeTransitionEstimatorPredictor(
      V currentState) {
      MultinomialDistribution conditionalDist;
      DirichletDistribution priorDist;
      if (currentState.getPathStateParam() .isOnRoad()) {
        conditionalDist =
            currentState.getEdgeTransitionParam()
                .getConditionalDistribution()
                .getEdgeMotionTransProb();
        priorDist =
            currentState.getEdgeTransitionParam()
                .getConditionalDistribution()
                .getEdgeMotionTransProbPrior();
      } else {
        conditionalDist =
            currentState.getEdgeTransitionParam()
                .getConditionalDistribution()
                .getFreeMotionTransProb();
        priorDist =
            currentState.getEdgeTransitionParam()
                .getConditionalDistribution()
                .getFreeMotionTransProbPrior();
      }
      this.estimator =
          new MultinomialBayesianEstimator(conditionalDist, priorDist);
      this.currentState = currentState;
    }

    @Override
    public OnOffEdgeTransDistribution<E> learn(
      Collection<? extends E> data) {
      return null;
    }

    /**
     * @see OnOffEdgeTransitionEstimatorPredictor#update(OnOffEdgeTransDistribution,
     *      InferenceGraphEdge)
     */
    @Override
    public void update(OnOffEdgeTransDistribution<E> target,
      Iterable<? extends E> to) {
      for (E toEdge : to) {
        update(target, toEdge);
      }
    }

    /**
     * This method updates the target with the conditional assumptions within
     * this estimator.
     */
    @Override
    public void
        update(OnOffEdgeTransDistribution<E> target, E toEdge) {
      final Vector transType =
          getTransitionType(fromParameter.getValue(), toEdge);
      if (fromParameter.getValue().isNullEdge()) {
        estimator.update(target.getFreeMotionTransProbPrior(),
            transType);
        target.getFreeMotionTransProb().setParameters(
            target.getFreeMotionTransProbPrior().getMean());
      } else {
        estimator.update(target.getEdgeMotionTransProbPrior(),
            transType);
        target.getEdgeMotionTransProb().setParameters(
            target.getEdgeMotionTransProbPrior().getMean());
      }
    }

    @Override
    public OnOffEdgeTransDistribution<E>
        createPredictiveDistribution(
          OnOffEdgeTransDistribution<E> posterior) {
      // TODO what to do?
      return null;
      //      if (fromParameter.getValue().isNullEdge()) {
      //        estimator.createPredictiveDistribution(posterior.freeMotionTransProbPrior);
      //      } else {
      //        estimator.createPredictiveDistribution(posterior.edgeMotionTransProbPrior);
      //      }
    }

    @Override
    public OnOffEdgeTransDistribution<E> createInitialLearnedObject() {
      // TODO where do we get the initial config values?  the vehicle state, naturally.
      return new OnOffEdgeTransDistribution<E>(
          this.currentState.getGraph(), null, null);
    }
  }
  

  @Override
  public ProbabilityFunction<E> getProbabilityFunction() {
    // TODO Auto-generated method stub
    return null;
  }
  

  @Override
  public Double evaluate(E input) {
    return Math.exp(this.logEvaluate(input));
  }


}
