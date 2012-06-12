package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.bayesian.conjugate.MultinomialBayesianEstimator;
import gov.sandia.cognition.statistics.distribution.DirichletDistribution;
import gov.sandia.cognition.statistics.distribution.MultinomialDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariatePolyaDistribution;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.List;
import java.util.Random;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

/**
 * Class representing the transition from one edge to another. For now we use
 * three transition types: 1. off-road to off-road/on-road to on-road 2.
 * off-road to on-road 3. on-road to off-road
 * 
 * @author bwillard
 * 
 */
public class EdgeTransitionDistributions extends
    AbstractCloneableSerializable {

  private static final long serialVersionUID = -8329433263373783485L;

  /*
   * Distribution corresponding to free-movement -> free-movement and
   * free-movement -> edge-movement
   */
  DirichletDistribution freeMotionTransProbPrior;
  MultinomialDistribution freeMotionTransPrior = new MultinomialDistribution(
      2, 1);
  MultinomialBayesianEstimator freeMotionTransEstimator = new MultinomialBayesianEstimator(
      freeMotionTransPrior, freeMotionTransProbPrior);

  /*
   * Distribution corresponding to edge-movement -> free-movement and
   * edge-movement -> edge-movement
   */
  DirichletDistribution edgeMotionTransProbPrior;
  MultinomialDistribution edgeMotionTransPrior = new MultinomialDistribution(
      2, 1);
  MultinomialBayesianEstimator edgeMotionTransEstimator = new MultinomialBayesianEstimator(
      freeMotionTransPrior, freeMotionTransProbPrior);

  private final InferredGraph graph;

  private static final Vector stateOffToOff = VectorFactory
      .getDefault().copyValues(1d, 0d);
  private static final Vector stateOffToOn = VectorFactory
      .getDefault().copyValues(0d, 1d);

  private static final Vector stateOnToOn = VectorFactory
      .getDefault().copyValues(1d, 0d);
  private static final Vector stateOnToOff = VectorFactory
      .getDefault().copyValues(0d, 1d);

  public EdgeTransitionDistributions(InferredGraph graph,
    Vector edgeMotionPriorParams, Vector freeMotionPriorParams) {
    this.graph = graph;
    this.freeMotionTransProbPrior = new DirichletDistribution(
        freeMotionPriorParams);
    this.edgeMotionTransProbPrior = new DirichletDistribution(
        edgeMotionPriorParams);
    /*
     * Start by setting the means.
     */
    freeMotionTransPrior.setParameters(freeMotionTransProbPrior
        .getMean());
    edgeMotionTransPrior.setParameters(edgeMotionTransProbPrior
        .getMean());
  }

  @Override
  public EdgeTransitionDistributions clone() {
    return (EdgeTransitionDistributions) super.clone();
  }

  public double evaluate(InferredEdge from, InferredEdge to) {
    if (from == InferredEdge.getEmptyEdge()) {
      return freeMotionTransPrior.getProbabilityFunction().evaluate(
          getTransitionType(from, to));
    } else {
      return edgeMotionTransPrior.getProbabilityFunction().evaluate(
          getTransitionType(from, to));
    }
  }

  public double logEvaluate(InferredEdge from, InferredEdge to) {
    if (from == InferredEdge.getEmptyEdge()) {
      return freeMotionTransPrior.getProbabilityFunction().logEvaluate(
          getTransitionType(from, to));
    } else {
      return edgeMotionTransPrior.getProbabilityFunction().logEvaluate(
          getTransitionType(from, to));
    }
  }

  public double predictiveLogLikelihood(InferredEdge from,
    InferredEdge to) {
    final Vector state = getTransitionType(from, to);
    if (from == InferredEdge.getEmptyEdge()) {
      final MultivariatePolyaDistribution predDist = freeMotionTransEstimator
          .createPredictiveDistribution(freeMotionTransProbPrior);
      return predDist.getProbabilityFunction().logEvaluate(state);
    } else {
      final MultivariatePolyaDistribution predDist = edgeMotionTransEstimator
          .createPredictiveDistribution(edgeMotionTransProbPrior);
      return predDist.getProbabilityFunction().logEvaluate(state);
    }
  }

  public InferredEdge sample(Random rng,
    List<InferredEdge> transferEdges, InferredEdge currentEdge) {

    Preconditions.checkArgument(!transferEdges.contains(InferredGraph
        .getEmptyEdge()));
    Preconditions.checkNotNull(currentEdge);
    Preconditions.checkNotNull(transferEdges);

    if (currentEdge == InferredGraph.getEmptyEdge()) {
      /*
       * We're currently in free-motion. If there are transfer edges, then
       * sample from those.
       */
      if (transferEdges.isEmpty()) {
        return InferredGraph.getEmptyEdge();
      } else {
        final Vector sample = this.freeMotionTransPrior.sample(rng);

        if (sample.equals(stateOffToOn)) {
          return transferEdges.get(rng.nextInt(transferEdges.size()));
        } else {
          return InferredGraph.getEmptyEdge();
        }
      }
    } else {
      /*
       * We're on an edge, so sample whether we go off-road, or transfer/stay
       * on.
       */
      final Vector sample = this.edgeMotionTransPrior.sample(rng);

      if (sample.equals(stateOnToOff) || transferEdges.isEmpty()) {
        return InferredGraph.getEmptyEdge();
      } else {
        final List<InferredEdge> support = Lists
            .newArrayList(transferEdges);
        return support.get(rng.nextInt(transferEdges.size()));
      }

    }

  }

  @Override
  public String toString() {
    return "EdgeTransitionDistributions [freeMotionTransPrior="
        + freeMotionTransPrior.getParameters()
        + ", edgeMotionTransPrior="
        + edgeMotionTransPrior.getParameters() + "]";
  }

  public void update(InferredEdge from, InferredEdge to) {
    final Vector transType = getTransitionType(from, to);
    if (from == InferredEdge.getEmptyEdge()) {
      freeMotionTransEstimator.update(
          freeMotionTransProbPrior, transType);
    } else {
      edgeMotionTransEstimator.update(
          edgeMotionTransProbPrior, transType);
    }
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

  public static Vector getTransitionType(InferredEdge from,
    InferredEdge to) {
    if (from == InferredGraph.getEmptyEdge()) {
      if (to == InferredGraph.getEmptyEdge()) {
        return stateOffToOff;
      } else {
        return stateOffToOn;
      }
    } else {
      if (to != InferredGraph.getEmptyEdge()) {
        return stateOnToOn;
      } else {
        return stateOnToOff;
      }
    }
  }

}
