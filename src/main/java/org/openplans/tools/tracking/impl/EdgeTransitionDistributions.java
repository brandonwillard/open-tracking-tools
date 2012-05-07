package org.openplans.tools.tracking.impl;


import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.bayesian.conjugate.MultinomialBayesianEstimator;
import gov.sandia.cognition.statistics.distribution.DirichletDistribution;
import gov.sandia.cognition.statistics.distribution.MultinomialDistribution;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

/**
 * Class representing the transition from one edge to another.
 * For now we use three transition types:
 * 1. off-road to off-road/on-road to on-road
 * 2. off-road to on-road
 * 3. on-road to off-road
 * 
 * @author bwillard
 *
 */
public class EdgeTransitionDistributions extends AbstractCloneableSerializable {
  
  private static final long serialVersionUID = -8329433263373783485L;
  
  DirichletDistribution edgeTransProbPrior = new DirichletDistribution(
      VectorFactory.getDefault().copyValues(1d, 0.5, 1d));
  MultinomialDistribution edgeTransPrior = new MultinomialDistribution(3, 1);
  MultinomialBayesianEstimator edgeTransEstimator = 
      new MultinomialBayesianEstimator(edgeTransPrior, edgeTransProbPrior);
  
  private static final Vector state1 = VectorFactory.getDefault().copyValues(1d, 0d, 0d);
  private static final Vector state2 = VectorFactory.getDefault().copyValues(0d, 1d, 0d);
  private static final Vector state3 = VectorFactory.getDefault().copyValues(0d, 0d, 1d);
  
  public static Vector getTransitionType(InferredEdge from, InferredEdge to) {
    if (from == InferredGraph.getEmptyEdge()) {
      if (to == InferredGraph.getEmptyEdge()) {
        return state1;
      } else {
        return state2;
      }
    } else {
      if (to != InferredGraph.getEmptyEdge()) {
        return state1;
      } else {
        return state3;
      }
    }
  }
  
  public void update(InferredEdge from, InferredEdge to) {
    Vector transType = getTransitionType(from, to);
    edgeTransEstimator.update(edgeTransProbPrior, transType);
  }
  
  public double evaluate(InferredEdge from, InferredEdge to) {
    return edgeTransPrior.getProbabilityFunction().evaluate(getTransitionType(from, to));
  }
  
  public double logEvaluate(InferredEdge from, InferredEdge to) {
    return edgeTransPrior.getProbabilityFunction().logEvaluate(getTransitionType(from, to));
  }

  public DirichletDistribution getEdgeTransProbPrior() {
    return edgeTransProbPrior;
  }

  public MultinomialDistribution getEdgeTransPrior() {
    return edgeTransPrior;
  }

  public MultinomialBayesianEstimator getEdgeTransEstimator() {
    return edgeTransEstimator;
  }

  public static Vector getState1() {
    return state1;
  }

  public static Vector getState2() {
    return state2;
  }

  public static Vector getState3() {
    return state3;
  }

  @Override
  public EdgeTransitionDistributions clone() {
    return (EdgeTransitionDistributions) super.clone();
  }
  
}
