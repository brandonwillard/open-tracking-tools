package org.opentrackingtools.statistics.filters.vehicles.road.impl;

import java.util.Collection;

import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.graph.paths.util.PathUtils;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.statistics.distributions.impl.DefaultCountedDataDistribution;
import org.opentrackingtools.statistics.impl.StatisticsUtil;

import com.google.common.base.Preconditions;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.bayesian.RecursiveBayesianEstimator;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.ObjectUtil;

public class PathStateDistributionEstimator
    implements
    RecursiveBayesianEstimator<PathStateBelief, Vector, MultivariateGaussian>,
    BayesianEstimatorPredictor<PathStateBelief, Vector, MultivariateGaussian> {

  private InferenceGraph graph;
  private GpsObservation obs;
  private InferredPath path;
  private VehicleState state;
  
  public PathStateDistributionEstimator() {
  }
  
  public PathStateDistributionEstimator(InferenceGraph graph, GpsObservation obs, 
    InferredPath path, VehicleState state) {
    this.graph = graph;
    this.obs = obs;
    this.path = path;
    this.state = state;
  }

  @Override
  public PathStateDistributionEstimator clone() {
    PathStateDistributionEstimator clone = (PathStateDistributionEstimator) 
       ObjectUtil.cloneSmart(this);
    return clone;
  }
  
  @Override
  public ComputableDistribution<PathStateBelief>
      createPredictiveDistribution(MultivariateGaussian posterior) {
    DefaultCountedDataDistribution<PathStateBelief> result = 
        new DefaultCountedDataDistribution<PathStateBelief>(true);
    for (PathEdge edge : this.path.getPathEdges()) {
      final PathStateBelief pathBelief;
      double totalLikelihood = Double.NEGATIVE_INFINITY;
      if (edge.isNullEdge()) {
        final MultivariateGaussian conditionalState = posterior;
        pathBelief = this.path.getStateBeliefOnPath(conditionalState);
      } else {
        final MultivariateGaussian conditionalState = getEdgePredictive(edge, posterior, obs);
        pathBelief = this.path.getStateBeliefOnPath(conditionalState);
        
        final double measurementPredLik =
            pathBelief.priorPredictiveLogLikelihood(
                obs.getProjectedPoint(),
                this.state.getMovementFilter());
  
        final double edgePredTransLogLik =
            this.state.getEdgeTransitionDist()
                .logEvaluate(
                    this.state.getBelief().getEdge()
                        .getInferredEdge(),
                        edge.getInferredEdge());
        totalLikelihood += measurementPredLik + edgePredTransLogLik;
      }
      
      result.increment(pathBelief, totalLikelihood);
    }
    return result;
  }
  
  
  private MultivariateGaussian getEdgePredictive(PathEdge edge, 
    MultivariateGaussian belief, GpsObservation obs) {

    Preconditions.checkArgument(belief.getMean().getDimensionality() == 2);
    Preconditions.checkArgument(!edge.isNullEdge());

    /*-
     * TODO really, this should just be the truncated/conditional
     * mean and covariance for the given interval/edge
     */
    final Matrix Or = AbstractRoadTrackingFilter.getOr();
    final double S =
        Or.times(belief.getCovariance())
            .times(Or.transpose()).getElement(0, 0)
            // + 1d;
            + Math.pow(edge.getLength()
                / Math.sqrt(12), 2);
    final Matrix W =
        belief.getCovariance().times(Or.transpose())
            .scale(1 / S);
    final Matrix R =
        belief.getCovariance().minus(
            W.times(W.transpose()).scale(S));

    final double direction = edge.isBackward() ? -1d : 1d;
    final double mean =
        (edge.getDistToStartOfEdge() + (edge
            .getDistToStartOfEdge() + direction
            * edge.getLength())) / 2d;

    final Vector beliefMean = belief.getMean();
    final double e =
        mean - Or.times(beliefMean).getElement(0);
    final Vector a =
        beliefMean.plus(W.getColumn(0).scale(e));

    assert StatisticsUtil
        .isPosSemiDefinite((gov.sandia.cognition.math.matrix.mtj.DenseMatrix) R);

    final MultivariateGaussian prediction = new MultivariateGaussian(a, R);
    
    return prediction;
  }

  @Override
  public MultivariateGaussian learn(
    Collection<? extends PathStateBelief> data) {
    return null;
  }

  @Override
  public MultivariateGaussian createInitialLearnedObject() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void update(MultivariateGaussian target, PathStateBelief data) {
//    final PathEdge edge = data.getEdge();
//    if (data.isOnRoad()) {
//      final MultivariateGaussian obsProj =
//          PathUtils.getRoadObservation(
//              this.obs, this.state.getObservationCov(), data.getPath(), edge);
//
//      this.state.roadFilter.setMeasurementCovariance(obsProj
//          .getCovariance());
//      
//      /*
//       * Clamp the projected obs
//       */
//      if (!data.getPath().isOnPath(obsProj.getMean().getElement(0))) {
//        obsProj.getMean().setElement(0, 
//            data.getPath().clampToPath(
//                obsProj.getMean().getElement(0)));
//      }
//      
//      this.state.roadFilter.measure(target, obsProj.getMean());
//
//    } else {
//      this.state.groundFilter.measure(target, this.obs);
//    }
  }

  @Override
  public void update(MultivariateGaussian target,
    Iterable<? extends PathStateBelief> data) {
    // TODO Auto-generated method stub
    
  }

}
