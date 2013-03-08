package org.opentrackingtools.estimators;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.distributions.PathStateMultivariateMixtureDensityModel;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.StatisticsUtil;

import com.beust.jcommander.internal.Lists;
import com.google.common.base.Preconditions;
import com.google.common.primitives.Doubles;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.DiscreteDistribution;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.MultivariateMixtureDensityModel;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

/**
 * This estimator provides a predictive motion state distribution,
 * conditional on a path and one of its edges, that's centered on the path edge
 * with a variance proportional to its length.  
 * 
 * @author bwillard
 *
 */
public class PathStateEstimatorPredictor
    extends AbstractCloneableSerializable
    implements BayesianEstimatorPredictor<PathState, PathState, PathStateDistribution> {
  
  protected VehicleState<? extends GpsObservation> currentState;
  protected Path path;
  
  public PathStateEstimatorPredictor(VehicleState<? extends GpsObservation> currentState, Path path) {
    this.path = path;
    this.currentState = currentState;
  }

  @Override
  public PathStateMultivariateMixtureDensityModel<PathStateDistribution> createPredictiveDistribution(
    PathStateDistribution prior) {
    
    List<PathStateDistribution> distributions = Lists.newArrayList();
    List<Double> weights = Lists.newArrayList();
    if (this.path.isNullPath()) {
      distributions.add(new PathStateDistribution(this.path, prior.getGroundBelief()));
      weights.add(1d);
    } else {
      for (PathEdge<?> edge : this.path.getPathEdges()) {
        Preconditions.checkArgument(prior.getPathState().isOnRoad());
    
        final Matrix Or = MotionStateEstimatorPredictor.getOr();
        final double S =
            Or.times(prior.getCovariance()).times(Or.transpose())
                .getElement(0, 0)
                // + 1d;
                + Math
                    .pow(
                        edge.getLength()
                            / Math.sqrt(12), 2);
        final Matrix W =
            prior.getCovariance().times(Or.transpose()).scale(1 / S);
        final Matrix R =
            prior.getCovariance().minus(W.times(W.transpose()).scale(S));
    
        final double direction = edge.isBackward() ? -1d : 1d;
        final double mean =
            (edge.getDistToStartOfEdge() + (edge.getDistToStartOfEdge() + direction
                * edge.getLength())) / 2d;
    
        final Vector beliefMean = prior.getMean();
        final double e = mean - Or.times(beliefMean).getElement(0);
        final Vector a = beliefMean.plus(W.getColumn(0).scale(e));
    
        assert StatisticsUtil
            .isPosSemiDefinite((gov.sandia.cognition.math.matrix.mtj.DenseMatrix) R);
    
        final PathStateDistribution prediction = new PathStateDistribution(this.path, 
            new MultivariateGaussian(a, R));
        
        distributions.add(prediction);
        weights.add(this.marginalPredictiveLogLikInternal(this.path, prediction.getMotionStateDistribution(), edge));
      }
    }
    final PathStateMultivariateMixtureDensityModel<PathStateDistribution> result = 
        new PathStateMultivariateMixtureDensityModel<PathStateDistribution>(this.path, distributions);
    result.setDistributions((ArrayList)distributions);
    result.setPriorWeights(Doubles.toArray(weights));
    
    return result;
  }

  /**
   * Truncated normal mixing component.
   * 
   * @param beliefPrediction
   * @return
   */
  protected double marginalPredictiveLogLikInternal(Path path,
    MultivariateGaussian beliefPrediction, PathEdge<?> currentEdge) {

    final double direction = currentEdge.isBackward() ? -1d : 1d;
    final double thisStartDistance =
        Math.abs(currentEdge.getDistToStartOfEdge());
    final double thisEndDistance =
        currentEdge.getLength() + thisStartDistance;

    final Matrix Or = MotionStateEstimatorPredictor.getOr();

    final double var =
        Or.times(beliefPrediction.getCovariance())
            .times(Or.transpose()).getElement(0, 0);

    final double mean =
        direction
            * Or.times(beliefPrediction.getMean()).getElement(0);

    final double t1 =
        UnivariateGaussian.CDF.evaluate(thisEndDistance, mean, var)
            - UnivariateGaussian.CDF.evaluate(thisStartDistance,
                mean, var);

    final double Z =
        UnivariateGaussian.CDF.evaluate(
            Math.abs(path.getTotalPathDistance()), mean, var)
            - UnivariateGaussian.CDF.evaluate(0, mean, var);

    return Math.log(t1) - Math.log(Z);
  }
  
  @Override
  public PathStateDistribution
      learn(Collection<? extends PathState> data) {
    return null;
  }


}
