package org.opentrackingtools.estimators;

import java.util.Collection;
import java.util.Random;

import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.distributions.TruncatedRoadGaussian;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.StatisticsUtil;
import org.opentrackingtools.util.TrueObservation;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;

import gov.sandia.cognition.learning.algorithm.IncrementalLearner;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.distribution.InverseWishartDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

public class RoadMeasurementCovarianceEstimatorPredictor extends
    AbstractCloneableSerializable
    implements
    BayesianEstimatorPredictor<Matrix, Matrix, InverseWishartDistribution>,
    IncrementalLearner<Vector, InverseWishartDistribution> {

  protected VehicleStateDistribution<?> vehicleState;
  protected MotionStateEstimatorPredictor motionStateEstimator;
  protected Random rng;
  protected Vector newStateObsSample;
  
  /**
   * This estimator will learn/update a path state model covariance with an inverse Wishart
   * prior. 
   * 
   * @param vehicleState
   * @param motionStateEstimator 
   * @param rng
   */
  public RoadMeasurementCovarianceEstimatorPredictor(VehicleStateDistribution<?> vehicleState, Vector newStateObsSample) {
    Preconditions.checkState(vehicleState.getMotionStateEstimatorPredictor() != null);
    this.vehicleState = vehicleState;
    this.newStateObsSample = newStateObsSample;
  }
  
  private static final Logger log = LoggerFactory
      .getLogger(RoadMeasurementCovarianceEstimatorPredictor.class);
  
  @Override
  public void update(InverseWishartDistribution obsCovPrior, Vector obs) {
    /*
     * observation covar update
     */
    final Vector obsError =
        obs.minus(newStateObsSample);
    final Matrix obsSmplCov = 
        obsError.outerProduct(obsError);
//        MatrixFactory.getDefault().createDiagonal(
//        obsError.dotTimes(obsError));
    
    // REMOVE debug.  remove.
    if (this.vehicleState.getObservation() instanceof TrueObservation) {
      final VehicleStateDistribution<?> trueState = ((TrueObservation)this.vehicleState.getObservation()).getTrueState();
      InverseWishartDistribution tmpPrior = obsCovPrior.clone();
      updateInvWishart(tmpPrior, obsSmplCov);
      Matrix trueQ = trueState.getObservationCovarianceParam().getValue();
      final Vector trueObsError = obs.minus(trueState.getMotionStateParam().getValue());
      final Vector nonSampleObsError = obs.minus(this.vehicleState.getMotionStateParam().getValue());
      final Matrix updateError = tmpPrior.getMean().minus(trueQ);
      if (updateError.normFrobenius()
            > 0.4 * trueQ.normFrobenius()) {
        log.warn("Large update error: " + updateError);
//        log.warn("True state:" + trueState);
      }
    }
    
    updateInvWishart(obsCovPrior, obsSmplCov);

  }

  private void updateInvWishart(
    InverseWishartDistribution covarPrior, Matrix smplCov) {
    final int nOld = covarPrior.getDegreesOfFreedom();
    final int nNew = nOld + 1;
    covarPrior.setDegreesOfFreedom(nNew);
    covarPrior.setInverseScale(covarPrior.getInverseScale()
        .plus(smplCov));
  }

  @Override
  public InverseWishartDistribution createInitialLearnedObject() {
    return new InverseWishartDistribution(2);
  }

  @Override
  public void update(InverseWishartDistribution target,
    Iterable<? extends Vector> data) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public InverseWishartDistribution learn(
    Collection<? extends Matrix> data) {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public ComputableDistribution<Matrix> createPredictiveDistribution(
    InverseWishartDistribution posterior) {
    return posterior;
  }

}
