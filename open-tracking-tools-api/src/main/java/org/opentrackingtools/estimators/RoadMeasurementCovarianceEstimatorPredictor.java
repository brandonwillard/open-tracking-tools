package org.opentrackingtools.estimators;

import gov.sandia.cognition.learning.algorithm.IncrementalLearner;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.distribution.InverseWishartDistribution;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Collection;
import java.util.Random;

import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.util.TrueObservation;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.base.Preconditions;

public class RoadMeasurementCovarianceEstimatorPredictor extends
    AbstractCloneableSerializable
    implements
    BayesianEstimatorPredictor<Matrix, Matrix, InverseWishartDistribution>,
    IncrementalLearner<Vector, InverseWishartDistribution> {

  private static final Logger log = LoggerFactory
      .getLogger(RoadMeasurementCovarianceEstimatorPredictor.class);
  /**
   * 
   */
  private static final long serialVersionUID = -6159036631088182363L;
  protected MotionStateEstimatorPredictor motionStateEstimator;
  protected Vector newStateObsSample;
  protected Random rng;

  protected VehicleStateDistribution<?> vehicleState;

  /**
   * This estimator will learn/update a path state model covariance with an
   * inverse Wishart prior.
   * 
   * @param vehicleState
   * @param motionStateEstimator
   * @param rng
   */
  public RoadMeasurementCovarianceEstimatorPredictor(
    VehicleStateDistribution<?> vehicleState, Vector newStateObsSample) {
    Preconditions.checkState(vehicleState
        .getMotionStateEstimatorPredictor() != null);
    this.vehicleState = vehicleState;
    this.newStateObsSample = newStateObsSample;
  }

  @Override
  public InverseWishartDistribution createInitialLearnedObject() {
    return new InverseWishartDistribution(2);
  }

  @Override
  public ComputableDistribution<Matrix> createPredictiveDistribution(
    InverseWishartDistribution posterior) {
    return posterior;
  }

  @Override
  public InverseWishartDistribution learn(
    Collection<? extends Matrix> data) {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void update(InverseWishartDistribution target,
    Iterable<? extends Vector> data) {
    // TODO Auto-generated method stub

  }

  @Override
  public void update(InverseWishartDistribution obsCovPrior,
    Vector obs) {
    /*
     * observation covar update
     */
    final Vector obsError = obs.minus(this.newStateObsSample);
    final Matrix obsSmplCov = obsError.outerProduct(obsError);
    //        MatrixFactory.getDefault().createDiagonal(
    //        obsError.dotTimes(obsError));

    // REMOVE debug.  remove.
    if (this.vehicleState.getObservation() instanceof TrueObservation) {
      final VehicleStateDistribution<?> trueState =
          ((TrueObservation) this.vehicleState.getObservation())
              .getTrueState();
      final InverseWishartDistribution tmpPrior = obsCovPrior.clone();
      this.updateInvWishart(tmpPrior, obsSmplCov);
      final Matrix trueQ =
          trueState.getObservationCovarianceParam().getValue();
      obs.minus(trueState.getMotionStateParam().getValue());
      obs.minus(this.vehicleState.getMotionStateParam().getValue());
      final Matrix updateError = tmpPrior.getMean().minus(trueQ);
      if (updateError.normFrobenius() > 0.4 * trueQ.normFrobenius()) {
        RoadMeasurementCovarianceEstimatorPredictor.log
            .warn("Large update error: " + updateError);
        //        log.warn("True state:" + trueState);
      }
    }

    this.updateInvWishart(obsCovPrior, obsSmplCov);

  }

  private void updateInvWishart(
    InverseWishartDistribution covarPrior, Matrix smplCov) {
    final int nOld = covarPrior.getDegreesOfFreedom();
    final int nNew = nOld + 1;
    covarPrior.setDegreesOfFreedom(nNew);
    covarPrior.setInverseScale(covarPrior.getInverseScale().plus(
        smplCov));
  }

}
