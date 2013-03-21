package org.opentrackingtools.estimators;

import java.util.Collection;
import java.util.Random;

import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.distributions.TruncatedRoadGaussian;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
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
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractKalmanFilter;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.bayesian.KalmanFilter;
import gov.sandia.cognition.statistics.distribution.InverseWishartDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

public class RoadModelCovarianceEstimatorPredictor extends
    AbstractCloneableSerializable
    implements
    BayesianEstimatorPredictor<Matrix, Matrix, InverseWishartDistribution>,
    IncrementalLearner<Vector, InverseWishartDistribution> {

  protected VehicleStateDistribution<?> vehicleState;
  protected MotionStateEstimatorPredictor motionStateEstimator;
  protected Random rng;
  protected PathStateDistribution newPriorStateSample;
  protected PathStateDistribution newPosteriorStateSample;
  
  public VehicleStateDistribution<?> getVehicleState() {
    return vehicleState;
  }

  public void setVehicleState(VehicleStateDistribution<?> vehicleState) {
    this.vehicleState = vehicleState;
  }

  public MotionStateEstimatorPredictor getMotionStateEstimator() {
    return motionStateEstimator;
  }

  public void setMotionStateEstimator(
    MotionStateEstimatorPredictor motionStateEstimator) {
    this.motionStateEstimator = motionStateEstimator;
  }

  public Random getRng() {
    return rng;
  }

  public void setRng(Random rng) {
    this.rng = rng;
  }

  public PathStateDistribution getNewPriorStateSample() {
    return newPriorStateSample;
  }

  public void setNewPriorStateSample(
    PathStateDistribution newPriorStateSample) {
    this.newPriorStateSample = newPriorStateSample;
  }

  public PathStateDistribution getNewPosteriorStateSample() {
    return newPosteriorStateSample;
  }

  public void setNewPosteriorStateSample(
    PathStateDistribution newPosteriorStateSample) {
    this.newPosteriorStateSample = newPosteriorStateSample;
  }

  /**
   * This estimator will learn/update a path state model covariance with an inverse Wishart
   * prior.<br>
   * 
   * @param vehicleState
   * @param motionStateEstimator 
   * @param rng
   */
  public RoadModelCovarianceEstimatorPredictor(VehicleStateDistribution<?> vehicleState, MotionStateEstimatorPredictor motionStateEstimator, Random rng) {
    Preconditions.checkState(vehicleState.getMotionStateEstimatorPredictor() != null);
    this.vehicleState = vehicleState;
    this.motionStateEstimator = motionStateEstimator;
    this.rng = rng;
  }
  
  private static final Logger log = LoggerFactory
      .getLogger(RoadModelCovarianceEstimatorPredictor.class);
  

  /**
   * Samples (x_{t+1} | x_t, y_{t+1}, ...) Needed for some parameter estimates.
   * 
   * @param priorPredictiveBelief
   * @param rng
   * @return
   */
  private PathStateDistribution sampleFilteredTransition(
    PathStateDistribution prior, Vector obs, Random rng) {

    final Path path = prior.getPathState().getPath();
    final MultivariateGaussian updatedStateSmplDist =
        this.vehicleState.getMotionStateEstimatorPredictor().createPredictiveDistribution(prior.getMotionDistribution());
    
    final Matrix stateTransCov = prior.getPathState().isOnRoad() ?
        this.motionStateEstimator.getRoadFilter().getModelCovariance()
        :this.motionStateEstimator.getGroundFilter().getModelCovariance();
    updatedStateSmplDist.setCovariance(stateTransCov);
    
    final PathStateDistribution predictState =
        new PathStateDistribution(path, updatedStateSmplDist);
    
    final MultivariateGaussian updatedPrediction = predictState.getMotionDistribution().clone();
    
    if (!path.isNullPath()) {
      AbstractKalmanFilter roadFilter = this.vehicleState.getMotionStateEstimatorPredictor().getRoadFilter().clone();
      
      final MultivariateGaussian obsProj =
          PathUtils.getRoadObservation(
              obs, this.vehicleState.getObservationCovarianceParam().getValue(), 
              path, predictState.getPathState().getEdge());//Iterables.getLast(path.getPathEdges()));

      roadFilter.setMeasurementCovariance(obsProj.getCovariance());
      roadFilter.measure(updatedPrediction, obsProj.getMean());
    } else {
      this.vehicleState.getMotionStateEstimatorPredictor().update(updatedPrediction, obs);
    }
    
    final PathStateDistribution postState = new PathStateDistribution(path, updatedPrediction);
    
    final TruncatedRoadGaussian sampler = new TruncatedRoadGaussian(postState.getMotionDistribution().clone());
    final Vector sampledMean = sampler.sample(rng);
    postState.setMean(sampledMean);

    return postState;
  }

  private PathStateDistribution sampleSmoothedPrevState(
    PathStateDistribution prior, PathStateDistribution posterior, Vector obs, Random rng) {

    final PathStateDistribution priorOnPath =
        posterior.getRelatableState(prior);

    final Matrix F;
    final Matrix G;
    final Matrix C;
    final Vector m;
    final Matrix Omega;
    final Vector y;
    final Matrix Sigma;

    if (posterior.getPathState().isOnRoad()) {
      /*
       * Force the observation onto the posterior edge,
       * since it's our best guess as to where it 
       * actually is.
       */
      final MultivariateGaussian obsProjBelief =
          PathUtils.getRoadObservation(
              obs, vehicleState.getObservationCovarianceParam().getValue(), 
              posterior.getPathState().getPath(),
              posterior.getPathState().getEdge());

      /*
       * Perform non-linear transform on y and obs cov. 
       */
      y = obsProjBelief.getMean();
      Sigma = obsProjBelief.getCovariance();
      
      F = MotionStateEstimatorPredictor.getOr();
      G = motionStateEstimator.getRoadModel().getA();
      C = priorOnPath.getMotionDistribution()
              .getCovariance();
      m = priorOnPath.getMotionDistribution().getMean();
      Omega = this.motionStateEstimator.getRoadFilter().getModelCovariance();
          //this.getOnRoadStateTransCovar();
    } else {
      y = obs;
      F = MotionStateEstimatorPredictor.getOg();
      G = motionStateEstimator.getGroundModel().getA();
      C = priorOnPath.getGroundDistribution().getCovariance();
      m = priorOnPath.getGroundDistribution().getMean();
      Omega = this.motionStateEstimator.getGroundFilter().getModelCovariance();
      Sigma = this.vehicleState.getObservationCovarianceParam().getValue();
    }

    final Matrix W =
        F.times(Omega).times(F.transpose()).plus(Sigma);
    final Matrix FG = F.times(G);
    final Matrix A =
        FG.times(C).times(FG.transpose()).plus(W);
    final Matrix Wtil =
        A.transpose().solve(FG.times(C.transpose()))
            .transpose();

    final Vector mSmooth =
        m.plus(Wtil.times(y.minus(FG.times(m))));
    final Matrix CSmooth =
        C.minus(Wtil.times(A).times(Wtil.transpose()));

    final MultivariateGaussian sampler = 
        new TruncatedRoadGaussian(mSmooth, CSmooth);

    final Vector result = sampler.sample(rng);
    
    sampler.setMean(result);
    
    if (posterior.getPathState().isOnRoad()) {
      final double clamped = 
              posterior.getPathState().getPath().clampToPath(sampler.getMean().getElement(0));
      sampler.getMean().setElement(0, clamped);
    }
    return new PathStateDistribution(posterior.getPathState().getPath(), sampler);
  }

  @Override
  public void update(InverseWishartDistribution covarPrior, Vector obs) {

    PathStateDistribution posteriorState = this.vehicleState.getPathStateParam().getParameterPrior();
    
    final PathStateDistribution priorState =
            posteriorState.getRelatableState(
                this.vehicleState.getParentState().getPathStateParam().getParameterPrior());
    
    final PathStateDistribution newPrevStateSample =
        sampleSmoothedPrevState(priorState, posteriorState, obs, rng);
    final PathStateDistribution newStateSample =
        sampleFilteredTransition(newPrevStateSample, obs, rng);
    
    this.newPriorStateSample = newPrevStateSample;
    this.newPosteriorStateSample = newStateSample;

    // TODO should keep these values, not recompute.
    final Matrix covFactor =
        motionStateEstimator.getCovarianceFactor(newPrevStateSample.getPathState().isOnRoad());

    final Matrix G =
        newPrevStateSample.getPathState().isOnRoad() ? motionStateEstimator.getRoadModel()
            .getA() : motionStateEstimator.getGroundModel().getA();
    final Vector sampleDiff =
        newStateSample.getMotionDistribution().getMean().minus(
            G.times(newPrevStateSample.getMotionDistribution().getMean()));
    final Matrix covFactorInv =
        StatisticsUtil.rootOfSemiDefinite(
            covFactor.times(covFactor.transpose())
                .pseudoInverse(1e-7), true, -1).transpose();
    final Vector stateError =
        covFactorInv.times(sampleDiff);
    final Matrix smplCov =
        stateError.outerProduct(stateError);

    // TODO debug.  remove.
    if (this.vehicleState.getObservation() instanceof TrueObservation) {
      final VehicleStateDistribution<?> trueState = ((TrueObservation)this.vehicleState.getObservation()).getTrueState();
      InverseWishartDistribution tmpPrior = covarPrior.clone();
      updateInvWishart(tmpPrior, smplCov);
      Matrix trueQ = newPrevStateSample.getPathState().isOnRoad()
          ? trueState.getOnRoadModelCovarianceParam().getValue() : 
             trueState.getOffRoadModelCovarianceParam().getValue();
      final Matrix updateError = tmpPrior.getMean().minus(trueQ);
      if (updateError.normFrobenius()
            > 0.4 * trueQ.normFrobenius()) {
        log.warn("Large update error: " + updateError);
//        log.warn("True state:" + trueState);
//        log.warn("Posterior:" + posteriorState);
      }
    }
    
    updateInvWishart(covarPrior, smplCov);
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
    // TODO Auto-generated method stub
    return null;
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
