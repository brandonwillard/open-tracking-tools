package org.opentrackingtools.estimators;

import gov.sandia.cognition.learning.algorithm.IncrementalLearner;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.decomposition.AbstractSingularValueDecomposition;
import gov.sandia.cognition.math.matrix.mtj.decomposition.SingularValueDecompositionMTJ;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.distribution.InverseGammaDistribution;
import gov.sandia.cognition.statistics.distribution.InverseWishartDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Collection;
import java.util.Random;

import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.distributions.TruncatedRoadGaussian;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.StatisticsUtil;
import org.opentrackingtools.util.TrueObservation;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.base.Preconditions;
import com.statslibextensions.math.matrix.SvdMatrix;
import com.statslibextensions.math.matrix.decomposition.SimpleSingularValueDecomposition;
import com.statslibextensions.statistics.distribution.SvdMultivariateGaussian;
import com.statslibextensions.statistics.distribution.ScaledInverseGammaCovDistribution;

public class RoadModelCovarianceEstimatorPredictor extends
    AbstractCloneableSerializable
    implements
    BayesianEstimatorPredictor<Matrix, Matrix, ScaledInverseGammaCovDistribution>,
    IncrementalLearner<Vector, ScaledInverseGammaCovDistribution> {

  private static final Logger log = LoggerFactory
      .getLogger(RoadModelCovarianceEstimatorPredictor.class);
  /**
   * 
   */
  private static final long serialVersionUID = 1077332231696992603L;
  protected MotionStateEstimatorPredictor motionStateEstimator;
  protected PathStateDistribution newPosteriorStateSample;
  protected PathStateDistribution newPriorStateSample;
  protected Random rng;

  protected VehicleStateDistribution<?> vehicleState;

  /**
   * This estimator will learn/update a path state model covariance with an
   * inverse Wishart prior.<br>
   * 
   * @param vehicleState
   * @param motionStateEstimator
   * @param rng
   */
  public RoadModelCovarianceEstimatorPredictor(
    VehicleStateDistribution<?> vehicleState,
    MotionStateEstimatorPredictor motionStateEstimator, Random rng) {
    Preconditions.checkState(vehicleState
        .getMotionStateEstimatorPredictor() != null);
    this.vehicleState = vehicleState;
    this.motionStateEstimator = motionStateEstimator;
    this.rng = rng;
  }

  @Override
  public ScaledInverseGammaCovDistribution createInitialLearnedObject() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public ComputableDistribution<Matrix> createPredictiveDistribution(
    ScaledInverseGammaCovDistribution posterior) {
    return posterior;
  }

  public MotionStateEstimatorPredictor getMotionStateEstimator() {
    return this.motionStateEstimator;
  }

  public PathStateDistribution getNewPosteriorStateSample() {
    return this.newPosteriorStateSample;
  }

  public PathStateDistribution getNewPriorStateSample() {
    return this.newPriorStateSample;
  }

  public Random getRng() {
    return this.rng;
  }

  public VehicleStateDistribution<?> getVehicleState() {
    return this.vehicleState;
  }

  @Override
  public ScaledInverseGammaCovDistribution learn(
    Collection<? extends Matrix> data) {
    // TODO Auto-generated method stub
    return null;
  }

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
        this.vehicleState.getMotionStateEstimatorPredictor()
            .createPredictiveDistribution(
                prior.getMotionDistribution());

    final Matrix stateTransCov =
        prior.getPathState().isOnRoad() ? this.motionStateEstimator
            .getRoadFilter().getModelCovariance()
            : this.motionStateEstimator.getGroundFilter()
                .getModelCovariance();
    updatedStateSmplDist.setCovariance(stateTransCov);

    final PathStateDistribution predictState =
        new PathStateDistribution(path, updatedStateSmplDist);

    final MultivariateGaussian updatedPrediction =
        predictState.getMotionDistribution().clone();

    if (!path.isNullPath()) {
      final TruncatedRoadKalmanFilter roadFilter =
          this.vehicleState.getMotionStateEstimatorPredictor()
              .getRoadFilter().clone();

      final SvdMultivariateGaussian obsProj =
          PathUtils.getRoadObservation(obs, this.vehicleState
              .getObservationCovarianceParam().getValue(), path,
              predictState.getPathState().getEdge());//Iterables.getLast(path.getPathEdges()));

      roadFilter.setMeasurementCovariance(obsProj.getCovariance());
      roadFilter.measure(updatedPrediction, obsProj.getMean());
    } else {
      this.vehicleState.getMotionStateEstimatorPredictor().update(
          updatedPrediction, obs);
    }

    final PathStateDistribution postState =
        new PathStateDistribution(path, updatedPrediction);

    final TruncatedRoadGaussian sampler =
        new TruncatedRoadGaussian(postState.getMotionDistribution()
            .clone());
    final Vector sampledMean = sampler.sample(rng);
    postState.setMean(sampledMean);

    return postState;
  }

  private PathStateDistribution sampleSmoothedPrevState(
    PathStateDistribution prior, PathStateDistribution posterior,
    Vector obs, Random rng) {

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
          PathUtils.getRoadObservation(obs, this.vehicleState
              .getObservationCovarianceParam().getValue(), posterior
              .getPathState().getPath(), posterior.getPathState()
              .getEdge());

      /*
       * Perform non-linear transform on y and obs cov. 
       */
      y = obsProjBelief.getMean();
      Sigma = obsProjBelief.getCovariance();

      F = MotionStateEstimatorPredictor.getOr();
      G = this.motionStateEstimator.getRoadModel().getA();
      C = priorOnPath.getMotionDistribution().getCovariance();
      m = priorOnPath.getMotionDistribution().getMean();
      Omega =
          this.motionStateEstimator.getRoadFilter()
              .getModelCovariance();
      //this.getOnRoadStateTransCovar();
    } else {
      y = obs;
      F = MotionStateEstimatorPredictor.getOg();
      G = this.motionStateEstimator.getGroundModel().getA();
      C = priorOnPath.getGroundDistribution().getCovariance();
      m = priorOnPath.getGroundDistribution().getMean();
      Omega =
          this.motionStateEstimator.getGroundFilter()
              .getModelCovariance();
      Sigma =
          this.vehicleState.getObservationCovarianceParam()
              .getValue();
    }

    final Matrix W = F.times(Omega).times(F.transpose()).plus(Sigma);
    final Matrix FG = F.times(G);
    final Matrix A = FG.times(C).times(FG.transpose()).plus(W);
    final Matrix Wtil =
        A.transpose().solve(FG.times(C.transpose())).transpose();

    final Vector mSmooth = m.plus(Wtil.times(y.minus(FG.times(m))));
    final Matrix CSmooth =
        C.minus(Wtil.times(A).times(Wtil.transpose()));

    // FIXME a temporary hack
    final AbstractSingularValueDecomposition Csvd =
        SingularValueDecompositionMTJ.create(CSmooth);

    final MultivariateGaussian sampler =
        new TruncatedRoadGaussian(mSmooth, new SvdMatrix(
            new SimpleSingularValueDecomposition(Csvd.getU(),
                Csvd.getS(), Csvd.getU().transpose())));

    final Vector result = sampler.sample(rng);

    sampler.setMean(result);

    if (posterior.getPathState().isOnRoad()) {
      final double clamped =
          posterior.getPathState().getPath()
              .clampToPath(sampler.getMean().getElement(0));
      sampler.getMean().setElement(0, clamped);
    }
    return new PathStateDistribution(posterior.getPathState()
        .getPath(), sampler);
  }

  public void setMotionStateEstimator(
    MotionStateEstimatorPredictor motionStateEstimator) {
    this.motionStateEstimator = motionStateEstimator;
  }

  public void setNewPosteriorStateSample(
    PathStateDistribution newPosteriorStateSample) {
    this.newPosteriorStateSample = newPosteriorStateSample;
  }

  public void setNewPriorStateSample(
    PathStateDistribution newPriorStateSample) {
    this.newPriorStateSample = newPriorStateSample;
  }

  public void setRng(Random rng) {
    this.rng = rng;
  }

  public void
      setVehicleState(VehicleStateDistribution<?> vehicleState) {
    this.vehicleState = vehicleState;
  }

  @Override
  public void update(ScaledInverseGammaCovDistribution target,
    Iterable<? extends Vector> data) {
    // TODO Auto-generated method stub

  }

  @Override
  public void
      update(ScaledInverseGammaCovDistribution covarPrior, Vector obs) {

    final PathStateDistribution posteriorState =
        this.vehicleState.getPathStateParam().getParameterPrior();

    final PathStateDistribution priorState =
        posteriorState
            .getRelatableState(this.vehicleState.getParentState()
                .getPathStateParam().getParameterPrior());

    final PathStateDistribution newPrevStateSample =
        this.sampleSmoothedPrevState(priorState, posteriorState, obs,
            this.rng);
    final PathStateDistribution newStateSample =
        this.sampleFilteredTransition(newPrevStateSample, obs,
            this.rng);

    this.newPriorStateSample = newPrevStateSample;
    this.newPosteriorStateSample = newStateSample;

    // TODO should keep these values, not recompute.
    final Matrix covFactor =
        this.motionStateEstimator
            .getCovarianceFactor(newPrevStateSample.getPathState()
                .isOnRoad());

    final Matrix G =
        newPrevStateSample.getPathState().isOnRoad()
            ? this.motionStateEstimator.getRoadModel().getA()
            : this.motionStateEstimator.getGroundModel().getA();
    final Vector sampleDiff =
        newStateSample
            .getMotionDistribution()
            .getMean()
            .minus(
                G.times(newPrevStateSample.getMotionDistribution()
                    .getMean()));
    final Matrix covFactorInv =
        StatisticsUtil.rootOfSemiDefinite(
            covFactor.times(covFactor.transpose())
                .pseudoInverse(1e-7), true, -1).transpose();
    final Vector stateError = covFactorInv.times(sampleDiff);

    // TODO debug.  remove.
    if (this.vehicleState.getObservation() instanceof TrueObservation) {
      final VehicleStateDistribution<?> trueState =
          ((TrueObservation) this.vehicleState.getObservation())
              .getTrueState();
      final ScaledInverseGammaCovDistribution tmpPrior = covarPrior.clone();
      this.updateInternal(tmpPrior, stateError);
      final Matrix trueQ =
          newPrevStateSample.getPathState().isOnRoad() ? trueState
              .getOnRoadModelCovarianceParam().getValue() : trueState
              .getOffRoadModelCovarianceParam().getValue();
      final Matrix updateError = tmpPrior.getMean().minus(trueQ);
      if (updateError.normFrobenius() > 0.4 * trueQ.normFrobenius()) {
        RoadModelCovarianceEstimatorPredictor.log
            .warn("Large update error: " + updateError);
        //        log.warn("True state:" + trueState);
        //        log.warn("Posterior:" + posteriorState);
      }
    }

    this.updateInternal(covarPrior, stateError);
  }

  private void updateInternal(
    ScaledInverseGammaCovDistribution covarPrior, Vector error) {
    final InverseGammaDistribution igDist = covarPrior.getInverseGammaDist();
    final double nOld = igDist.getShape();
    final double nNew = nOld + 0.5d;
    final double newScale = igDist.getScale() + error.dotProduct(error) * 0.5d;
    igDist.setShape(nNew);
    igDist.setScale(newScale);
  }

}
