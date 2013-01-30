package org.opentrackingtools.statistics.filters.vehicles.road.impl;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.mtj.DenseVector;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.distribution.InverseWishartDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Random;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.states.PathState;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.graph.paths.util.PathUtils;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.statistics.filters.impl.AdjKalmanFilter;
import org.opentrackingtools.statistics.impl.StatisticsUtil;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;

public class ErrorEstimatingRoadTrackingFilter
    extends
    AbstractRoadTrackingFilter<ErrorEstimatingRoadTrackingFilter> {

  public static class StateSample extends
      AbstractCloneableSerializable {

    private static final long serialVersionUID =
        -3237286995760051104L;

    private Vector state;
    private Vector stateLocation;
    private Boolean isBackward;

    public StateSample(Vector newStateSample,
      Boolean isBackward, Vector stateLocation) {
      this.state = newStateSample;
      this.isBackward = isBackward;
      this.stateLocation = stateLocation;
    }

    @Override
    public StateSample clone() {
      final StateSample clone = (StateSample) super.clone();
      clone.isBackward = this.isBackward;
      clone.state = this.state.clone();
      clone.stateLocation =
          this.stateLocation != null ? this.stateLocation
              .clone() : null;
      return clone;
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
      final StateSample other = (StateSample) obj;
      if (isBackward == null) {
        if (other.isBackward != null) {
          return false;
        }
      } else if (!isBackward.equals(other.isBackward)) {
        return false;
      }
      if (state == null) {
        if (other.state != null) {
          return false;
        }
      } else if (!StatisticsUtil.vectorEquals(state,
          other.state)) {
        return false;
      }
      if (stateLocation == null) {
        if (other.stateLocation != null) {
          return false;
        }
      } else if (!StatisticsUtil.vectorEquals(
          stateLocation, other.stateLocation)) {
        return false;
      }
      return true;
    }

    public Boolean getIsBackward() {
      return isBackward;
    }

    public Vector getState() {
      return this.state;
    }

    public Vector getStateLocation() {
      return this.stateLocation;
    }

    @Override
    public int hashCode() {
      final int prime = 31;
      int result = 1;
      result =
          prime
              * result
              + ((isBackward == null) ? 0 : isBackward
                  .hashCode());
      result =
          prime
              * result
              + ((state == null) ? 0 : StatisticsUtil
                  .hashCodeVector(state));
      result =
          prime
              * result
              + ((stateLocation == null) ? 0
                  : StatisticsUtil
                      .hashCodeVector(stateLocation));
      return result;
    }

    @Override
    public String toString() {
      return "StateSample [state=" + state
          + ", stateLocation=" + stateLocation
          + ", isBackward=" + isBackward + "]";
    }
  }

  private static final long serialVersionUID =
      2120712519463445779L;

  private InverseWishartDistribution obsVariancePrior;
  private InverseWishartDistribution onRoadStateVariancePrior;
  private InverseWishartDistribution offRoadStateVariancePrior;
  private PathState currentStateSample;
  private PathState prevStateSample;

  public ErrorEstimatingRoadTrackingFilter(
    InferenceGraph graph,
    Vector obsVarPrior, final int obsVarDof,
    Vector offRoadStateVarPrior, final int offRoadVarDof,
    Vector onRoadStateVarPrior, final int onRoadVarDof,
    int initialObsFreq, Random rng) {

    this.graph = graph;
    this.currentTimeDiff = initialObsFreq;
    /*
     * Initialize the priors with an expectation of the given "prior"
     * values.
     */
    final int obsInitialDof =
        obsVarDof - obsVarPrior.getDimensionality() - 1;
    this.obsVariancePrior =
        new InverseWishartDistribution(MatrixFactory
            .getDefault().createDiagonal(obsVarPrior)
            .scale(obsInitialDof), obsVarDof);
    final int offInitialDof =
        offRoadVarDof
            - offRoadStateVarPrior.getDimensionality() - 1;
    this.offRoadStateVariancePrior =
        new InverseWishartDistribution(MatrixFactory
            .getDefault()
            .createDiagonal(offRoadStateVarPrior)
            .scale(offInitialDof), offRoadVarDof);
    final int onInitialDof =
        onRoadVarDof
            - onRoadStateVarPrior.getDimensionality() - 1;
    this.onRoadStateVariancePrior =
        new InverseWishartDistribution(MatrixFactory
            .getDefault()
            .createDiagonal(onRoadStateVarPrior)
            .scale(onInitialDof), onRoadVarDof);

    if (rng != null) {
      this.obsCovar =
          StatisticsUtil.sampleInvWishart(obsVariancePrior,
              rng);
      this.setQg(StatisticsUtil.sampleInvWishart(
          offRoadStateVariancePrior, rng));
      this.setQr(StatisticsUtil.sampleInvWishart(
          onRoadStateVariancePrior, rng));
    } else {
      this.setObsCovar(obsVariancePrior.getMean());
      this.setQg(offRoadStateVariancePrior.getMean());
      this.setQr(onRoadStateVariancePrior.getMean());
    }

    /*
     * Create the road-coordinates filter
     */
    final LinearDynamicalSystem roadModel =
        new LinearDynamicalSystem(0, 2);
    final Matrix roadG =
        createStateTransitionMatrix(currentTimeDiff, true);
    roadModel.setA(roadG);
    roadModel.setB(MatrixFactory.getDefault()
        .createIdentity(2, 2));
    roadModel.setC(Or);
    this.roadModel = roadModel;

    final Matrix onRoadStateTransCovar =
        createStateCovarianceMatrix(this.currentTimeDiff,
            this.getQr(), true);
    this.roadFilter =
        new AdjKalmanFilter(roadModel,
            onRoadStateTransCovar, this.getObsCovar());
    this.setOnRoadStateTransCovar(onRoadStateTransCovar);

    /*
     * Create the ground-coordinates filter
     */
    final LinearDynamicalSystem groundModel =
        new LinearDynamicalSystem(0, 4);

    final Matrix groundGct =
        createStateTransitionMatrix(this.currentTimeDiff,
            false);

    groundModel.setA(groundGct);
    groundModel.setB(MatrixFactory.getDefault()
        .createIdentity(4, 4));
    groundModel.setC(Og);

    this.groundModel = groundModel;

    final Matrix offRoadStateTransCovar =
        createStateCovarianceMatrix(this.currentTimeDiff,
            this.getQg(), false);
    this.groundFilter =
        new AdjKalmanFilter(groundModel,
            offRoadStateTransCovar, this.getObsCovar());
    this.setOffRoadStateTransCovar(offRoadStateTransCovar);
  }

  @Override
  public ErrorEstimatingRoadTrackingFilter clone() {
    final ErrorEstimatingRoadTrackingFilter clone =
        (ErrorEstimatingRoadTrackingFilter) super.clone();
    clone
        .setCurrentStateSample(this.currentStateSample != null
            ? this.currentStateSample.clone() : null);
    clone
        .setPrevStateSample(this.prevStateSample != null
            ? this.prevStateSample.clone() : null);
    clone
        .setObsVariancePrior(this.obsVariancePrior.clone());
    clone
        .setOffRoadStateVariancePrior(this.offRoadStateVariancePrior
            .clone());
    clone
        .setOnRoadStateVariancePrior(this.onRoadStateVariancePrior
            .clone());
    return clone;
  }

  @Override
  public int compareTo(ErrorEstimatingRoadTrackingFilter o) {
    final CompareToBuilder comparator =
        new CompareToBuilder();
    
    comparator.append(((DenseVector) this.obsVariancePrior
        .convertToVector()).getArray(), ((DenseVector) o
        .getObsVariancePrior().convertToVector())
        .getArray());
    
    comparator.append(
        ((DenseVector) this.onRoadStateVariancePrior
            .convertToVector()).getArray(),
        ((DenseVector) o.getOnRoadStateVariancePrior()
            .convertToVector()).getArray());
    comparator.append(
        ((DenseVector) this.offRoadStateVariancePrior
            .convertToVector()).getArray(),
        ((DenseVector) o.getOffRoadStateVariancePrior()
            .convertToVector()).getArray());
    
    comparator.append(this.currentStateSample,
        o.getCurrentStateSample());
    comparator.append(this.prevStateSample,
        o.getPrevStateSample());
    
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
    if (!super.equals(obj)) {
      return false;
    }

    final ErrorEstimatingRoadTrackingFilter other =
        (ErrorEstimatingRoadTrackingFilter) obj;
    if (currentStateSample == null) {
      if (other.currentStateSample != null) {
        return false;
      }
    } else if (!currentStateSample
        .equals(other.currentStateSample)) {
      return false;
    }
    if (prevStateSample == null) {
      if (other.prevStateSample != null) {
        return false;
      }
    } else if (!prevStateSample
        .equals(other.prevStateSample)) {
      return false;
    }
    if (obsVariancePrior == null) {
      if (other.obsVariancePrior != null) {
        return false;
      }
    } else if (!StatisticsUtil.vectorEquals(
        obsVariancePrior.convertToVector(),
        other.obsVariancePrior.convertToVector())) {
      return false;
    }
    if (offRoadStateVariancePrior == null) {
      if (other.offRoadStateVariancePrior != null) {
        return false;
      }
    } else if (!StatisticsUtil.vectorEquals(
        offRoadStateVariancePrior.convertToVector(),
        other.offRoadStateVariancePrior.convertToVector())) {
      return false;
    }
    if (onRoadStateVariancePrior == null) {
      if (other.onRoadStateVariancePrior != null) {
        return false;
      }
    } else if (!StatisticsUtil.vectorEquals(
        onRoadStateVariancePrior.convertToVector(),
        other.onRoadStateVariancePrior.convertToVector())) {
      return false;
    }
    return true;
  }

  public PathState getCurrentStateSample() {
    return currentStateSample;
  }

  public InverseWishartDistribution getObsVariancePrior() {
    return obsVariancePrior;
  }

  public InverseWishartDistribution
      getOffRoadStateVariancePrior() {
    return offRoadStateVariancePrior;
  }

  public InverseWishartDistribution
      getOnRoadStateVariancePrior() {
    return onRoadStateVariancePrior;
  }

  public PathState getPrevStateSample() {
    return prevStateSample;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = super.hashCode();
    result =
        prime
            * result
            + ((currentStateSample == null) ? 0
                : currentStateSample.hashCode());
    result =
        prime
            * result
            + ((prevStateSample == null) ? 0
                : prevStateSample.hashCode());
    result =
        prime
            * result
            + ((obsVariancePrior == null) ? 0
                : StatisticsUtil
                    .hashCodeVector(obsVariancePrior
                        .convertToVector()));
    result =
        prime
            * result
            + ((offRoadStateVariancePrior == null)
                ? 0
                : StatisticsUtil
                    .hashCodeVector(offRoadStateVariancePrior
                        .convertToVector()));
    result =
        prime
            * result
            + ((onRoadStateVariancePrior == null)
                ? 0
                : StatisticsUtil
                    .hashCodeVector(onRoadStateVariancePrior
                        .convertToVector()));
    return result;
  }

  /**
   * Samples (x_{t+1} | x_t, y_{t+1}, ...) Needed for some parameter estimates.
   * 
   * @param priorPredictiveBelief
   * @param rng
   * @return
   */
  private PathState sampleFilteredTransition(
    PathState prevStateSample1, Vector obs, Random rng) {

    final InferredPath path = prevStateSample1.getPath();
    final PathStateBelief prior =
        path.getStateBeliefOnPath(
            new MultivariateGaussian(prevStateSample1
                .getGlobalState(), MatrixFactory
                .getDenseDefault().createIdentity(
                    prevStateSample1.getRawState()
                        .getDimensionality(),
                    prevStateSample1.getRawState()
                        .getDimensionality())));
//        SimplePathStateBelief.getPathStateBelief(
//            path,
//            new MultivariateGaussian(prevStateSample1
//                .getGlobalState(), MatrixFactory
//                .getDenseDefault().createIdentity(
//                    prevStateSample1.getRawState()
//                        .getDimensionality(),
//                    prevStateSample1.getRawState()
//                        .getDimensionality())),
//                        this.graph);
    final PathStateBelief prediction =
        this.predict(prior, path);
    final MultivariateGaussian updatedStateSmplDist =
        new MultivariateGaussian(
            prediction.getGlobalState(),
            prevStateSample1.isOnRoad() ? this
                .getOnRoadStateTransCovar() : this
                .getOffRoadStateTransCovar());
    final PathStateBelief predictState =
        prediction.getPath().getStateBeliefOnPath(updatedStateSmplDist);
//        PathStateBelief.getPathStateBelief(
//            prediction.getPath(), updatedStateSmplDist, this.graph);
    final PathStateBelief postState =
        this.measure(predictState, obs,
            predictState.getEdge());

    final Matrix Hsqrt =
        StatisticsUtil.rootOfSemiDefinite(postState
            .getCovariance());

    final Vector result =
        MultivariateGaussian.sample(
            postState.getGlobalState(), Hsqrt, rng);

    return path.getStateOnPath(result);
//    return PathState.getPathState(path, result, this.graph);
  }

  private PathState sampleSmoothedPrevState(
    PathStateBelief prior, PathStateBelief priorPred,
    PathStateBelief posterior, Vector obs, Random rng) {

    final PathStateBelief priorOnPath =
        priorPred.getPath().getStateBeliefOnPath(prior);
    Preconditions.checkState(Iterables.getFirst(
        priorOnPath.getPath().getPathEdges(), null).equals(
        Iterables.getFirst(priorPred.getPath().getPathEdges(),
            null))
        && Iterables.getFirst(
            priorPred.getPath().getPathEdges(), null).equals(
            Iterables.getFirst(posterior.getPath()
                .getPathEdges(), null)));

    final Matrix F;
    final Matrix G;
    final Matrix C;
    final Vector m;
    final Matrix Omega;
    final Vector y;
    final Matrix Sigma;

    if (posterior.isOnRoad()) {
      final MultivariateGaussian obsProjBelief =
          PathUtils.getRoadObservation(
              obs, this.obsCovar, priorPred.getPath(),
              Iterables.getLast(priorPred.getPath()
                  .getPathEdges()));

      /*
       * Perform non-linear transform on y and obs cov. 
       */
      y = obsProjBelief.getMean();
      Sigma = obsProjBelief.getCovariance();

      F = AbstractRoadTrackingFilter.getOr();
      G = this.getRoadModel().getA();
      C =
          priorOnPath.getGlobalStateBelief()
              .getCovariance();
      m = priorOnPath.getGlobalState();
      Omega = this.getOnRoadStateTransCovar();
    } else {
      y = obs;
      F = AbstractRoadTrackingFilter.getOg();
      G = this.getGroundModel().getA();
      C = priorOnPath.getGroundBelief().getCovariance();
      m = priorOnPath.getGroundState();
      Omega = this.getOffRoadStateTransCovar();
      Sigma = this.getObsCovar();
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

    final Matrix Csqrt =
        StatisticsUtil.rootOfSemiDefinite(CSmooth);

    final Vector result =
        MultivariateGaussian.sample(mSmooth, Csqrt, rng);

    return posterior.getPath().getStateOnPath(result);
//    return PathState.getPathState(posterior.getPath(),
//        result);
  }

  public void setCurrentStateSample(PathState pathState) {
    this.currentStateSample = pathState;
  }

  public void setObsVariancePrior(
    InverseWishartDistribution obsVariancePrior) {
    this.obsVariancePrior = obsVariancePrior;
  }

  public void setOffRoadStateVariancePrior(
    InverseWishartDistribution offRoadStateVariancePrior) {
    this.offRoadStateVariancePrior =
        offRoadStateVariancePrior;
  }

  public void setOnRoadStateVariancePrior(
    InverseWishartDistribution onRoadStateVariancePrior) {
    this.onRoadStateVariancePrior =
        onRoadStateVariancePrior;
  }

  public void setPrevStateSample(PathState pathState) {
    this.prevStateSample = pathState;
  }

  @Override
  public void update(VehicleState state, GpsObservation obs,
    PathStateBelief posteriorState,
    PathStateBelief priorPredictiveState, Random rng) {

    final PathStateBelief priorState =
            priorPredictiveState.getPath().getStateBeliefOnPath(
                state.getBelief());
    
    final PathState newPrevStateSample =
        sampleSmoothedPrevState(priorState,
            priorPredictiveState, posteriorState,
            obs.getProjectedPoint(), rng);
    final PathState newStateSample =
        sampleFilteredTransition(newPrevStateSample,
            obs.getProjectedPoint(), rng);

    /*
     * Update state covariance.
     */
    final InverseWishartDistribution covarPrior =
        newPrevStateSample.isOnRoad() ? this
            .getOnRoadStateVariancePrior() : this
            .getOffRoadStateVariancePrior();

    // TODO should keep these values, not recompute.
    final Matrix covFactor =
        this.getCovarianceFactor(newPrevStateSample.isOnRoad());

    final Matrix G =
        newPrevStateSample.isOnRoad() ? this.getRoadModel()
            .getA() : this.getGroundModel().getA();
    final Vector sampleDiff =
        newStateSample.getGlobalState().minus(
            G.times(newPrevStateSample.getGlobalState()));
    final Matrix covFactorInv =
        StatisticsUtil.rootOfSemiDefinite(
            covFactor.times(covFactor.transpose())
                .pseudoInverse(1e-7), true, -1).transpose();
    final Vector stateError =
        covFactorInv.times(sampleDiff);
    final Matrix smplCov =
        stateError.outerProduct(stateError);

    updateInvWishart(covarPrior, smplCov);

    final Matrix qSmpl =
        StatisticsUtil.sampleInvWishart(covarPrior, rng);

    final Matrix stateCovarSmpl =
        covFactor.times(qSmpl).times(covFactor.transpose());

    if (newStateSample.isOnRoad()) {
      this.setQr(qSmpl);
      this.setOnRoadStateTransCovar(stateCovarSmpl);
    } else {
      this.setQg(qSmpl);
      this.setOffRoadStateTransCovar(stateCovarSmpl);
    }

    /*
     * observation covar update
     */
    final InverseWishartDistribution obsCovPrior =
        this.getObsVariancePrior();
    final Vector obsError =
        obs.getProjectedPoint().minus(
            AbstractRoadTrackingFilter.getOg().times(
                newStateSample.getGroundState()));
    final Matrix obsSmplCov =
        obsError.outerProduct(obsError);
    updateInvWishart(obsCovPrior, obsSmplCov);

    final Matrix obsVarSmpl =
        StatisticsUtil.sampleInvWishart(obsCovPrior, rng);
    this.setObsCovar(obsVarSmpl);

    this.prevStateSample = newPrevStateSample;
    this.currentStateSample = newStateSample;
  }

  private void updateInvWishart(
    InverseWishartDistribution covarPrior, Matrix smplCov) {
    final int nOld = covarPrior.getDegreesOfFreedom();
    final int nNew = nOld + 1;
    covarPrior.setDegreesOfFreedom(nNew);
    covarPrior.setInverseScale(covarPrior.getInverseScale()
        .plus(smplCov));
  }

}
