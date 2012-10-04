package org.openplans.tools.tracking.impl.statistics.filters;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.mtj.decomposition.SingularValueDecompositionMTJ;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.distribution.InverseWishartDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.DefaultPair;
import gov.sandia.cognition.util.Pair;

import java.util.Random;

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.statistics.StatisticsUtil;

import com.google.common.collect.Iterables;

public class ErrorEstimatingRoadTrackingFilter extends
    AbstractRoadTrackingFilter {

  private static final long serialVersionUID = 2120712519463445779L;

  private InverseWishartDistribution obsVariancePrior;
  private InverseWishartDistribution onRoadStateVariancePrior;
  private InverseWishartDistribution offRoadStateVariancePrior;
  private Pair<Vector, InferredEdge> currentStateSample;

  public ErrorEstimatingRoadTrackingFilter(Vector obsVarPrior,
    final int obsVarDof, Vector offRoadStateVarPrior,
    final int offRoadVarDof, Vector onRoadStateVarPrior,
    final int onRoadVarDof, int initialObsFreq, Random rng) {

    this.currentTimeDiff = initialObsFreq;
    this.obsVariancePrior =
        new InverseWishartDistribution(MatrixFactory.getDefault()
            .createDiagonal(obsVarPrior), obsVarDof);
    this.offRoadStateVariancePrior =
        new InverseWishartDistribution(MatrixFactory.getDefault()
            .createDiagonal(offRoadStateVarPrior), offRoadVarDof);
    this.onRoadStateVariancePrior =
        new InverseWishartDistribution(MatrixFactory.getDefault()
            .createDiagonal(onRoadStateVarPrior), onRoadVarDof);

    if (rng != null) {
      this.obsVariance =
          StatisticsUtil.sampleInvWishart(obsVariancePrior, rng);
      this.Qg =
          StatisticsUtil.sampleInvWishart(offRoadStateVariancePrior,
              rng);
      this.Qr =
          StatisticsUtil.sampleInvWishart(onRoadStateVariancePrior,
              rng);
    } else {
      this.obsVariance = obsVariancePrior.getMean();
      this.Qg = offRoadStateVariancePrior.getMean();
      this.Qr = onRoadStateVariancePrior.getMean();
    }

    /*
     * Create the road-coordinates filter
     */
    final LinearDynamicalSystem roadModel =
        new LinearDynamicalSystem(0, 2);
    final Matrix roadG =
        createStateTransitionMatrix(currentTimeDiff, true);
    roadModel.setA(roadG);
    roadModel.setB(MatrixFactory.getDefault().createIdentity(2, 2));
    roadModel.setC(Or);
    this.roadModel = roadModel;

    this.onRoadStateVariance =
        createStateCovarianceMatrix(this.currentTimeDiff, Qr, true);
    this.roadFilter =
        new AdjKalmanFilter(roadModel, this.onRoadStateVariance,
            this.obsVariance);

    /*
     * Create the ground-coordinates filter
     */
    final LinearDynamicalSystem groundModel =
        new LinearDynamicalSystem(0, 4);

    final Matrix groundGct =
        createStateTransitionMatrix(this.currentTimeDiff, false);

    groundModel.setA(groundGct);
    groundModel.setB(MatrixFactory.getDefault().createIdentity(4, 4));
    groundModel.setC(Og);

    this.groundModel = groundModel;

    this.offRoadStateVariance =
        createStateCovarianceMatrix(this.currentTimeDiff, Qg, false);
    this.groundFilter =
        new AdjKalmanFilter(groundModel, this.offRoadStateVariance,
            this.obsVariance);
  }

  @Override
  public ErrorEstimatingRoadTrackingFilter clone() {
    final ErrorEstimatingRoadTrackingFilter clone =
        (ErrorEstimatingRoadTrackingFilter) super.clone();
    // XXX this isn't a clone!
    clone.setCurrentStateSample(this.currentStateSample);
    clone.setObsVariancePrior(this.obsVariancePrior.clone());
    clone.setOffRoadStateVariancePrior(this.offRoadStateVariancePrior
        .clone());
    clone.setOnRoadStateVariancePrior(this.onRoadStateVariancePrior
        .clone());
    return clone;
  }

  public Pair<Vector, InferredEdge> getCurrentStateSample() {
    return currentStateSample;
  }

  public InverseWishartDistribution getObsVariancePrior() {
    return obsVariancePrior;
  }

  public InverseWishartDistribution getOffRoadStateVariancePrior() {
    return offRoadStateVariancePrior;
  }

  public InverseWishartDistribution getOnRoadStateVariancePrior() {
    return onRoadStateVariancePrior;
  }

  public void setCurrentStateSample(
    Pair<Vector, InferredEdge> currentStateSample) {
    this.currentStateSample = currentStateSample;
  }

  public void setObsVariancePrior(
    InverseWishartDistribution obsVariancePrior) {
    this.obsVariancePrior = obsVariancePrior;
  }

  public void setOffRoadStateVariancePrior(
    InverseWishartDistribution offRoadStateVariancePrior) {
    this.offRoadStateVariancePrior = offRoadStateVariancePrior;
  }

  public void setOnRoadStateVariancePrior(
    InverseWishartDistribution onRoadStateVariancePrior) {
    this.onRoadStateVariancePrior = onRoadStateVariancePrior;
  }

  @Override
  public void updateSufficientStatistics(Observation obs,
    VehicleState state, MultivariateGaussian sampledBelief,
    InferredPath sampledPath, Random rng) {

    /*
     * observation covar 
     */
    final Vector stateMean = state.getMeanLocation();
    final Vector obsError =
        obs.getProjectedPoint().minus(stateMean);

    final InverseWishartDistribution obsCovPrior =
        this.getObsVariancePrior();
    updateInvWishart(obsCovPrior, obsError);

    Matrix obsVarSmpl = this.getObsVariancePrior().sample(rng);
    this.setObsVariance(obsVarSmpl);

    /*
     * state covar
     */
    final Vector stateError;
    if (sampledBelief.getInputDimensionality() == 2) {
      final InverseWishartDistribution covarPrior =
          this.getOnRoadStateVariancePrior();

      /*
       * Since we can't project onto an edge when we were considered off-road,
       * we have to come up with a way to handle off -> on road state covariance
       * updates.
       * 
       * For now, we skip it.
       */

      final InferredPath newSamplePath = sampledPath;
      final Vector newStateSample = sampledBelief.sample(rng);
      if (!newSamplePath.isOnPath(newStateSample.getElement(0))) {
        newStateSample.setElement(0,
            newSamplePath.clampToPath(newStateSample.getElement(0)));
      }

      final Vector oldStateSample =
          this.getCurrentStateSample().getFirst().clone();
      final InferredEdge oldSampleEdge =
          this.getCurrentStateSample().getSecond();

      if (!oldSampleEdge.isEmptyEdge()) {

        /*
         * Recall that the sign is relative, with regard to the path,
         * so we might need to change the sign for a proper difference.
         */
        final PathEdge newSampleEdge =
            Iterables.getFirst(newSamplePath.getEdges(), null);
        if (!oldSampleEdge.getEdge().getGeometry()
            .equalsExact(newSampleEdge.getEdge().getGeometry())
            && oldSampleEdge.getEdge().getGeometry()
                .equalsTopo(newSampleEdge.getEdge().getGeometry())) {
          /*
           * If we're here, then our edges are facing opposite directions, but
           * are otherwise the same.
           * Now we convert the old state sample to the new state's orientation.
           */

          oldStateSample.setElement(0, -oldStateSample.getElement(0));
        }

        // TODO should keep these values, not recompute.
        final Matrix covFactor = this.getCovarianceFactor(true);
        final Matrix covFactorInv = covFactor.pseudoInverse();

        stateError =
            covFactorInv.times(oldStateSample.minus(newStateSample));

        updateInvWishart(covarPrior, stateError);

        Matrix qrSmpl = StatisticsUtil.sampleInvWishart(covarPrior, rng);
        this.setQr(qrSmpl);
        // TODO necessary, given that we'll update this value for each time change?
        this.setOnRoadStateVariance(covFactor.times(this.getQr())
            .times(covFactor.transpose()));
      }

      final PathEdge newSampleEdge =
          newSamplePath.getEdgeForDistance(
              newStateSample.getElement(0), true);
      /*
       *  Make the position relative to the edge, not the path.
       */
      newStateSample.setElement(0, newStateSample.getElement(0)
          - newSampleEdge.getDistToStartOfEdge());

      this.setCurrentStateSample(new DefaultPair<Vector, InferredEdge>(
          newStateSample, newSampleEdge.getInferredEdge()));

    } else {
      final InverseWishartDistribution covarPrior =
          this.getOffRoadStateVariancePrior();

      final Vector newStateSample = sampledBelief.sample(rng);

      final InferredEdge oldEdge =
          this.getCurrentStateSample().getSecond();
      /*
       * TODO We're not dealing with transitions on -> off right now.
       */
      if (oldEdge.isEmptyEdge()) {
        final Vector oldStateSample =
            this.getCurrentStateSample().getFirst().clone();

        // TODO should keep these values, not recompute.
        final Matrix covFactor = this.getCovarianceFactor(false);
        final Matrix covFactorInv = covFactor.pseudoInverse();

        stateError =
            covFactorInv.times(oldStateSample.minus(newStateSample));

        updateInvWishart(covarPrior, stateError);

        Matrix qgSmpl = StatisticsUtil.sampleInvWishart(covarPrior, rng);
        this.setQg(qgSmpl);
        // TODO necessary, given that we'll update this value for each time change?
        this.setOffRoadStateVariance(covFactor.times(this.getQg())
            .times(covFactor.transpose()));
      }
      this.setCurrentStateSample(new DefaultPair<Vector, InferredEdge>(
          newStateSample, InferredEdge.getEmptyEdge()));
    }
  }

  private void updateInvWishart(
    InverseWishartDistribution covarPrior, Vector stateError) {
    final int nOld = covarPrior.getDegreesOfFreedom();
    final int nNew = nOld + 1;
    covarPrior.setDegreesOfFreedom(nNew);
    final Matrix smplCov = stateError.outerProduct(stateError);
    covarPrior.setInverseScale(covarPrior.getInverseScale().scale(((double)nOld)).plus(smplCov).scale(1d/(double)nNew));
  }

}
