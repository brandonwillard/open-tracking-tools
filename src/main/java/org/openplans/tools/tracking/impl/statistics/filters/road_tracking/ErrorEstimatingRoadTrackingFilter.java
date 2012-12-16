package org.openplans.tools.tracking.impl.statistics.filters.road_tracking;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.mtj.DenseVector;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.distribution.InverseWishartDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.graph.paths.PathState;
import org.openplans.tools.tracking.impl.graph.paths.PathStateBelief;
import org.openplans.tools.tracking.impl.statistics.StatisticsUtil;
import org.openplans.tools.tracking.impl.statistics.filters.AdjKalmanFilter;
import org.opentripplanner.routing.edgetype.PlainStreetEdge;
import org.opentripplanner.routing.edgetype.StreetTraversalPermission;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.vertextype.StreetVertex;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineString;

public class ErrorEstimatingRoadTrackingFilter extends
    AbstractRoadTrackingFilter<ErrorEstimatingRoadTrackingFilter> {

  public static class StateSample extends
      AbstractCloneableSerializable {

    private static final long serialVersionUID =
        -3237286995760051104L;

    private Vector state;
    private Vector stateLocation;
    private Boolean isBackward;

    public StateSample(Vector newStateSample, Boolean isBackward,
      Vector stateLocation) {
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
          this.stateLocation != null ? this.stateLocation.clone()
              : null;
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
      } else if (!StatisticsUtil.vectorEquals(state, other.state)) {
        return false;
      }
      if (stateLocation == null) {
        if (other.stateLocation != null) {
          return false;
        }
      } else if (!StatisticsUtil.vectorEquals(stateLocation,
          other.stateLocation)) {
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
          prime * result
              + ((isBackward == null) ? 0 : isBackward.hashCode());
      result =
          prime
              * result
              + ((state == null) ? 0 : StatisticsUtil
                  .hashCodeVector(state));
      result =
          prime
              * result
              + ((stateLocation == null) ? 0 : StatisticsUtil
                  .hashCodeVector(stateLocation));
      return result;
    }

    @Override
    public String toString() {
      return "StateSample [state=" + state + ", stateLocation="
          + stateLocation + ", isBackward=" + isBackward + "]";
    }
  }

  private static final long serialVersionUID = 2120712519463445779L;

  private InverseWishartDistribution obsVariancePrior;
  private InverseWishartDistribution onRoadStateVariancePrior;
  private InverseWishartDistribution offRoadStateVariancePrior;
  private PathState currentStateSample;

  public ErrorEstimatingRoadTrackingFilter(Vector obsVarPrior,
    final int obsVarDof, Vector offRoadStateVarPrior,
    final int offRoadVarDof, Vector onRoadStateVarPrior,
    final int onRoadVarDof, int initialObsFreq, Random rng) {

    this.currentTimeDiff = initialObsFreq;
    /*
     * Initialize the priors with an expectation of the given "prior"
     * values.
     */
    final int obsInitialDof =
        obsVarDof - obsVarPrior.getDimensionality() - 1;
    this.obsVariancePrior =
        new InverseWishartDistribution(MatrixFactory.getDefault()
            .createDiagonal(obsVarPrior).scale(obsInitialDof),
            obsVarDof);
    final int offInitialDof =
        offRoadVarDof - offRoadStateVarPrior.getDimensionality() - 1;
    this.offRoadStateVariancePrior =
        new InverseWishartDistribution(MatrixFactory.getDefault()
            .createDiagonal(offRoadStateVarPrior)
            .scale(offInitialDof), offRoadVarDof);
    final int onInitialDof =
        onRoadVarDof - onRoadStateVarPrior.getDimensionality() - 1;
    this.onRoadStateVariancePrior =
        new InverseWishartDistribution(MatrixFactory.getDefault()
            .createDiagonal(onRoadStateVarPrior).scale(onInitialDof),
            onRoadVarDof);

    if (rng != null) {
      this.obsCovar = StatisticsUtil.sampleInvWishart(
          obsVariancePrior, rng);
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
    roadModel.setB(MatrixFactory.getDefault().createIdentity(2, 2));
    roadModel.setC(Or);
    this.roadModel = roadModel;

    final Matrix onRoadStateTransCovar =
        createStateCovarianceMatrix(this.currentTimeDiff,
            this.getQr(), true);
    this.roadFilter =
        new AdjKalmanFilter(roadModel, onRoadStateTransCovar,
            this.getObsCovar());
    this.setOnRoadStateTransCovar(onRoadStateTransCovar);

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

    final Matrix offRoadStateTransCovar =
        createStateCovarianceMatrix(this.currentTimeDiff,
            this.getQg(), false);
    this.groundFilter =
        new AdjKalmanFilter(groundModel, offRoadStateTransCovar,
            this.getObsCovar());
    this.setOffRoadStateTransCovar(offRoadStateTransCovar);
  }

  @Override
  public ErrorEstimatingRoadTrackingFilter clone() {
    final ErrorEstimatingRoadTrackingFilter clone =
        (ErrorEstimatingRoadTrackingFilter) super.clone();
    clone.setCurrentStateSample(this.currentStateSample != null
        ? this.currentStateSample.clone() : null);
    clone.setObsVariancePrior(this.obsVariancePrior.clone());
    clone.setOffRoadStateVariancePrior(this.offRoadStateVariancePrior
        .clone());
    clone.setOnRoadStateVariancePrior(this.onRoadStateVariancePrior
        .clone());
    return clone;
  }

  @Override
  public int compareTo(ErrorEstimatingRoadTrackingFilter o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(((DenseVector) this.obsVariancePrior
        .convertToVector()).getArray(), ((DenseVector) o
        .getObsVariancePrior().convertToVector()).getArray());
    comparator.append(((DenseVector) this.onRoadStateVariancePrior
        .convertToVector()).getArray(), ((DenseVector) o
        .getOnRoadStateVariancePrior().convertToVector()).getArray());
    comparator
        .append(((DenseVector) this.offRoadStateVariancePrior
            .convertToVector()).getArray(), ((DenseVector) o
            .getOffRoadStateVariancePrior().convertToVector())
            .getArray());
    comparator.append(this.currentStateSample, o.getCurrentStateSample());
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
    } else if (!currentStateSample.equals(other.currentStateSample)) {
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

  public InverseWishartDistribution getOffRoadStateVariancePrior() {
    return offRoadStateVariancePrior;
  }

  public InverseWishartDistribution getOnRoadStateVariancePrior() {
    return onRoadStateVariancePrior;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = super.hashCode();
    result =
        prime
            * result
            + ((currentStateSample == null) ? 0 : currentStateSample
                .hashCode());
    result =
        prime
            * result
            + ((obsVariancePrior == null) ? 0 : StatisticsUtil
                .hashCodeVector(obsVariancePrior.convertToVector()));
    result =
        prime
            * result
            + ((offRoadStateVariancePrior == null) ? 0
                : StatisticsUtil
                    .hashCodeVector(offRoadStateVariancePrior
                        .convertToVector()));
    result =
        prime
            * result
            + ((onRoadStateVariancePrior == null) ? 0
                : StatisticsUtil
                    .hashCodeVector(onRoadStateVariancePrior
                        .convertToVector()));
    return result;
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
    this.offRoadStateVariancePrior = offRoadStateVariancePrior;
  }

  public void setOnRoadStateVariancePrior(
    InverseWishartDistribution onRoadStateVariancePrior) {
    this.onRoadStateVariancePrior = onRoadStateVariancePrior;
  }

  @Override
  public void update(VehicleState state, Observation obs, 
    PathStateBelief posteriorState,
    PathStateBelief priorPredictiveState, Random rng) {

    final Vector priorPredictiveBelief =
        priorPredictiveState.getRawState();

    final Matrix sampledBeliefRoot =
        StatisticsUtil.rootOfSemiDefinite(priorPredictiveBelief
            .getDimensionality() == 2 ? this
            .getOnRoadStateTransCovar() : this
            .getOffRoadStateTransCovar());
    final Vector newStateSampleVec =
        MultivariateGaussian.sample(priorPredictiveBelief,
            sampledBeliefRoot, rng);
    final InferredPath newSamplePath =
          priorPredictiveState.getPath();
    
    final PathState newStateSample = PathState.getPathState(newSamplePath,
        newStateSampleVec);

    /*
     * Update state covariances.
     */
    final Vector stateError;
    if (!priorPredictiveState.getPath().isEmptyPath()) {
      /*
       * Handle the on-road case
       */

      final InverseWishartDistribution covarPrior =
          this.getOnRoadStateVariancePrior();

      if (this.currentStateSample == null) {
        this.currentStateSample = newStateSample;
        return;
      }

      final PathState oldStateSample = this.getCurrentStateSample();

      if (this.getCurrentStateSample().isOnRoad()) {

        // TODO should keep these values, not recompute.
        final Matrix covFactor = this.getCovarianceFactor(true);
        final Matrix covFactorInv = covFactor.pseudoInverse();

        stateError =
            covFactorInv.times(
                newStateSample.minus(oldStateSample));

        updateInvWishart(covarPrior, stateError);

        final Matrix qrSmpl =
            StatisticsUtil.sampleInvWishart(covarPrior, rng);
        this.setQr(qrSmpl);
        // TODO necessary, given that we'll update this value for each time change?
        this.setOnRoadStateTransCovar(covFactor.times(this.getQr())
            .times(covFactor.transpose()));
      }

    } else {

      /*
       * Handle the off-road case.
       */
      final InverseWishartDistribution covarPrior =
          this.getOffRoadStateVariancePrior();

      if (this.currentStateSample == null) {
        this.currentStateSample = newStateSample;
        return;
      }

      /*
       * TODO We're not dealing with transitions on -> off right now.
       */
      final PathState oldStateSample =
          this.getCurrentStateSample();
      if (!oldStateSample.isOnRoad()) {

        // TODO should keep these values, not recompute.
        final Matrix covFactor = this.getCovarianceFactor(false);
        final Matrix covFactorInv = covFactor.pseudoInverse();

        stateError =
            covFactorInv.times(
                newStateSample.minus(oldStateSample));

        updateInvWishart(covarPrior, stateError);

        final Matrix qgSmpl =
            StatisticsUtil.sampleInvWishart(covarPrior, rng);
        this.setQg(qgSmpl);
        // TODO necessary, given that we'll update this value for each time change?
        this.setOffRoadStateTransCovar(covFactor.times(this.getQg())
            .times(covFactor.transpose()));
      }
    }

    /*
     * observation covar update
     */
    final Vector obsError =
        obs.getProjectedPoint().minus(
          AbstractRoadTrackingFilter.getOg().times(
            newStateSample.getGroundState()));

    final InverseWishartDistribution obsCovPrior =
        this.getObsVariancePrior();
    updateInvWishart(obsCovPrior, obsError);

    final Matrix obsVarSmpl =
        StatisticsUtil.sampleInvWishart(obsCovPrior, rng);
    this.setObsCovar(obsVarSmpl);

    this.currentStateSample = newStateSample;
  }


  private void updateInvWishart(
    InverseWishartDistribution covarPrior, Vector stateError) {
    final int nOld = covarPrior.getDegreesOfFreedom();
    final int nNew = nOld + 1;
    covarPrior.setDegreesOfFreedom(nNew);
    final Matrix smplCov = stateError.outerProduct(stateError);
    covarPrior.setInverseScale(covarPrior.getInverseScale().plus(
        smplCov));
  }

}
