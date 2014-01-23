package org.opentrackingtools.util;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorEntry;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian.SufficientStatistic;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Date;

import org.apache.log4j.BasicConfigurator;
import org.geotools.factory.FactoryRegistryException;
import org.geotools.referencing.operation.projection.ProjectionException;
import org.opengis.referencing.FactoryException;
import org.opengis.referencing.NoSuchAuthorityCodeException;
import org.opengis.referencing.operation.NoninvertibleTransformException;
import org.opengis.referencing.operation.TransformException;
import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.distributions.OnOffEdgeTransDistribution;
import org.opentrackingtools.distributions.TruncatedRoadGaussian;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.graph.GenericJTSGraph;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.Simulation.SimulationParameters;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.testng.Assert;
import org.testng.AssertJUnit;
import org.testng.annotations.AfterMethod;
import org.testng.annotations.BeforeMethod;
import org.testng.annotations.BeforeTest;
import org.testng.annotations.DataProvider;
import org.testng.annotations.Test;
import org.testng.internal.junit.ArrayAsserts;

import au.com.bytecode.opencsv.CSVWriter;

import com.google.common.collect.Ranges;
import com.statslibextensions.math.matrix.SvdMatrix;
import com.vividsolutions.jts.geom.Coordinate;

/**
 * Tests for simulated movement on a square grid graph for three
 * travel scenarios: road only, off-road only, and both.
 * Since simulations are the primary automatic means of 
 * testing for our filters, we need to be sure the simulations
 * are consistent.
 * 
 * Specifically, these tests check that the state and observation errors are
 * within bounds given by their covariances.
 * 
 * @author bwillar0
>>>>>>> origin/master
 *
 */
public class SimulationTest {

  private static final Logger _log = LoggerFactory
      .getLogger(SimulationTest.class);

  @DataProvider
  private static final Object[][] initialStateData() {
    return new Object[][] {
        {
            /*
             * Road only
             */
            new VehicleStateInitialParameters(null, VectorFactory
                .getDefault().createVector2D(60d, 60d), 20,
                VectorFactory.getDefault().createVector1D(6.25e-4),
                30, VectorFactory.getDefault().createVector2D(
                    6.25e-4, 6.25e-4), 20, VectorFactory.getDefault()
                    .createVector2D(1d, Double.MAX_VALUE),
                VectorFactory.getDefault().createVector2D(
                    Double.MAX_VALUE, 1d), 25, 30, 2159585l),
            Boolean.FALSE, 126000 }
        ,{
            /*
             * Ground only
             */
            new VehicleStateInitialParameters(null, VectorFactory
                .getDefault().createVector2D(60d, 60d), 20,
                VectorFactory.getDefault().createVector1D(6.25e-4),
                30, VectorFactory.getDefault().createVector2D(
                    6.25e-4, 6.25e-4), 20, VectorFactory.getDefault()
                    .createVector2D(Double.MAX_VALUE, 1d),
                VectorFactory.getDefault().createVector2D(1d,
                    Double.MAX_VALUE), 25, 10, 215955l),
            Boolean.FALSE, 126000 },
        {
            /*
             * Mixed 
             */
            new VehicleStateInitialParameters(null, VectorFactory
                .getDefault().createVector2D(60d, 60d), 20,
                VectorFactory.getDefault().createVector1D(6.25e-4),
                30, VectorFactory.getDefault().createVector2D(
                    6.25e-4, 6.25e-4), 20, VectorFactory.getDefault()
                    .createVector2D(1d, 1d), VectorFactory
                    .getDefault().createVector2D(1d, 1d), 25, 15,
                21595857l), Boolean.TRUE, 46000 } 
        };
  }

  private Matrix avgTransform;
  private InferenceGraph graph;
  private double[] movementZeroArray;
  private double[] obsErrorZeroArray;
  private Simulation sim;



  private Coordinate startCoord;

  private VehicleStateDistribution<GpsObservation> resetState(
    VehicleStateDistribution<GpsObservation> vehicleState) {
    return this.sim.computeInitialState();
  }

  @Test(dataProvider = "initialStateData")
  public void runSimulation(
    VehicleStateInitialParameters vehicleStateInitialParams,
    boolean generalizeMoveDiff, long duration)
      throws NoninvertibleTransformException, TransformException,
      SecurityException, IllegalArgumentException,
      ClassNotFoundException, NoSuchMethodException,
      InstantiationException, IllegalAccessException,
      InvocationTargetException {
    BasicConfigurator.configure();

    final SimulationParameters simParams =
        new SimulationParameters(this.startCoord, new Date(0l),
            duration, vehicleStateInitialParams.getInitialObsFreq(),
            false, true, vehicleStateInitialParams);

    this.sim =
        new Simulation("test-sim", this.graph, simParams,
            vehicleStateInitialParams);

    long time = this.sim.getSimParameters().getStartTime().getTime();

    VehicleStateDistribution<GpsObservation> vehicleState =
        this.sim.computeInitialState();

    final MultivariateGaussian.SufficientStatistic obsErrorSS =
        new MultivariateGaussian.SufficientStatistic();
    final MultivariateGaussian.SufficientStatistic movementSS =
        new MultivariateGaussian.SufficientStatistic();
    final MultivariateGaussian.SufficientStatistic transitionsSS =
        new MultivariateGaussian.SufficientStatistic();

    this.obsErrorZeroArray = null;
    this.movementZeroArray = null;

    final long approxRuns =
        this.sim.getSimParameters().getDuration()
            / Math.round(this.sim.getSimParameters().getFrequency());

    this.checkConditionsAndUpdateStats(vehicleState, obsErrorSS, movementSS,
        transitionsSS, generalizeMoveDiff);

    do {
      try {
        vehicleState = this.sim.stepSimulation(vehicleState);
        this.checkConditionsAndUpdateStats(vehicleState, obsErrorSS, movementSS,
            transitionsSS, generalizeMoveDiff);
        time = vehicleState.getObservation().getTimestamp().getTime();
      } catch (final ProjectionException ex) {
        vehicleState = this.resetState(vehicleState);
        //        if (vehicleState.getParentState() != null)
        //          vehicleState.setParentState(resetState(vehicleState.getParentState()));
        SimulationTest._log
            .warn("Outside of projection!  Flipped state velocities");
      }

    } while (time < this.sim.getSimParameters().getEndTime()
        .getTime());

    AssertJUnit
        .assertTrue(movementSS.getCount() > 0.95d * approxRuns);

    SimulationTest._log.info(obsErrorSS.getMean().toString());
    SimulationTest._log.info(movementSS.getMean().toString());
    SimulationTest._log.info(transitionsSS.getMean().toString());

  }

  @BeforeTest
  public void setUp() throws NoSuchAuthorityCodeException,
      FactoryRegistryException, FactoryException, IOException {

    this.startCoord = new Coordinate(40.7549, -73.97749);

    this.graph =
        new GenericJTSGraph(
            TestUtils.createGridGraph(this.startCoord), true);

    this.avgTransform =
        MatrixFactory
            .getDefault()
            .copyArray(
                new double[][] { { 1, 0, 1, 0 }, { 0, 1, 0, 1 } })
            .scale(1d / 2d);

    /*
     * Limit the maximum speed so that this doesn't take forever
     * with crazy long graph searches.
     */
    TruncatedRoadGaussian.velocityRange = Ranges.closed(0d, 15d);
  }

  private void checkConditionsAndUpdateStats(
    VehicleStateDistribution<GpsObservation> vehicleState,
    SufficientStatistic obsErrorSS, SufficientStatistic stateSS,
    SufficientStatistic transitionsSS, boolean generalizeMoveDiff) {

    final Vector obsError =
        vehicleState.getObservation().getProjectedPoint()
            .minus(vehicleState.getMeanLocation());
    obsErrorSS.update(obsError);

    SimulationTest._log.info("obsError=" + obsErrorSS.getMean());

    final MotionStateEstimatorPredictor motionStateEstimator =
        new MotionStateEstimatorPredictor(vehicleState, null,
            this.sim.getSimParameters().getFrequency());

    final VehicleStateDistribution<GpsObservation> parentState =
        vehicleState.getParentState();
    if (parentState != null) {
      final PathState sampledPathState =
          vehicleState.getPathStateParam().getValue();
      SimulationTest._log.info(sampledPathState.toString());
      final PathState parentPathState =
          parentState.getPathStateParam().getValue();
      

      final MultivariateGaussian priorState;
      final MultivariateGaussian predictedMotionStateDist;
      if (sampledPathState.getPath().isNullPath()
          && !parentPathState.getPath().isNullPath()) {
        priorState =
            new TruncatedRoadGaussian(
                parentPathState.getGroundState(), new SvdMatrix(
                    MatrixFactory.getDefault().createMatrix(4, 4)));
        final Vector expectedError =
            this.sim.getUpdater().getSampledTransitionError();
        predictedMotionStateDist =
            motionStateEstimator
                .createPredictiveDistribution(priorState);

        predictedMotionStateDist.setMean(predictedMotionStateDist
            .getMean().plus(expectedError));
      } else if (!sampledPathState.getPath().isNullPath()
          && parentPathState.getPath().isNullPath()) {
        priorState =
            new TruncatedRoadGaussian(parentPathState.getEdgeState(),
                new SvdMatrix(MatrixFactory.getDefault()
                    .createMatrix(4, 4)));
        predictedMotionStateDist =
            motionStateEstimator
                .createPredictiveDistribution(priorState);
        predictedMotionStateDist.getMean().plusEquals(
            this.sim.getUpdater().getSampledTransitionError());
        PathUtils.convertToRoadBelief(predictedMotionStateDist,
            sampledPathState.getPath(), sampledPathState.getEdge(),
            true, null, null);
        
      } else {

        if (!sampledPathState.getPath().isNullPath()
                  && !parentPathState.getPath().isNullPath()) {
          /*
           * Make sure that we're moving forward only.
           */
          Vector stateMotionDiff = sampledPathState.minus(parentPathState);

          Assert.assertTrue(stateMotionDiff.getElement(0) > 0d);
        }
      
        priorState =
            new TruncatedRoadGaussian(parentState.getPathStateParam()
                .getValue().getEdgeState(), new SvdMatrix(parentState
                .getMotionStateParam().getParameterPrior()
                .getCovariance()));
        final Vector expectedError =
            this.sim.getUpdater().getSampledTransitionError();
        predictedMotionStateDist =
            motionStateEstimator
                .createPredictiveDistribution(priorState);

        predictedMotionStateDist.setMean(predictedMotionStateDist
            .getMean().plus(expectedError));
      }
      final PathState predictedPathState =
          new PathState(sampledPathState.getPath(),
              predictedMotionStateDist.getMean());

      final Vector stateDiff =
          sampledPathState.minus(predictedPathState);
      ArrayAsserts.assertArrayEquals("state error", 
          VectorFactory.getDefault().createVector(
              stateDiff.getDimensionality()).toArray(),
          stateDiff.toArray(), 1e-5);

      if (generalizeMoveDiff && stateDiff.getDimensionality() == 4) {
        final Vector stateDiffAvg =
            this.avgTransform.times(stateDiff);
        stateSS.update(stateDiffAvg);
      } else {
        stateSS.update(stateDiff);
      }
      SimulationTest._log.info("movementError=" + stateDiff);
      SimulationTest._log
          .info("movementMean=" + stateSS.getMean());

      Vector transType =
          OnOffEdgeTransDistribution.getTransitionType(
              parentPathState.getEdge().getInferenceGraphSegment(),
              sampledPathState.getEdge().getInferenceGraphSegment());

      if (parentPathState.isOnRoad()) {
        transType =
            transType.stack(MotionStateEstimatorPredictor.zeros2D);
      } else {
        transType =
            MotionStateEstimatorPredictor.zeros2D.stack(transType);
      }

      transitionsSS.update(transType);
      SimulationTest._log.info("transitionsMean="
          + transitionsSS.getMean());
    }

    if (stateSS.getCount() > 0) {//Math.min(approxRuns/16, 25)) {
      if (this.obsErrorZeroArray == null) {
        this.obsErrorZeroArray =
            VectorFactory
                .getDefault()
                .createVector(
                    obsErrorSS.getMean().getDimensionality())
                .toArray();
      }

      ArrayAsserts.assertArrayEquals("obs. error avg.",
          this.obsErrorZeroArray,
          obsErrorSS.getMean().toArray(),
          2 * Math.sqrt(vehicleState.getObservationCovarianceParam()
              .getValue().normFrobenius()));

      if (this.movementZeroArray == null) {
        this.movementZeroArray =
            VectorFactory
                .getDefault()
                .createVector(
                    stateSS.getMean().getDimensionality())
                .toArray();
      }

      final Matrix stateCovariance;
      if (vehicleState.getPathStateParam().getValue().isOnRoad()) {
        final Matrix covFactor =
            motionStateEstimator.getCovarianceFactor(true);
        stateCovariance =
            covFactor.times(
                vehicleState.getOnRoadModelCovarianceParam()
                    .getValue()).times(covFactor.transpose());
      } else {
        final Matrix covFactor =
            motionStateEstimator.getCovarianceFactor(false);
        stateCovariance =
            covFactor.times(
                vehicleState.getOffRoadModelCovarianceParam()
                    .getValue()).times(covFactor.transpose());
      }

      ArrayAsserts.assertArrayEquals("state error avg.", this.movementZeroArray,
          stateSS.getMean().toArray(),
          10 * Math.sqrt(stateCovariance.normFrobenius()));

      if (vehicleState.getPathStateParam().getValue().isOnRoad()) {
        for (final VectorEntry entry : vehicleState
            .getPathStateParam().getValue()) {
          AssertJUnit.assertTrue(entry.getValue() >= 0);
        }
      }
    }
  }

}
