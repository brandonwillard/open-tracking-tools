package org.opentrackingtools;

import gov.sandia.cognition.math.matrix.VectorFactory;

import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Random;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.opentrackingtools.distributions.CountedDataDistribution;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.distributions.PathStateMixtureDensityModel;
import org.opentrackingtools.graph.GenericJTSGraph;
import org.opentrackingtools.graph.InferenceGraphSegment;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.ProjectedCoordinate;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.model.VehicleStateDistribution.VehicleStateDistributionFactory;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.updater.VehicleStatePLUpdater;
import org.testng.AssertJUnit;
import org.testng.annotations.Test;

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.geom.LineString;

public class VehicleStatePLFilterTest {

  /**
   * Starting on the first edge, check that the best state is 
   * the one that's 4m forward on the last edge (the observation
   * is the last point on that edge).  The initial velocity
   * and time diff are such that the predicted state should
   * be exactly at the observation.
   */
  @Test
  public void update1() {
    final List<LineString> edges = Lists.newArrayList();
    edges.add(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] { new Coordinate(0, 0),
            new Coordinate(1, 0), }));
    edges.add(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] { new Coordinate(1, 0),
            new Coordinate(1, 1), }));
    edges.add(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] { new Coordinate(1, 0),
            new Coordinate(1, -1), }));
    edges.add(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] { new Coordinate(1, 1),
            new Coordinate(1, 2), }));
    edges.add(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] { new Coordinate(1, 2),
            new Coordinate(1, 3), }));
    final GenericJTSGraph graph = new GenericJTSGraph(edges, false);
    final InferenceGraphSegment startLine =
        Iterables.getOnlyElement(graph.getNearbyEdges(edges.get(0)
            .getCoordinate(), 0.5d));

    final Random rng = new Random(102343292l);

    final VehicleStateInitialParameters parameters =
        new VehicleStateInitialParameters(VectorFactory.getDefault()
            .copyArray(new double[] { 0d, 1d, 0d, 0d }),
            VectorFactory.getDefault().createVector2D(0.02d, 0.02d), Integer.MAX_VALUE,
            VectorFactory.getDefault().createVector1D(1e-4d), Integer.MAX_VALUE,
            VectorFactory.getDefault().createVector2D(1e-4d, 1e-4d), Integer.MAX_VALUE,
            VectorFactory.getDefault().createVector2D(1,
                Double.MAX_VALUE), VectorFactory.getDefault()
                .createVector2D(Double.MAX_VALUE, 1), 1, 4, 0);

    startLine.getParentEdge();
    final PathEdge startPathEdge = new PathEdge(startLine, 0d, false);
    
    final Coordinate initialObsCoord = new Coordinate(0, 0);
    final GpsObservation initialObs =
        new GpsObservation("test", new Date(0l), initialObsCoord, null, null, null, 0, null,
            new ProjectedCoordinate(null, initialObsCoord, null));

    VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph> factory = 
        new VehicleStateDistribution.VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph>();
    final VehicleStateDistribution<GpsObservation> currentState =
        factory.createInitialVehicleState(
            parameters, graph, 
            initialObs, rng, startPathEdge);

    final VehicleStatePLFilter<GpsObservation, GenericJTSGraph> filter =
        new VehicleStatePLFilter<GpsObservation, GenericJTSGraph>(initialObs, graph,
            factory, parameters, true, rng);

    final Coordinate newObsCoord = new Coordinate(1, 3);
    final GpsObservation newObs =
        new GpsObservation("test", new Date(4l), newObsCoord, null, null, null, 0, null,
            new ProjectedCoordinate(null, newObsCoord, null));
    final CountedDataDistribution<VehicleStateDistribution<GpsObservation>> initialParticles = 
        new CountedDataDistribution<VehicleStateDistribution<GpsObservation>>(
            Collections.singletonList(currentState), true);
    filter.update(initialParticles, newObs);
    
    VehicleStateDistribution<GpsObservation> bestVehicleState = initialParticles.getMaxValueKey();
    final PathStateDistribution bestPathStateDist = bestVehicleState.getPathStateParam().getParameterPrior();
    
    AssertJUnit.assertEquals(new LineSegment(new Coordinate(1, 2), new Coordinate(1, 3)), 
        bestPathStateDist.getPathState().getEdge().getLine());
    
    /*
     * Check that ground coords of the motion state are nearby where they're supposed to be.
     */
    AssertJUnit.assertArrayEquals(new double[] {1d, 3d}, 
        bestVehicleState.getMotionStateParam().getValue().toArray(), 1e-1);
    AssertJUnit.assertArrayEquals(new double[] {1d, 3d}, 
        bestVehicleState.getMotionStateParam().getConditionalDistribution().getMean().toArray(), 1e-1);
    
    /*
     * Now, check the on-road motion state.
     */
    AssertJUnit.assertArrayEquals(new double[] {4d, 1d}, 
        bestVehicleState.getPathStateParam().getParameterPrior().getMean().toArray(), 1e-1);
    AssertJUnit.assertArrayEquals(new double[] {4d, 1d}, 
        bestVehicleState.getPathStateParam().getValue().getMotionState().toArray(), 1e-1);
  }
  
  /**
   * Starting off-road, check that the best state is 
   * the one that's 4m forward on the last edge (the observation
   * is the last point on that edge).  The initial velocity
   * and time diff are such that the predicted state should
   * be exactly at the observation.
   */
  @Test
  public void update2() {
    final List<LineString> edges = Lists.newArrayList();
    edges.add(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] { new Coordinate(0, 0),
            new Coordinate(1, 0), }));
    edges.add(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] { new Coordinate(1, 0),
            new Coordinate(1, 1), }));
    edges.add(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] { new Coordinate(1, 0),
            new Coordinate(1, -1), }));
    edges.add(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] { new Coordinate(1, 1),
            new Coordinate(1, 2), }));
    edges.add(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] { new Coordinate(1, 2),
            new Coordinate(1, 3), }));
    final GenericJTSGraph graph = new GenericJTSGraph(edges, false);

    final Random rng = new Random(102343292l);

    final VehicleStateInitialParameters parameters =
        new VehicleStateInitialParameters(VectorFactory.getDefault()
            .copyArray(new double[] { 0d, 1d, 0d, 0d }),
            VectorFactory.getDefault().createVector2D(0.02d, 0.02d), Integer.MAX_VALUE,
            VectorFactory.getDefault().createVector1D(1e-4d), Integer.MAX_VALUE,
            VectorFactory.getDefault().createVector2D(1e-4d, 1e-4d), Integer.MAX_VALUE,
            VectorFactory.getDefault().createVector2D(1,
                Double.MAX_VALUE), VectorFactory.getDefault()
                .createVector2D(Double.MAX_VALUE, 1), 1, 4, 0);

    final Coordinate initialObsCoord = new Coordinate(0, 0);
    final GpsObservation initialObs =
        new GpsObservation("test", new Date(0l), initialObsCoord, null, null, null, 0, null,
            new ProjectedCoordinate(null, initialObsCoord, null));

    VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph> factory = 
        new VehicleStateDistribution.VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph>();
    final VehicleStateDistribution<GpsObservation> currentState =
        factory.createInitialVehicleState(
            parameters, graph, initialObs, rng, PathEdge.nullPathEdge);

    final VehicleStatePLFilter<GpsObservation, GenericJTSGraph> filter =
        new VehicleStatePLFilter<GpsObservation, GenericJTSGraph>(initialObs, graph,
            factory, parameters, true, rng);

    final Coordinate newObsCoord = new Coordinate(1, 3);
    final GpsObservation newObs =
        new GpsObservation("test", new Date(4l), newObsCoord, null, null, null, 0, null,
            new ProjectedCoordinate(null, newObsCoord, null));
    final CountedDataDistribution<VehicleStateDistribution<GpsObservation>> initialParticles = 
        new CountedDataDistribution<VehicleStateDistribution<GpsObservation>>(
            Collections.singletonList(currentState), true);
    filter.update(initialParticles, newObs);
    
    VehicleStateDistribution<GpsObservation> bestVehicleState = initialParticles.getMaxValueKey();
    final PathStateDistribution bestPathStateDist = bestVehicleState.getPathStateParam().getParameterPrior();
    
    AssertJUnit.assertEquals(new LineSegment(new Coordinate(1, 2), new Coordinate(1, 3)), 
        bestPathStateDist.getPathState().getEdge().getLine());
    
    /*
     * Check that ground coords of the motion state are nearby where they're supposed to be.
     */
    AssertJUnit.assertArrayEquals(new double[] {1d, 3d}, 
        bestVehicleState.getMotionStateParam().getValue().toArray(), 1e-1);
    AssertJUnit.assertArrayEquals(new double[] {1d, 3d}, 
        bestVehicleState.getMotionStateParam().getConditionalDistribution().getMean().toArray(), 1e-1);
    
    /*
     * Now, check the on-road motion state.
     */
    AssertJUnit.assertArrayEquals(new double[] {4d, 1d}, 
        bestVehicleState.getPathStateParam().getParameterPrior().getMean().toArray(), 1e-1);
    AssertJUnit.assertArrayEquals(new double[] {4d, 1d}, 
        bestVehicleState.getPathStateParam().getValue().getMotionState().toArray(), 1e-1);
  }
}
