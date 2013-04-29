package org.opentrackingtools.updater;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;

import java.util.Date;
import java.util.List;
import java.util.Random;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.graph.GenericJTSGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.graph.InferenceGraphSegment;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.ProjectedCoordinate;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.model.VehicleStateDistribution.VehicleStateDistributionFactory;
import org.opentrackingtools.paths.PathEdge;
import org.testng.AssertJUnit;
import org.testng.annotations.Test;
import org.testng.internal.junit.ArrayAsserts;

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineString;

public class VehicleStateBootstrapUpdaterTest {

  /**
   * Test a prediction that leaves us on the same edge as the one we started on.
   */
  @Test
  public void testUpdate1() {
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

    final GpsObservation obs =
        new GpsObservation("test", new Date(0l), edges.get(0)
            .getCoordinate(), null, null, null, 0, null,
            new ProjectedCoordinate(null, edges.get(0)
                .getCoordinate(), null));

    final Random rng = new Random(102343292l);

    final VehicleStateInitialParameters parameters =
        new VehicleStateInitialParameters(VectorFactory.getDefault()
            .copyArray(new double[] { 0.4d, 1d / 40d, 0d, 0d }),
            VectorFactory.getDefault().createVector2D(100d, 100d),
            Integer.MAX_VALUE, VectorFactory.getDefault()
                .createVector1D(0d), Integer.MAX_VALUE,
            /*
             * No error, so the bootstrap sampling doesn't affect the sampled
             * edge results.
             * Same with the transition prior probabilities: make sure it stays
             * on-road.
             */
            VectorFactory.getDefault().createVector2D(0d, 0d),
            Integer.MAX_VALUE, VectorFactory.getDefault()
                .createVector2D(1, Double.MAX_VALUE), VectorFactory
                .getDefault().createVector2D(Double.MAX_VALUE, 1), 0,
            4, 0);

    startLine.getParentEdge();
    final PathEdge startPathEdge = new PathEdge(startLine, 0d, false);

    final VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph> factory =
        new VehicleStateDistribution.VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph>();
    final VehicleStateDistribution<GpsObservation> currentState =
        factory.createInitialVehicleState(parameters, graph, obs,
            rng, startPathEdge);

    final VehicleStateBootstrapUpdater<GpsObservation> updater =
        new VehicleStateBootstrapUpdater<GpsObservation>(obs, graph,
            parameters, rng);

    final VehicleStateDistribution<GpsObservation> newState =
        updater.update(currentState);

    final InferenceGraphSegment expectedSegment = startLine;

    AssertJUnit.assertEquals(expectedSegment.getLine(), newState
        .getPathStateParam().getValue().getEdge().getLine());
  }

  /**
   * Test a prediction that moves forward four edges, where forward edges exist.
   */
  @Test
  public void testUpdate2() {
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

    final GpsObservation obs =
        new GpsObservation("test", new Date(0l), edges.get(0)
            .getCoordinate(), null, null, null, 0, null,
            new ProjectedCoordinate(null, edges.get(0)
                .getCoordinate(), null));

    final Random rng = new Random(102343292l);

    final VehicleStateInitialParameters parameters =
        new VehicleStateInitialParameters(VectorFactory.getDefault()
            .copyArray(new double[] { 0d, 1d, 0d, 0d }),
            VectorFactory.getDefault().createVector2D(100d, 100d),
            Integer.MAX_VALUE, VectorFactory.getDefault()
                .createVector1D(0d), Integer.MAX_VALUE,
            /*
             * No error, so the bootstrap sampling doesn't affect the sampled
             * edge results.
             * Same with the transition prior probabilities: make sure it stays
             * on-road.
             */
            VectorFactory.getDefault().createVector2D(0d, 0d),
            Integer.MAX_VALUE, VectorFactory.getDefault()
                .createVector2D(1, Double.MAX_VALUE), VectorFactory
                .getDefault().createVector2D(Double.MAX_VALUE, 1), 0,
            4, 0);

    startLine.getParentEdge();
    final PathEdge startPathEdge = new PathEdge(startLine, 0d, false);

    final VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph> factory =
        new VehicleStateDistribution.VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph>();
    final VehicleStateDistribution<GpsObservation> currentState =
        factory.createInitialVehicleState(parameters, graph, obs,
            rng, startPathEdge);

    final VehicleStateBootstrapUpdater<GpsObservation> updater =
        new VehicleStateBootstrapUpdater<GpsObservation>(obs, graph,
            parameters, rng);

    final VehicleStateDistribution<GpsObservation> newState =
        updater.update(currentState);

    final InferenceGraphEdge expectedEdge =
        Iterables.getOnlyElement(
            graph.getNearbyEdges(new Coordinate(1d, 3d), 0.5d))
            .getParentEdge();

    AssertJUnit.assertEquals(expectedEdge, newState
        .getPathStateParam().getValue().getEdge()
        .getInferenceGraphEdge());
  }

  /**
   * Test a prediction that moves forward three edges, where no third forward
   * edge exists.
   */
  @Test
  public void testUpdate3() {
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
    final GenericJTSGraph graph = new GenericJTSGraph(edges, false);
    final InferenceGraphSegment startLine =
        Iterables.getOnlyElement(graph.getNearbyEdges(edges.get(0)
            .getCoordinate(), 0.5d));

    final GpsObservation obs =
        new GpsObservation("test", new Date(0l), edges.get(0)
            .getCoordinate(), null, null, null, 0, null,
            new ProjectedCoordinate(null, edges.get(0)
                .getCoordinate(), null));

    final Random rng = new Random(102343292l);

    final VehicleStateInitialParameters parameters =
        new VehicleStateInitialParameters(VectorFactory.getDefault()
            .copyArray(new double[] { 0d, 1d, 0d, 0d }),
            VectorFactory.getDefault().createVector2D(100d, 100d),
            Integer.MAX_VALUE, VectorFactory.getDefault()
                .createVector1D(0d), Integer.MAX_VALUE,
            /*
             * No error, so the bootstrap sampling doesn't affect the sampled
             * edge results.
             * Same with the transition prior probabilities: make sure it stays
             * on-road.
             */
            VectorFactory.getDefault().createVector2D(0d, 0d),
            Integer.MAX_VALUE, VectorFactory.getDefault()
                .createVector2D(1, Double.MAX_VALUE), VectorFactory
                .getDefault().createVector2D(Double.MAX_VALUE, 1), 0,
            4, 0);

    startLine.getParentEdge();
    final PathEdge startPathEdge = new PathEdge(startLine, 0d, false);

    final VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph> factory =
        new VehicleStateDistribution.VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph>();
    final VehicleStateDistribution<GpsObservation> currentState =
        factory.createInitialVehicleState(parameters, graph, obs,
            rng, startPathEdge);

    final VehicleStateBootstrapUpdater<GpsObservation> updater =
        new VehicleStateBootstrapUpdater<GpsObservation>(obs, graph,
            parameters, rng);

    final VehicleStateDistribution<GpsObservation> newState =
        updater.update(currentState);

    final InferenceGraphEdge expectedEdge =
        InferenceGraphEdge.nullGraphEdge;

    AssertJUnit.assertEquals(expectedEdge, newState
        .getPathStateParam().getValue().getEdge()
        .getInferenceGraphEdge());

    final Vector expectedMotionState =
        VectorFactory.getDefault().copyArray(
            new double[] { 4d, 1d, 0d, 0d });

    ArrayAsserts.assertArrayEquals(expectedMotionState.toArray(),
        newState.getPathStateParam().getValue().getMotionState()
            .toArray(), 1e-5);
  }

  /**
   * Test a prediction that moves from an edge to off-road.
   */
  @Test
  public void testUpdate4() {
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

    final GpsObservation obs =
        new GpsObservation("test", new Date(0l), edges.get(0)
            .getCoordinate(), null, null, null, 0, null,
            new ProjectedCoordinate(null, edges.get(0)
                .getCoordinate(), null));

    final Random rng = new Random(102343292l);

    final VehicleStateInitialParameters parameters =
        new VehicleStateInitialParameters(VectorFactory.getDefault()
            .copyArray(new double[] { 0d, 1d, 0d, 0d }),
            VectorFactory.getDefault().createVector2D(100d, 100d),
            Integer.MAX_VALUE, VectorFactory.getDefault()
                .createVector1D(0d), Integer.MAX_VALUE,
            /*
             * No error, so the bootstrap sampling doesn't affect the sampled
             * edge results.
             */
            VectorFactory.getDefault().createVector2D(0d, 0d),
            Integer.MAX_VALUE,
            /*
             * Changed these transition probabilities so that they'll guarantee
             * an off-road sampled result within the updater.
             */
            VectorFactory.getDefault().createVector2D(
                Double.MAX_VALUE, 1), VectorFactory.getDefault()
                .createVector2D(1, Double.MAX_VALUE), 0, 4, 0);

    final PathEdge startPathEdge = new PathEdge(startLine, 0d, false);

    final VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph> factory =
        new VehicleStateDistribution.VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph>();
    final VehicleStateDistribution<GpsObservation> currentState =
        factory.createInitialVehicleState(parameters, graph, obs,
            rng, startPathEdge);

    final VehicleStateBootstrapUpdater<GpsObservation> updater =
        new VehicleStateBootstrapUpdater<GpsObservation>(obs, graph,
            parameters, rng);

    final VehicleStateDistribution<GpsObservation> newState =
        updater.update(currentState);

    final InferenceGraphEdge expectedEdge =
        InferenceGraphEdge.nullGraphEdge;
    AssertJUnit.assertEquals(expectedEdge, newState
        .getPathStateParam().getValue().getEdge()
        .getInferenceGraphEdge());

    final Vector expectedMotionState =
        VectorFactory.getDefault().copyArray(
            new double[] { 4d, 1d, 0d, 0d });

    ArrayAsserts.assertArrayEquals(expectedMotionState.toArray(),
        newState.getPathStateParam().getValue().getMotionState()
            .toArray(), 1e-5);
  }

  /**
   * Test a prediction that moves from off-road to on-road.
   */
  @Test
  public void testUpdate6() {
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
    final GenericJTSGraph graph = new GenericJTSGraph(edges, false);

    final GpsObservation obs =
        new GpsObservation("test", new Date(0l), edges.get(0)
            .getCoordinate(), null, null, null, 0, null,
            new ProjectedCoordinate(null, edges.get(0)
                .getCoordinate(), null));

    final Random rng = new Random(102343292l);

    final VehicleStateInitialParameters parameters =
        new VehicleStateInitialParameters(VectorFactory.getDefault()
            .copyArray(new double[] { 0d, 1d, 0d, 2d }),
            VectorFactory.getDefault().createVector2D(0.1d, 0.1d),
            Integer.MAX_VALUE, VectorFactory.getDefault()
                .createVector1D(0d), Integer.MAX_VALUE,
            /*
             * No error, so the bootstrap sampling doesn't affect the sampled
             * edge results.
             * Same with the transition prior probabilities: make sure it stays
             * on-road.
             */
            VectorFactory.getDefault().createVector2D(0d, 0d),
            Integer.MAX_VALUE, VectorFactory.getDefault()
                .createVector2D(1, Double.MAX_VALUE), VectorFactory
                .getDefault().createVector2D(Double.MAX_VALUE, 1), 0,
            1, 0);

    final PathEdge startPathEdge = PathEdge.nullPathEdge;

    final VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph> factory =
        new VehicleStateDistribution.VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph>();
    final VehicleStateDistribution<GpsObservation> currentState =
        factory.createInitialVehicleState(parameters, graph, obs,
            rng, startPathEdge);

    final VehicleStateBootstrapUpdater<GpsObservation> updater =
        new VehicleStateBootstrapUpdater<GpsObservation>(obs, graph,
            parameters, rng);

    final VehicleStateDistribution<GpsObservation> newState =
        updater.update(currentState);

    final InferenceGraphEdge expectedEdge =
        Iterables.getOnlyElement(
            graph.getNearbyEdges(new Coordinate(1d, 2d), 0.5d))
            .getParentEdge();

    final InferenceGraphEdge actualEdge =
        newState.getPathStateParam().getValue().getEdge()
            .getInferenceGraphEdge();
    AssertJUnit.assertEquals(expectedEdge, actualEdge);

    final Vector expectedMotionState =
        VectorFactory.getDefault().copyArray(
            new double[] { 1d, Math.sqrt(5) });

    ArrayAsserts.assertArrayEquals(expectedMotionState.toArray(),
        newState.getPathStateParam().getValue().getMotionState()
            .toArray(), 1e-5);
  }
}
