package org.opentrackingtools.distributions;

import gov.sandia.cognition.math.matrix.VectorFactory;

import java.util.Date;
import java.util.List;
import java.util.Random;
import java.util.Set;

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

import com.beust.jcommander.internal.Lists;
import com.google.common.base.Predicate;
import com.google.common.collect.Iterables;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineString;

public class OnOffEdgeTransDistributionTest {

  /**
   * Test that we get the correct edge for the motion state we provide. In this
   * case, the motion state says we're 4 meters away from our initial edge, so
   * return the only edge that far away
   */
  @Test
  public void getDomain1() {

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
            .getCoordinate(), 1d));

    final GpsObservation obs =
        new GpsObservation("test", new Date(0l), edges.get(0)
            .getCoordinate(), null, null, null, 0, null,
            new ProjectedCoordinate(null, edges.get(0)
                .getCoordinate(), null));

    final Random rng = new Random(102343292l);

    final VehicleStateInitialParameters parameters =
        new VehicleStateInitialParameters(null, VectorFactory
            .getDefault().createVector2D(100, 100),
            Integer.MAX_VALUE, VectorFactory.getDefault()
                .createVector1D(6.25e-4), Integer.MAX_VALUE,
            VectorFactory.getDefault().createVector2D(6.25e-4,
                6.25e-4), Integer.MAX_VALUE, VectorFactory
                .getDefault().createVector2D(10, 50), VectorFactory
                .getDefault().createVector2D(50, 10), 0, 30, 0);

    final PathEdge startPathEdge = new PathEdge(startLine, 0d, false);

    final VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph> factory =
        new VehicleStateDistribution.VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph>();
    final VehicleStateDistribution<GpsObservation> currentState =
        factory.createInitialVehicleState(parameters, graph, obs,
            rng, startPathEdge);

    final OnOffEdgeTransDistribution edgeTransDist =
        currentState.getEdgeTransitionParam()
            .getConditionalDistribution();

    /*
     * Now we move our path state forward, by hand
     */
    edgeTransDist.setMotionState(VectorFactory.getDefault()
        .createVector2D(4d, 1d));

    final Set<InferenceGraphSegment> transitionSupport =
        edgeTransDist.getDomain();

    final InferenceGraphEdge expectedEdge =
        Iterables.getOnlyElement(
            graph.getNearbyEdges(new Coordinate(1d, 3d), 1d));

    /*
     * Grab the only non-null edge.
     */
    final InferenceGraphSegment actualEdge =
        Iterables.find(transitionSupport,
            new Predicate<InferenceGraphSegment>() {
              @Override
              public boolean apply(InferenceGraphSegment input) {
                return !input.isNullEdge();
              }
            });
    AssertJUnit.assertEquals(expectedEdge, actualEdge);
  }

  /**
   * In this case, the motion state says we're 4 meters away from our initial
   * edge, however, there are no edges for that distance so we should return
   * only off-road.
   * 
   */
  @Test
  public void getDomain2() {

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
            .getCoordinate(), 1d));

    final GpsObservation obs =
        new GpsObservation("test", new Date(0l), edges.get(0)
            .getCoordinate(), null, null, null, 0, null,
            new ProjectedCoordinate(null, edges.get(0)
                .getCoordinate(), null));

    final Random rng = new Random(102343292l);

    final VehicleStateInitialParameters parameters =
        new VehicleStateInitialParameters(null, VectorFactory
            .getDefault().createVector2D(100, 100),
            Integer.MAX_VALUE, VectorFactory.getDefault()
                .createVector1D(6.25e-4), Integer.MAX_VALUE,
            VectorFactory.getDefault().createVector2D(6.25e-4,
                6.25e-4), Integer.MAX_VALUE, VectorFactory
                .getDefault().createVector2D(10, 50), VectorFactory
                .getDefault().createVector2D(50, 10), 0, 30, 0);

    final PathEdge startPathEdge = new PathEdge(startLine, 0d, false);

    final VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph> factory =
        new VehicleStateDistribution.VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph>();
    final VehicleStateDistribution<GpsObservation> currentState =
        factory.createInitialVehicleState(parameters, graph, obs,
            rng, startPathEdge);

    final OnOffEdgeTransDistribution edgeTransDist =
        currentState.getEdgeTransitionParam()
            .getConditionalDistribution();

    /*
     * Now we move our path state forward, by hand
     */
    edgeTransDist.setMotionState(VectorFactory.getDefault()
        .createVector2D(4d, 1d));

    final Set<InferenceGraphSegment> transitionSupport =
        edgeTransDist.getDomain();

    /*
     * We expect to have only the null edge.
     */
    final InferenceGraphEdge actualEdge =
        Iterables.getOnlyElement(transitionSupport, null);
    AssertJUnit.assertEquals(InferenceGraphSegment.nullGraphSegment,
        actualEdge);
  }

  /**
   * In this case, our motion state starts us off in the middle of the edge and
   * doesn't change, so we should get our initial edge back (along with the
   * null-edge, naturally).
   */
  @Test
  public void getDomain3() {

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
            .getCoordinate(), 1d));

    final GpsObservation obs =
        new GpsObservation("test", new Date(0l), edges.get(0)
            .getCoordinate(), null, null, null, 0, null,
            new ProjectedCoordinate(null, edges.get(0)
                .getCoordinate(), null));

    final Random rng = new Random(102343292l);

    final VehicleStateInitialParameters parameters =
        new VehicleStateInitialParameters(null, VectorFactory
            .getDefault().createVector2D(100, 100),
            Integer.MAX_VALUE, VectorFactory.getDefault()
                .createVector1D(6.25e-4), Integer.MAX_VALUE,
            VectorFactory.getDefault().createVector2D(6.25e-4,
                6.25e-4), Integer.MAX_VALUE, VectorFactory
                .getDefault().createVector2D(10, 50), VectorFactory
                .getDefault().createVector2D(50, 10), 0, 30, 0);

    final InferenceGraphEdge startEdge = startLine;
    final PathEdge startPathEdge = new PathEdge(startLine, 0d, false);

    final VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph> factory =
        new VehicleStateDistribution.VehicleStateDistributionFactory<GpsObservation, GenericJTSGraph>();
    final VehicleStateDistribution<GpsObservation> currentState =
        factory.createInitialVehicleState(parameters, graph, obs,
            rng, startPathEdge);

    final OnOffEdgeTransDistribution edgeTransDist =
        currentState.getEdgeTransitionParam()
            .getConditionalDistribution();

    /*
     * Now we move our path state forward, by hand
     */
    edgeTransDist.setMotionState(VectorFactory.getDefault()
        .createVector2D(0.5d, 1d));

    final Set<InferenceGraphSegment> transitionSupport =
        edgeTransDist.getDomain();

    /*
     * Grab the only non-null edge.
     */
    final InferenceGraphSegment actualEdge =
        Iterables.find(transitionSupport,
            new Predicate<InferenceGraphSegment>() {
              @Override
              public boolean apply(InferenceGraphSegment input) {
                return !input.isNullEdge();
              }
            });
    AssertJUnit.assertEquals(startEdge, actualEdge);
  }
}
