package org.opentrackingtools.updater;

import gov.sandia.cognition.math.matrix.VectorFactory;

import java.util.Date;
import java.util.List;
import java.util.Random;

import org.apache.commons.lang.builder.ToStringBuilder;
import org.apache.commons.lang.builder.ToStringStyle;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.graph.GenericJTSGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.graph.InferenceGraphSegment;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.ProjectedCoordinate;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.paths.PathEdge;
import org.testng.AssertJUnit;
import org.testng.annotations.Test;

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineString;

public class VehicleStatePLUpdaterTest {

  static {
    ToStringBuilder.setDefaultStyle(ToStringStyle.SHORT_PREFIX_STYLE);
  }

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

    final GpsObservation obs =
        new GpsObservation("test", new Date(0l), edges.get(0)
            .getCoordinate(), null, null, null, 0, null,
            new ProjectedCoordinate(null, edges.get(0)
                .getCoordinate(), null));

    final Random rng = new Random(102343292l);

    final VehicleStateInitialParameters parameters =
        new VehicleStateInitialParameters(VectorFactory.getDefault()
            .copyArray(new double[] { 0.4d, 1d / 40d, 0d, 0d }),
            VectorFactory.getDefault().createVector2D(0.02d, 0.02d), 0,
            VectorFactory.getDefault().createVector1D(1e-4d), 0,
            VectorFactory.getDefault().createVector2D(1e-4d, 1e-4d), 0,
            VectorFactory.getDefault().createVector2D(1,
                Double.MAX_VALUE), VectorFactory.getDefault()
                .createVector2D(Double.MAX_VALUE, 1), 0, 4, 0);

    startLine.getParentEdge();
    final PathEdge startPathEdge = new PathEdge(startLine, 0d, false);
    final VehicleStateDistribution<GpsObservation> currentState =
        VehicleStateDistribution.constructInitialVehicleState(
            parameters, graph, obs, rng, startPathEdge);

    final VehicleStateBootstrapUpdater<GpsObservation> updater =
        new VehicleStateBootstrapUpdater<GpsObservation>(obs, graph,
            parameters, rng);

    final VehicleStateDistribution<GpsObservation> newState =
        updater.update(currentState);

    final InferenceGraphSegment expectedSegment = startLine;

    AssertJUnit.assertEquals(expectedSegment.getLine(), newState
        .getPathStateParam().getValue().getEdge().getLine());
  }

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

    final Coordinate obsCoord = new Coordinate(1, 3);
    final GpsObservation obs =
        new GpsObservation("test", new Date(0l), obsCoord, null, null, null, 0, null,
            new ProjectedCoordinate(null, obsCoord, null));

    final Random rng = new Random(102343292l);

    final VehicleStateInitialParameters parameters =
        new VehicleStateInitialParameters(VectorFactory.getDefault()
            .copyArray(new double[] { 0d, 1d, 0d, 0d }),
            VectorFactory.getDefault().createVector2D(0.02d, 0.02d), 0,
            VectorFactory.getDefault().createVector1D(1e-4d), 0,
            VectorFactory.getDefault().createVector2D(1e-4d, 1e-4d), 0,
            VectorFactory.getDefault().createVector2D(1,
                Double.MAX_VALUE), VectorFactory.getDefault()
                .createVector2D(Double.MAX_VALUE, 1), 0, 4, 0);

    startLine.getParentEdge();
    final PathEdge startPathEdge = new PathEdge(startLine, 0d, false);
    final VehicleStateDistribution<GpsObservation> currentState =
        VehicleStateDistribution.constructInitialVehicleState(
            parameters, graph, obs, rng, startPathEdge);

    final VehicleStatePLUpdater<GpsObservation> updater =
        new VehicleStatePLUpdater<GpsObservation>(obs, graph,
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
}
