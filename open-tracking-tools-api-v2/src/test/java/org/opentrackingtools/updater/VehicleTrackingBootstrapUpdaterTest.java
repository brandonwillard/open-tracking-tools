package org.opentrackingtools.updater;

import gov.sandia.cognition.math.matrix.VectorFactory;

import java.util.Date;
import java.util.List;
import java.util.Random;
import java.util.Set;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.distributions.OnOffEdgeTransDistribution;
import org.opentrackingtools.graph.GenericJTSGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.graph.InferenceGraphSegment;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.ProjectedCoordinate;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.PathEdge;
import org.testng.AssertJUnit;
import org.testng.annotations.Test;

import com.google.common.base.Predicate;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineString;

public class VehicleTrackingBootstrapUpdaterTest {

  /**
   * Test a prediction that leaves us on the same edge
   * as the one we started on.
   */
  @Test
  public void testUpdate1() {
    throw new RuntimeException("Test not implemented");
  }
  
  /**
   * Test a prediction that moves forward three edges, where 
   * forward edges exist.
   */
  @Test
  public void testUpdate2() {
    List<LineString> edges = Lists.newArrayList();
    edges.add(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {
           new Coordinate(0,0), 
           new Coordinate(1,0), 
        }));
    edges.add(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {
           new Coordinate(1,0), 
           new Coordinate(1,1), 
        }));
    edges.add(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {
           new Coordinate(1,0), 
           new Coordinate(1,-1), 
        }));
    edges.add(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {
           new Coordinate(1,1), 
           new Coordinate(1,2), 
        }));
    edges.add(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {
           new Coordinate(1,2), 
           new Coordinate(1,3), 
        }));
    GenericJTSGraph graph = new GenericJTSGraph(edges, false);
    InferenceGraphSegment startLine = Iterables.getOnlyElement(graph.getNearbyEdges(edges.get(0).getCoordinate(), 1d));
    
    GpsObservation obs = new GpsObservation("test", new Date(0l), edges.get(0).getCoordinate(), null, null, null, 0, null, 
        new ProjectedCoordinate(null, edges.get(0).getCoordinate(), null));
    
    Random rng = new Random(102343292l);
    
    VehicleStateInitialParameters parameters = new VehicleStateInitialParameters(
        null,
        VectorFactory.getDefault().createVector2D(100, 100), 0, 
        VectorFactory.getDefault().createVector1D(0d), 0, 
        /*
         * No error, so the bootstrap sampling doesn't affect the sampled
         * edge results.
         * Same with the transition prior probabilities: make sure it stays
         * on-road.
         */
        VectorFactory.getDefault().createVector2D(0d, 0d), 0, 
        VectorFactory.getDefault().createVector2D(1, Double.MAX_VALUE), 
        VectorFactory.getDefault().createVector2D(Double.MAX_VALUE, 1), 0, 30, 0);
    
    InferenceGraphEdge startEdge = startLine.getParentEdge();
    PathEdge startPathEdge = new PathEdge(startLine, 0d, false);
    VehicleState<GpsObservation> currentState = VehicleState.constructInitialVehicleState(parameters, graph, obs, rng, startPathEdge);
    
    VehicleTrackingBootstrapUpdater<GpsObservation> updater = new VehicleTrackingBootstrapUpdater<GpsObservation>(obs, graph, parameters, rng);
    
    VehicleState<GpsObservation> newState = updater.update(currentState);
    
    
    InferenceGraphEdge expectedEdge = Iterables.getOnlyElement(graph.getNearbyEdges(new Coordinate(1d,3d), 1d)).getParentEdge();
    
    AssertJUnit.assertEquals(expectedEdge, newState.getPathStateParam().getValue().getEdge().getInferenceGraphEdge());
  }
  
  /**
   * Test a prediction that moves forward three edges, where 
   * no third forward edge exists.
   */
  @Test
  public void testUpdate3() {
    throw new RuntimeException("Test not implemented");
  }
  
  /**
   * Test a prediction that moves from an edge to off-road.
   */
  @Test
  public void testUpdate4() {
    throw new RuntimeException("Test not implemented");
  }
  
  /**
   * Test a prediction that moves from off-road to off-road.
   */
  @Test
  public void testUpdate5() {
    throw new RuntimeException("Test not implemented");
  }
  
  /**
   * Test a prediction that moves from off-road to on-road.
   */
  @Test
  public void testUpdate6() {
    throw new RuntimeException("Test not implemented");
  }
}
