package org.opentrackingtools.distributions;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Date;
import java.util.List;
import java.util.Random;
import java.util.Set;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.geotools.referencing.operation.builder.MathTransformBuilder;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.MathTransform2D;
import org.opengis.referencing.operation.MathTransformFactory;
import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.graph.GenericJTSGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.graph.InferenceGraphSegment;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.ProjectedCoordinate;
import org.opentrackingtools.model.SimpleBayesianParameter;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.paths.PathEdge;
import org.testng.AssertJUnit;
import org.testng.annotations.Test;

import com.beust.jcommander.internal.Lists;
import com.google.common.base.Predicate;
import com.google.common.collect.Iterables;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.linearref.LengthLocationMap;

public class OnOffEdgeTransDistributionTest {

  /**
   * Test that we get the correct edge for the motion state
   * we provide.  In this case, the motion state says we're
   * 4 meters away from our initial edge, so return the only
   * edge that far away
   */
  @Test
  public void getDomain1() {
    
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
        VectorFactory.getDefault().createVector1D(6.25e-4), 0, 
        VectorFactory.getDefault().createVector2D(6.25e-4, 6.25e-4), 0, 
        VectorFactory.getDefault().createVector2D(10, 50), 
        VectorFactory.getDefault().createVector2D(50, 10), 0, 30, 0);
    
    InferenceGraphEdge startEdge = startLine.getParentEdge();
    PathEdge startPathEdge = new PathEdge(startLine, 0d, false);
    VehicleStateDistribution<GpsObservation> currentState = 
        VehicleStateDistribution.constructInitialVehicleState(parameters, graph, obs, rng, startPathEdge);
    
    
    OnOffEdgeTransDistribution edgeTransDist = currentState.getEdgeTransitionParam().getConditionalDistribution();
    
    /*
     * Now we move our path state forward, by hand
     */
    edgeTransDist.setMotionState(VectorFactory.getDefault().createVector2D(4d, 1d));
    
    Set<InferenceGraphEdge> transitionSupport = edgeTransDist.getDomain();
    
    InferenceGraphEdge expectedEdge = Iterables.getOnlyElement(graph.getNearbyEdges(new Coordinate(1d,3d), 1d)).getParentEdge();
    
    /*
     * Grab the only non-null edge.
     */
    InferenceGraphEdge actualEdge = Iterables.find(transitionSupport, new Predicate<InferenceGraphEdge>() {
      @Override
      public boolean apply(InferenceGraphEdge input) {
        return !input.isNullEdge();
      }
    });
    AssertJUnit.assertEquals(expectedEdge, actualEdge);
  }
  
  /**
   * In this case, the motion state says we're 4 meters away from 
   * our initial edge, however, there are no edges for that distance
   * so we should return only off-road.
   * 
   */
  @Test
  public void getDomain2() {
    
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
    GenericJTSGraph graph = new GenericJTSGraph(edges, false);
    InferenceGraphSegment startLine = Iterables.getOnlyElement(graph.getNearbyEdges(edges.get(0).getCoordinate(), 1d));
    
    GpsObservation obs = new GpsObservation("test", new Date(0l), edges.get(0).getCoordinate(), null, null, null, 0, null, 
        new ProjectedCoordinate(null, edges.get(0).getCoordinate(), null));
    
    Random rng = new Random(102343292l);
    
    VehicleStateInitialParameters parameters = new VehicleStateInitialParameters(
        null,
        VectorFactory.getDefault().createVector2D(100, 100), 0, 
        VectorFactory.getDefault().createVector1D(6.25e-4), 0, 
        VectorFactory.getDefault().createVector2D(6.25e-4, 6.25e-4), 0, 
        VectorFactory.getDefault().createVector2D(10, 50), 
        VectorFactory.getDefault().createVector2D(50, 10), 0, 30, 0);
    
    InferenceGraphEdge startEdge = startLine.getParentEdge();
    PathEdge startPathEdge = new PathEdge(startLine, 0d, false);
    VehicleStateDistribution<GpsObservation> currentState = VehicleStateDistribution.constructInitialVehicleState(parameters, graph, obs, rng, startPathEdge);
    
    OnOffEdgeTransDistribution edgeTransDist = currentState.getEdgeTransitionParam().getConditionalDistribution();
    
    /*
     * Now we move our path state forward, by hand
     */
    edgeTransDist.setMotionState(VectorFactory.getDefault().createVector2D(4d, 1d));
    
    Set<InferenceGraphEdge> transitionSupport = edgeTransDist.getDomain();
    
    /*
     * We expect to have only the null edge.
     */
    InferenceGraphEdge actualEdge = Iterables.getOnlyElement(transitionSupport, null); 
    AssertJUnit.assertEquals(InferenceGraphEdge.nullGraphEdge, actualEdge);
  }
  
  /**
   * In this case, our motion state starts us off in the middle
   * of the edge and doesn't change, so we should get our initial
   * edge back (along with the null-edge, naturally). 
   */
  @Test
  public void getDomain3() {
    
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
        VectorFactory.getDefault().createVector1D(6.25e-4), 0, 
        VectorFactory.getDefault().createVector2D(6.25e-4, 6.25e-4), 0, 
        VectorFactory.getDefault().createVector2D(10, 50), 
        VectorFactory.getDefault().createVector2D(50, 10), 0, 30, 0);
    
    InferenceGraphEdge startEdge = startLine.getParentEdge();
    PathEdge startPathEdge = new PathEdge(startLine, 0d, false);
    VehicleStateDistribution<GpsObservation> currentState = VehicleStateDistribution.constructInitialVehicleState(parameters, graph, obs, rng, startPathEdge);
    
    OnOffEdgeTransDistribution edgeTransDist = currentState.getEdgeTransitionParam().getConditionalDistribution();
    
    /*
     * Now we move our path state forward, by hand
     */
    edgeTransDist.setMotionState(VectorFactory.getDefault().createVector2D(0.5d, 1d));
    
    Set<InferenceGraphEdge> transitionSupport = edgeTransDist.getDomain();
    
    /*
     * Grab the only non-null edge.
     */
    InferenceGraphEdge actualEdge = Iterables.find(transitionSupport, new Predicate<InferenceGraphEdge>() {
      @Override
      public boolean apply(InferenceGraphEdge input) {
        return !input.isNullEdge();
      }
    });
    AssertJUnit.assertEquals(startEdge, actualEdge);
  }
}
