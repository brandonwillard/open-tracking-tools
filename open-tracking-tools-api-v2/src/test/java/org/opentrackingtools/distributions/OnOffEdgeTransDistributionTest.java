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
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.ProjectedCoordinate;
import org.opentrackingtools.model.SimpleBayesianParameter;
import org.opentrackingtools.model.VehicleState;
import org.testng.AssertJUnit;
import org.testng.annotations.Test;

import com.beust.jcommander.internal.Lists;
import com.google.common.collect.Iterables;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineString;

public class OnOffEdgeTransDistributionTest {

  @Test
  public void getDomain() {
    
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
    GenericJTSGraph graph = new GenericJTSGraph(edges);
    InferenceGraphEdge startEdge = Iterables.getOnlyElement(graph.getNearbyEdges(edges.get(0).getCoordinate(), 1d));
    
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
    
    VehicleState<GpsObservation> currentState = VehicleState.constructInitialVehicleState(parameters, graph, obs, rng, startEdge);
    
    /*
     * Now we move our vehicle state forward, by hand
     */
    currentState.getMotionStateParam().getParameterPrior().setMean(
        VectorFactory.getDefault().createVector2D(4d, 1d));
    
    OnOffEdgeTransDistribution edgeTransDist = new OnOffEdgeTransDistribution(currentState, startEdge,
        parameters.getOnTransitionProbsPrior(), parameters.getOffTransitionProbsPrior());
    
    Set<InferenceGraphEdge> transitionSupport = edgeTransDist.getDomain();
    
    InferenceGraphEdge expectedEdge = Iterables.getOnlyElement(graph.getNearbyEdges(new Coordinate(1d,3d), 1d));
    AssertJUnit.assertEquals(expectedEdge, Iterables.getOnlyElement(transitionSupport, null));
  }
}
