package org.opentrackingtools.impl;

import org.geotools.coverage.grid.GeneralGridEnvelope;
import org.geotools.data.simple.SimpleFeatureCollection;
import org.geotools.data.simple.SimpleFeatureSource;
import org.geotools.feature.FeatureIterator;
import org.geotools.geometry.Envelope2D;
import org.geotools.geometry.jts.JTS;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.geotools.geometry.jts.ReferencedEnvelope;
import org.geotools.grid.Grids;
import org.geotools.grid.Lines;
import org.geotools.grid.ortholine.LineOrientation;
import org.geotools.grid.ortholine.OrthoLineDef;
import org.junit.Test;

import java.io.IOException;
import java.util.Arrays;
import java.util.Date;
import java.util.List;



import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import org.opengis.feature.Feature;
import org.opengis.feature.simple.SimpleFeature;
import org.opengis.geometry.BoundingBox;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.impl.GenericJTSGraph;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.impl.Simulation.SimulationParameters;
import org.opentrackingtools.statistics.distributions.impl.OnOffEdgeTransDirMulti;
import org.opentrackingtools.statistics.filters.vehicles.impl.VehicleTrackingBootstrapFilter;
import org.opentrackingtools.util.GeoUtils;

import com.google.common.collect.Iterators;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.LineString;

public class SimulationTest {
  
  @Test
  public void testRoadOnlySimulation() throws IOException {
    
    final long seed = 2159585l;
    
    VehicleStateInitialParameters vehicleStateInitialParams =
        new VehicleStateInitialParameters(VectorFactory
            .getDefault().createVector2D(100d, 100d), 20,
            VectorFactory.getDefault().createVector1D(
                6.25e-4), 30, VectorFactory.getDefault()
                .createVector2D(6.25e-4, 6.25e-4), 20,
            VectorFactory.getDefault().createVector2D(1d,
                Double.MAX_VALUE), VectorFactory.getDefault()
                .createVector2D(Double.MAX_VALUE, 1d),
            VehicleTrackingBootstrapFilter.class.getName(), 25,
            15, seed);
    
    List<LineString> edges = Lists.newArrayList();
    
    /*
     * Create a grid
     */
    final Coordinate startCoord = new Coordinate(40, -70);
//    Vector projStart = GeoUtils.getEuclideanVectorFromLatLon(startCoord);
    Envelope startEnvelope = new Envelope(startCoord);
    startEnvelope.expandBy(20);
    ReferencedEnvelope gridBounds = new ReferencedEnvelope(
        startEnvelope.getMaxX(), startEnvelope.getMaxY(),
        startEnvelope.getMinX(), startEnvelope.getMinY(),
        org.geotools.referencing.crs.DefaultGeographicCRS.WGS84);
    List<OrthoLineDef> lineDefs = Arrays.asList(
        // vertical (longitude) lines
        new OrthoLineDef(LineOrientation.VERTICAL, 2, 10.0),
        new OrthoLineDef(LineOrientation.VERTICAL, 1, 2.0),

        // horizontal (latitude) lines
        new OrthoLineDef(LineOrientation.HORIZONTAL, 2, 10.0),
        new OrthoLineDef(LineOrientation.HORIZONTAL, 1, 2.0));

    // Specify vertex spacing to get "densified" polygons
    double vertexSpacing = 0.1;
    SimpleFeatureSource grid = Lines.createOrthoLines(gridBounds, lineDefs, vertexSpacing);
    FeatureIterator iter = grid.getFeatures().features();
    
    while (iter.hasNext()) {
      Feature feature = iter.next();
      edges.add((LineString)feature.getDefaultGeometryProperty().getValue());
    }
    
    InferenceGraph graph = new GenericJTSGraph(edges);
    
    SimulationParameters simParams = new SimulationParameters(
        startCoord, 
        new Date(0l), 66000, 15, false, vehicleStateInitialParams);
    
    Simulation sim = new Simulation("test-sim", graph, simParams, 
        vehicleStateInitialParams);
    
    long time = sim.getSimParameters().getStartTime().getTime();
    
    VehicleState vehicleState = sim.computeInitialState();
    
    final MultivariateGaussian.SufficientStatistic obsErrorSS =
        new MultivariateGaussian.SufficientStatistic();
    final MultivariateGaussian.SufficientStatistic movementSS =
        new MultivariateGaussian.SufficientStatistic();
    final MultivariateGaussian.SufficientStatistic transitionsSS =
        new MultivariateGaussian.SufficientStatistic();
    
    do {
      System.out.println("obs=" + vehicleState.getObservation().getProjectedPoint());
      System.out.println("mean=" + vehicleState.getMeanLocation());
      
      final Vector obsError = vehicleState.getObservation().getProjectedPoint().
          minus(vehicleState.getMeanLocation());
      obsErrorSS.update(obsError);
      
      final VehicleState parentState = vehicleState.getParentState();
      if (parentState != null) {
        PathStateBelief predictedState = parentState.getMovementFilter()
            .predict(parentState.getBelief(), vehicleState.getBelief().getPath());
        
        final Vector movementDiff = vehicleState.getBelief().minus(
           predictedState);
           
        System.out.println("movementDiff=" + movementDiff);
        movementSS.update(movementDiff);
        System.out.println("movementMean=" + movementSS.getMean());
        
        transitionsSS.update(OnOffEdgeTransDirMulti.getTransitionType(
            parentState.getBelief().getEdge().getInferredEdge(), 
            vehicleState.getBelief().getEdge().getInferredEdge()));
      }
      
      vehicleState = sim.stepSimulation(vehicleState);
      time = vehicleState.getObservation().getTimestamp().getTime();
      
    } while (time < sim.getSimParameters().getEndTime().getTime());
    

    System.out.println(obsErrorSS.getMean());
    System.out.println(movementSS.getMean());
    System.out.println(transitionsSS.getMean());
    
    
    org.junit.Assert.assertArrayEquals(new double[] {0, 0},
      obsErrorSS.getMean().toArray(), 10);
    
    org.junit.Assert.assertArrayEquals(new double[] {0, 0},
      movementSS.getMean().toArray(), 10);
  }

  @Test
  public void testGroundOnlySimulation() {
    
    final long seed = 2159585l;
    
    VehicleStateInitialParameters vehicleStateInitialParams =
        new VehicleStateInitialParameters(VectorFactory
            .getDefault().createVector2D(100d, 100d), 20,
            VectorFactory.getDefault().createVector1D(
                6.25e-4), 30, VectorFactory.getDefault()
                .createVector2D(6.25e-4, 6.25e-4), 20,
            VectorFactory.getDefault().createVector2D(5d,
                95d), VectorFactory.getDefault()
                .createVector2D(95d, 5d),
            VehicleTrackingBootstrapFilter.class.getName(), 25,
            15, seed);
    
    List<LineString> edges = Lists.newArrayList();
    
//    edges.add(JTSFactoryFinder.getGeometryFactory().createLineString(
//        
//        ));
    
    InferenceGraph graph = new GenericJTSGraph(edges);
    
    SimulationParameters simParams = new SimulationParameters(
        new Coordinate(40, -70), 
        new Date(0l), 66000, 15, false, vehicleStateInitialParams);
    
    Simulation sim = new Simulation("test-sim", graph, simParams, 
        vehicleStateInitialParams);
    
    long time = sim.getSimParameters().getStartTime().getTime();
    
    VehicleState vehicleState = sim.computeInitialState();
    
    final MultivariateGaussian.SufficientStatistic obsErrorSS =
        new MultivariateGaussian.SufficientStatistic();
    final MultivariateGaussian.SufficientStatistic movementSS =
        new MultivariateGaussian.SufficientStatistic();
    final MultivariateGaussian.SufficientStatistic transitionsSS =
        new MultivariateGaussian.SufficientStatistic();
    
    do {
      System.out.println("obs=" + vehicleState.getObservation().getProjectedPoint());
      System.out.println("mean=" + vehicleState.getMeanLocation());
      
      final Vector obsError = vehicleState.getObservation().getProjectedPoint().
          minus(vehicleState.getMeanLocation());
      obsErrorSS.update(obsError);
      
      final VehicleState parentState = vehicleState.getParentState();
      if (parentState != null) {
        PathStateBelief predictedState = parentState.getMovementFilter()
            .predict(parentState.getBelief(), vehicleState.getBelief().getPath());
        
        final Vector movementDiff = vehicleState.getBelief().minus(
           predictedState);
           
        System.out.println("movementDiff=" + movementDiff);
        movementSS.update(movementDiff);
        System.out.println("movementMean=" + movementSS.getMean());
        
        transitionsSS.update(OnOffEdgeTransDirMulti.getTransitionType(
            parentState.getBelief().getEdge().getInferredEdge(), 
            vehicleState.getBelief().getEdge().getInferredEdge()));
      }
      
      vehicleState = sim.stepSimulation(vehicleState);
      time = vehicleState.getObservation().getTimestamp().getTime();
      
    } while (time < sim.getSimParameters().getEndTime().getTime());
    

    System.out.println(obsErrorSS.getMean());
    System.out.println(movementSS.getMean());
    System.out.println(transitionsSS.getMean());
    
    
    org.junit.Assert.assertArrayEquals(new double[] {0, 0},
      obsErrorSS.getMean().toArray(), 10);
    
    org.junit.Assert.assertArrayEquals(new double[] {0, 0, 0, 0},
      movementSS.getMean().toArray(), 10);
  }
  
}
