package org.opentrackingtools.impl;

import org.testng.annotations.BeforeTest;
import org.testng.annotations.DataProvider;
import org.testng.annotations.Test;
import org.testng.AssertJUnit;
import org.testng.annotations.Test;
import org.testng.AssertJUnit;
import org.geotools.data.simple.SimpleFeatureSource;
import org.geotools.factory.FactoryRegistryException;
import org.geotools.feature.FeatureIterator;
import org.geotools.geometry.jts.ReferencedEnvelope;
import org.geotools.grid.Lines;
import org.geotools.grid.ortholine.LineOrientation;
import org.geotools.grid.ortholine.OrthoLineDef;
import org.geotools.referencing.CRS;

import java.io.IOException;
import java.util.Arrays;
import java.util.Date;
import java.util.List;



import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import org.opengis.feature.Feature;
import org.opengis.referencing.FactoryException;
import org.opengis.referencing.NoSuchAuthorityCodeException;
import org.opengis.referencing.crs.CoordinateReferenceSystem;
import org.opengis.referencing.operation.NoninvertibleTransformException;
import org.opengis.referencing.operation.TransformException;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.impl.GenericJTSGraph;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.impl.Simulation.SimulationParameters;
import org.opentrackingtools.statistics.distributions.impl.OnOffEdgeTransDirMulti;
import org.opentrackingtools.statistics.filters.vehicles.impl.VehicleTrackingBootstrapFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.util.GeoUtils;

import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.LineString;

public class SimulationTest {
  
  private InferenceGraph graph;
  private Coordinate startCoord;
  private Matrix avgTransform;

  @BeforeTest
  public void setUp() throws NoSuchAuthorityCodeException, FactoryRegistryException, FactoryException, IOException {
    List<LineString> edges = Lists.newArrayList();
    
    /*
     * Create a grid
     */
    CoordinateReferenceSystem crs = 
        CRS.getAuthorityFactory(true).createGeographicCRS("EPSG:4326");
    
    startCoord = new Coordinate(40.7549, -73.97749);
    
    Envelope tmpEnv = new Envelope(startCoord);
    
    final double appMeter = GeoUtils.getMetersInAngleDegrees(1d);
    tmpEnv.expandBy(appMeter * 10e3);
    
    ReferencedEnvelope gridBounds = new ReferencedEnvelope(
       tmpEnv.getMinX() ,tmpEnv.getMaxX() , 
       tmpEnv.getMinY() , tmpEnv.getMaxY(), crs);
//        40.7012, 40.8086, -74.020, -73.935, crs);
    
    List<OrthoLineDef> lineDefs = Arrays.asList(
        // vertical (longitude) lines
//        new OrthoLineDef(LineOrientation.VERTICAL, 2, appMeter * 100d),
        new OrthoLineDef(LineOrientation.VERTICAL, 1, appMeter * 100d),

        // horizontal (latitude) lines
//        new OrthoLineDef(LineOrientation.HORIZONTAL, 2, appMeter * 100d),
        new OrthoLineDef(LineOrientation.HORIZONTAL, 1, appMeter * 100d)
        );

    SimpleFeatureSource grid = Lines.createOrthoLines(gridBounds, lineDefs);
    FeatureIterator iter = grid.getFeatures().features();
    
    while (iter.hasNext()) {
      Feature feature = iter.next();
      LineString geom = (LineString)feature.getDefaultGeometryProperty().getValue();
      edges.add(geom);
      /*
       * Add the reverse so that there are no dead-ends to mess with 
       * the test results
       */
      edges.add((LineString) geom.reverse());
    }
    
    graph = new GenericJTSGraph(edges);
    
    avgTransform = MatrixFactory.getDefault().copyArray(
        new double[][] {
            {1, 0, 1, 0},
            {0, 1, 0, 1}
        }).scale(1d/2d);
  }
  
  
  @DataProvider
  private static final Object[][] initialStateData() {
    return new Object[][] {
        {
          /*
           * Road only
           */
        new VehicleStateInitialParameters(VectorFactory
            .getDefault().createVector2D(100d, 100d), 20,
            VectorFactory.getDefault().createVector1D(
                6.25e-4), 30, VectorFactory.getDefault()
                .createVector2D(6.25e-4, 6.25e-4), 20,
            VectorFactory.getDefault().createVector2D(1d,
                Double.MAX_VALUE), VectorFactory.getDefault()
                .createVector2D(Double.MAX_VALUE, 1d),
            VehicleTrackingBootstrapFilter.class.getName(), 25,
            15, 2159585l),
        Boolean.FALSE,
            66000
        },
        {
          /*
           * Ground only
           */
        new VehicleStateInitialParameters(VectorFactory
            .getDefault().createVector2D(100d, 100d), 20,
            VectorFactory.getDefault().createVector1D(
                6.25e-4), 30, VectorFactory.getDefault()
                .createVector2D(6.25e-5, 6.25e-5), 20,
            VectorFactory.getDefault().createVector2D(
                Double.MAX_VALUE, 1d), 
            VectorFactory.getDefault().createVector2D(
                1d, Double.MAX_VALUE),
            VehicleTrackingBootstrapFilter.class.getName(), 25,
            10, 215955l),
        Boolean.FALSE,
            66000
        }, 
        {
          /*
           * Mixed 
           */
        new VehicleStateInitialParameters(VectorFactory
            .getDefault().createVector2D(100d, 100d), 20,
            VectorFactory.getDefault().createVector1D(
                6.25e-4), 30, VectorFactory.getDefault()
                .createVector2D(6.25e-4, 6.25e-4), 20,
            VectorFactory.getDefault().createVector2D(
                1d, 1d), 
            VectorFactory.getDefault().createVector2D(
                1d, 1d),
            VehicleTrackingBootstrapFilter.class.getName(), 25,
            15, 21595857l), 
            Boolean.TRUE,
            46000
        }
    };
  }
  
  @Test(dataProvider="initialStateData")
  public void runSimulation(VehicleStateInitialParameters vehicleStateInitialParams,
    boolean generalizeMoveDiff, long duration) throws NoninvertibleTransformException, TransformException {
    
    SimulationParameters simParams = new SimulationParameters(
        startCoord, new Date(0l), duration, 15, false, vehicleStateInitialParams);
    
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
    
    double[] obsErrorZeroArray = null;
    double[] movementZeroArray = null;
    
    final long approxRuns = sim.getSimParameters().getDuration()/
        sim.getSimParameters().getFrequency();
    
    do {
      
      final Vector obsError = vehicleState.getObservation().getProjectedPoint().
          minus(vehicleState.getMeanLocation());
      obsErrorSS.update(obsError);
      
      System.out.println("obsError=" + obsErrorSS.getMean());
      
      final VehicleState parentState = vehicleState.getParentState();
      if (parentState != null) {
        PathStateBelief predictedState = parentState.getMovementFilter()
            .predict(parentState.getBelief(), vehicleState.getBelief().getPath());
        
        final Vector movementDiff = vehicleState.getBelief().minus(
           predictedState);
             
        if (generalizeMoveDiff && movementDiff.getDimensionality() == 4) {
          final Vector movementDiffAvg = avgTransform.times(movementDiff);
          movementSS.update(movementDiffAvg);
        } else {
          movementSS.update(movementDiff);
        }
        System.out.println("movementMean=" + movementSS.getMean());
        
        Vector transType = OnOffEdgeTransDirMulti.getTransitionType(
            parentState.getBelief().getEdge().getInferredEdge(), 
            vehicleState.getBelief().getEdge().getInferredEdge());
        
        if (parentState.getBelief().isOnRoad()) {
          transType = transType.stack(AbstractRoadTrackingFilter.zeros2D);
        } else {
          transType = AbstractRoadTrackingFilter.zeros2D.stack(transType);
        }
        
        transitionsSS.update(transType);
        System.out.println("transitionsMean=" + transitionsSS.getMean());
      }
      
      if (movementSS.getCount() > Math.min(approxRuns/16, 25)) {
        if (obsErrorZeroArray == null)
          obsErrorZeroArray = VectorFactory.getDefault()
            .createVector(obsErrorSS.getMean().getDimensionality())
            .toArray();
        
        AssertJUnit.assertArrayEquals(obsErrorZeroArray,
          obsErrorSS.getMean().toArray(), 5);
      
        if (movementZeroArray == null)
          movementZeroArray = VectorFactory.getDefault()
            .createVector(movementSS.getMean().getDimensionality())
            .toArray();
        
        AssertJUnit.assertArrayEquals(movementZeroArray,
        movementSS.getMean().toArray(), 6);
      }
      
      vehicleState = sim.stepSimulation(vehicleState);
      time = vehicleState.getObservation().getTimestamp().getTime();
      
    } while (time < sim.getSimParameters().getEndTime().getTime());
    

    System.out.println(obsErrorSS.getMean());
    System.out.println(movementSS.getMean());
    System.out.println(transitionsSS.getMean());
    
    
  }
  
}
