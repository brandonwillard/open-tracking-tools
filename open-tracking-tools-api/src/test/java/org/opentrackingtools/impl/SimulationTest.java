package org.opentrackingtools.impl;

import org.junit.Test;
import java.util.Date;
import java.util.List;



import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.impl.GenericJTSGraph;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.impl.Simulation.SimulationParameters;
import org.opentrackingtools.statistics.distributions.impl.OnOffEdgeTransDirMulti;
import org.opentrackingtools.statistics.filters.vehicles.impl.VehicleTrackingBootstrapFilter;

import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineString;

public class SimulationTest {
  
  private Simulation sim;
  private InferenceGraph graph;

//  @BeforeMethod
//  public void setUp() throws Exception {
//  }
  
  @Test
  public void testSimulation() {
    /*
     * Create a graph
     */
    
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
    
    graph = new GenericJTSGraph(edges);
    
    SimulationParameters simParams = new SimulationParameters(
        new Coordinate(40, -70), 
        new Date(0l), 360000, 15, false, vehicleStateInitialParams);
    
    sim = new Simulation("test-sim", graph, simParams, 
        vehicleStateInitialParams);
    
    long time = this.sim.getSimParameters().getStartTime().getTime();
    
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
      
      vehicleState = this.sim.stepSimulation(vehicleState);
      time = vehicleState.getObservation().getTimestamp().getTime();
      
    } while (time < this.sim.getSimParameters().getEndTime().getTime());
    

    System.out.println(obsErrorSS.getMean());
    System.out.println(movementSS.getMean());
    System.out.println(transitionsSS.getMean());
    
    
    org.junit.Assert.assertArrayEquals(new double[] {0, 0},
      obsErrorSS.getMean().toArray(), 10);
    
    org.junit.Assert.assertArrayEquals(new double[] {0, 0, 0, 0},
      movementSS.getMean().toArray(), 10);
  }
}
